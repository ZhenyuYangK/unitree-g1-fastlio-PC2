#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cctype>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "g1/g1_audio_client.hpp"
#include "g1_task_interfaces/action/speak.hpp"

namespace {
using Speak = g1_task_interfaces::action::Speak;
using GoalHandle = rclcpp_action::ServerGoalHandle<Speak>;
using Clock = std::chrono::steady_clock;

// BaseClient::Call waits on a condition variable. Its response subscription must
// keep spinning on another thread, including while PlayStop waits for a reply.
// This executor is never shared with the bridge's action callbacks.
class AudioBackend {
 public:
  AudioBackend() : client(std::make_shared<unitree::ros2::g1::AudioClient>()) {
    executor_.add_node(client);
    spin_thread_ = std::thread([this]() { executor_.spin(); });
  }

  ~AudioBackend() {
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  std::shared_ptr<unitree::ros2::g1::AudioClient> client;

 private:
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
};

class AudioBridge : public rclcpp::Node {
 public:
  AudioBridge() : Node("g1_audio_bridge") {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    dry_run_ = declare_parameter<bool>("dry_run", true, fixed);
    const auto configured_volume = declare_parameter<int64_t>("volume", -1, fixed);
    app_name_ = declare_parameter<std::string>("app_name", "g1_audio_bridge", fixed);
    if (configured_volume < -1 || configured_volume > 100) {
      throw std::invalid_argument("volume must be -1 (keep current) or 0..100");
    }
    volume_ = static_cast<int>(configured_volume);
    if (app_name_.empty()) {
      throw std::invalid_argument("app_name must not be empty");
    }
    if (!dry_run_) {
      backend_ = std::make_unique<AudioBackend>();
    }
    server_ = rclcpp_action::create_server<Speak>(
        this, "/g1_audio/speak",
        [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Speak::Goal> goal) {
          return accept_goal(uuid, *goal);
        },
        [this](std::shared_ptr<GoalHandle> goal) {
          // SDK calls can each wait five seconds. Never call them here: cancel
          // acceptance and the task's motion cleanup must stay responsive.
          {
            std::lock_guard<std::mutex> lock(mutex_);
            if (finishing_ || goal->get_goal_id() != active_uuid_) {
              return rclcpp_action::CancelResponse::REJECT;
            }
            cancel_requested_ = true;
          }
          cv_.notify_all();
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<GoalHandle> goal) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            pending_ = std::move(goal);
          }
          cv_.notify_all();
        });
    RCLCPP_INFO(get_logger(), "Speech action ready: dry_run=%s, volume=%d, app_name=%s",
                dry_run_ ? "true" : "false", volume_, app_name_.c_str());
    worker_ = std::thread([this]() { work(); });
  }

  ~AudioBridge() override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  rclcpp_action::GoalResponse accept_goal(const rclcpp_action::GoalUUID& uuid,
                                        const Speak::Goal& goal) {
    const bool blank = std::all_of(goal.text.begin(), goal.text.end(), [](unsigned char c) {
      return std::isspace(c) != 0;
    });
    if (blank || goal.speaker_id < 0 || !std::isfinite(goal.wait_sec) || goal.wait_sec <= 0) {
      RCLCPP_WARN(get_logger(), "Rejected speech: nonempty text, speaker_id>=0, finite wait_sec>0 required");
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (busy_ || stopping_) {
      RCLCPP_WARN(get_logger(), "Rejected speech: bridge is busy or stopping");
      return rclcpp_action::GoalResponse::REJECT;
    }
    busy_ = true;
    active_uuid_ = uuid;
    cancel_requested_ = false;
    finishing_ = false;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  bool interrupted(const std::shared_ptr<GoalHandle>& goal) {
    std::lock_guard<std::mutex> lock(mutex_);
    return stopping_ || cancel_requested_ || !rclcpp::ok() || goal->is_canceling();
  }

  void feedback(const std::shared_ptr<GoalHandle>& goal, const std::string& state,
                double remaining) {
    auto message = std::make_shared<Speak::Feedback>();
    message->state = state;
    message->remaining_sec = std::max(0.0, remaining);
    goal->publish_feedback(message);
  }

  bool initialize_volume(const std::shared_ptr<GoalHandle>& goal, std::string& error) {
    if (volume_initialized_ || dry_run_) {
      return true;
    }
    uint8_t current = 0;
    const int32_t read_code = backend_->client->GetVolume(current);
    if (read_code != 0) {
      error = "GetVolume failed, code=" + std::to_string(read_code);
      return false;
    }
    RCLCPP_INFO(get_logger(), "Current robot volume: %u", static_cast<unsigned>(current));
    if (interrupted(goal)) {
      return false;
    }
    if (volume_ >= 0) {
      const int32_t set_code = backend_->client->SetVolume(static_cast<uint8_t>(volume_));
      if (set_code != 0) {
        error = "SetVolume failed, code=" + std::to_string(set_code);
        return false;
      }
    }
    volume_initialized_ = true;
    return true;
  }

  void request_stop() {
    if (dry_run_ || !rclcpp::ok()) {
      return;
    }
    // Best effort: Unitree's PlayStop effect on built-in TTS needs robot testing.
    try {
      const int32_t code = backend_->client->PlayStop(app_name_);
      if (code != 0) {
        RCLCPP_WARN(get_logger(), "PlayStop failed, code=%d", code);
      }
    } catch (const std::exception& error) {
      RCLCPP_WARN(get_logger(), "PlayStop failed: %s", error.what());
    }
  }

  void execute(const std::shared_ptr<GoalHandle>& goal) {
    auto result = std::make_shared<Speak::Result>();
    bool requested_tts = false;
    try {
      const auto request = goal->get_goal();
      if (!interrupted(goal)) {
        feedback(goal, dry_run_ ? "DRY_RUN_REQUESTING" : "REQUESTING", request->wait_sec);
        std::string error;
        if (initialize_volume(goal, error) && !interrupted(goal)) {
          RCLCPP_INFO(get_logger(), "%s TTS speaker_id=%d wait_sec=%.3f text=%s",
                      dry_run_ ? "Dry run" : "Request", request->speaker_id,
                      request->wait_sec, request->text.c_str());
          requested_tts = !dry_run_;
          const int32_t code = dry_run_ ? 0 :
              backend_->client->TtsMaker(request->text, request->speaker_id);
          if (code != 0) {
            error = "TtsMaker failed, code=" + std::to_string(code);
          } else if (!interrupted(goal)) {
            // Timing starts after request acknowledgement, never at submission.
            // Acknowledgement does not report the physical end of playback.
            const auto start = Clock::now();
            while (!interrupted(goal)) {
              const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
              const double remaining = request->wait_sec - elapsed;
              if (remaining <= 0) {
                result->success = true;
                result->message = "Configured speech wait elapsed; playback completion is not confirmed";
                break;
              }
              feedback(goal, dry_run_ ? "DRY_RUN_WAITING" : "WAITING", remaining);
              std::unique_lock<std::mutex> lock(mutex_);
              cv_.wait_for(lock, std::chrono::duration<double>(std::min(remaining, 0.1)));
            }
          }
        }
        if (!error.empty()) {
          result->message = error;
        }
      }
    } catch (const std::exception& error) {
      result->success = false;
      result->message = std::string("Speech request exception: ") + error.what();
    }

    bool canceled = false;
    bool accepted_cancel = false;
    {
      // Atomically choose a terminal outcome against the cancel callback. Once
      // finishing is set, later cancellation requests are rejected; accepted
      // requests always get a canceled result and an attempted stop.
      std::lock_guard<std::mutex> lock(mutex_);
      finishing_ = true;
      accepted_cancel = cancel_requested_;
      canceled = stopping_ || accepted_cancel || !rclcpp::ok() || goal->is_canceling();
    }
    if (canceled) {
      result->success = false;
      result->message = "Speech canceled; playback stop requested (best effort)";
      // Keep the slot reserved until the stop completes, so a new utterance
      // cannot start before this utterance's delayed PlayStop request.
      request_stop();
    } else if (!result->success && requested_tts) {
      request_stop();
    }

    {
      // Release before publishing the result, so an immediate next-language
      // request cannot see a stale busy flag. UUID checks keep delayed cancels
      // for this goal from affecting a newly accepted goal. The sole worker
      // will start that goal only after finishing this result.
      std::lock_guard<std::mutex> lock(mutex_);
      busy_ = false;
    }
    if (rclcpp::ok()) {
      try {
        // The callback records acceptance just before rclcpp_action transitions
        // the handle to CANCELING. Do not race that transition on this worker.
        while (accepted_cancel && !goal->is_canceling() && rclcpp::ok()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (!rclcpp::ok()) {
          return;
        }
        if (goal->is_canceling()) {
          goal->canceled(result);
        } else if (result->success) {
          goal->succeed(result);
        } else {
          goal->abort(result);
        }
      } catch (const std::exception& error) {
        RCLCPP_WARN(get_logger(), "Unable to finish speech action: %s", error.what());
      }
    }
  }

  void work() {
    while (true) {
      std::shared_ptr<GoalHandle> goal;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return stopping_ || pending_; });
        if (stopping_) {
          return;
        }
        goal = std::move(pending_);
      }
      execute(goal);
    }
  }

  bool dry_run_{true};
  int volume_{-1};
  std::string app_name_;
  bool volume_initialized_{false};
  std::unique_ptr<AudioBackend> backend_;
  rclcpp_action::Server<Speak>::SharedPtr server_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool stopping_{false};
  bool busy_{false};
  bool cancel_requested_{false};
  bool finishing_{false};
  rclcpp_action::GoalUUID active_uuid_{};
  std::shared_ptr<GoalHandle> pending_;
  std::thread worker_;
};
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<AudioBridge>();
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("g1_audio_bridge"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
