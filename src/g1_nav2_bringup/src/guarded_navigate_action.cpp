#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"

namespace g1_navigation
{
using Action = nav2_msgs::action::NavigateToPose;
using Client = rclcpp_action::Client<Action>;
using Handle = rclcpp_action::ClientGoalHandle<Action>;
using Clock = std::chrono::steady_clock;

struct Call
{
  std::mutex mutex;
  geometry_msgs::msg::PoseStamped pose;
  Handle::SharedPtr handle;
  bool responded{false};
  bool finished{false};
  bool cancel_requested{false};
  rclcpp_action::ResultCode code{rclcpp_action::ResultCode::UNKNOWN};
  Clock::time_point started{Clock::now()};
  Clock::time_point cancel_started;
};

struct PathCache
{
  std::mutex mutex;
  nav_msgs::msg::Path path;
};

// Each request owns its callbacks. A late result can never finish a newer call.
// The dedicated executor also processes late acceptance/cancellation while the
// public BT is idle, after an outer task has already been canceled.
class GuardedNavigate : public BT::ActionNodeBase
{
public:
  GuardedNavigate(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ActionNodeBase(name, config), paths_(std::make_shared<PathCache>())
  {
    auto parent = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    auto options = rclcpp::NodeOptions().use_global_arguments(false);
    options.context(parent->get_node_base_interface()->get_context());
    node_ = std::make_shared<rclcpp::Node>("g1_guarded_navigate_client", options);
    timeout_ = config.blackboard->get<std::chrono::milliseconds>("server_timeout");
    getInput("server_timeout", timeout_);
    std::string endpoint = "/g1_navigation/execute";
    getInput("server_name", endpoint);
    client_ = rclcpp_action::create_client<Action>(node_, endpoint);
    auto cache = paths_;
    path_sub_ = node_->create_subscription<nav_msgs::msg::Path>("/g1_navigation/path", 1,
      [cache](nav_msgs::msg::Path::SharedPtr path) {
        std::lock_guard<std::mutex> lock(cache->mutex);
        cache->path = *path;
      });
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = node_->get_node_base_interface()->get_context();
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_node(node_);
    auto executor = executor_;
    thread_ = std::thread([executor]() {
        try {executor->spin();} catch (const std::exception &) {}
      });
    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
      executor_->cancel();
      thread_.join();
      throw std::runtime_error("Navigation coordinator unavailable: " + endpoint);
    }
  }

  ~GuardedNavigate() override
  {
    cancel(call_);
    // No new executor may be created after the ROS context has shut down.
    const auto deadline = Clock::now() + std::chrono::seconds(6);
    while (rclcpp::ok(node_->get_node_base_interface()->get_context()) &&
      call_ && !finished(call_) && Clock::now() < deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    executor_->cancel();
    if (thread_.joinable()) {thread_.join();}
  }

  static BT::PortsList providedPorts()
  {
    return nav2_behavior_tree::BtActionNode<Action>::providedBasicPorts(
      {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")});
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped target;
    if (!getInput("goal", target)) {return BT::NodeStatus::FAILURE;}
    {
      std::lock_guard<std::mutex> lock(paths_->mutex);
      config().blackboard->set("path", paths_->path);
    }
    if (call_) {
      bool canceled;
      {
        std::lock_guard<std::mutex> lock(call_->mutex);
        canceled = call_->cancel_requested;
      }
      const bool changed = target.pose != call_->pose.pose ||
        target.header.frame_id != call_->pose.header.frame_id;
      if (canceled || changed) {
        cancel(call_);
        if (!finished(call_)) {
          std::lock_guard<std::mutex> lock(call_->mutex);
          if (Clock::now() - call_->cancel_started > std::chrono::seconds(6)) {
            RCLCPP_ERROR(node_->get_logger(), "PREVIOUS_NAVIGATION_STOP_UNCONFIRMED");
            return BT::NodeStatus::FAILURE;
          }
          return BT::NodeStatus::RUNNING;
        }
        call_.reset();
      } else {
        bool done, responded;
        rclcpp_action::ResultCode code;
        {
          std::lock_guard<std::mutex> lock(call_->mutex);
          done = call_->finished;
          responded = call_->responded;
          code = call_->code;
        }
        if (done) {
          return code == rclcpp_action::ResultCode::SUCCEEDED ?
                 BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
        if (!responded && Clock::now() - call_->started > timeout_) {
          cancel(call_);
          RCLCPP_ERROR(node_->get_logger(), "COORDINATOR_ACCEPTANCE_TIMEOUT");
          return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
      }
    }
    // A completed previous public task may be reused by the next goal at the
    // same pose. halt() marks that call canceled before the tree returns IDLE.
    call_ = std::make_shared<Call>();
    call_->pose = target;
    config().blackboard->set("path", nav_msgs::msg::Path());
    {
      std::lock_guard<std::mutex> lock(paths_->mutex);
      paths_->path = nav_msgs::msg::Path();
    }
    auto call = call_;
    std::weak_ptr<Client> weak_client(client_);
    Client::SendGoalOptions options;
    options.goal_response_callback = [call, weak_client](std::shared_future<Handle::SharedPtr> future) {
        const auto handle = future.get();
        bool stop;
        {
          std::lock_guard<std::mutex> lock(call->mutex);
          call->handle = handle;
          call->responded = true;
          if (!handle) {
            call->finished = true;
            call->code = rclcpp_action::ResultCode::ABORTED;
          }
          stop = call->cancel_requested && !call->finished;
        }
        if (stop && handle) {
          if (auto client = weak_client.lock()) {
            try {client->async_cancel_goal(handle);} catch (const std::exception &) {}
          }
        }
      };
    options.result_callback = [call](const Handle::WrappedResult & result) {
        std::lock_guard<std::mutex> lock(call->mutex);
        call->code = result.code;
        call->finished = true;
      };
    Action::Goal goal;
    goal.pose = target;
    try {
      client_->async_send_goal(goal, options);
    } catch (const std::exception & error) {
      cancel(call_);
      RCLCPP_ERROR(node_->get_logger(), "COORDINATOR_SEND_ERROR: %s", error.what());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void halt() override
  {
    cancel(call_);
    setStatus(BT::NodeStatus::IDLE);
  }

private:
  static bool finished(const std::shared_ptr<Call> & call)
  {
    std::lock_guard<std::mutex> lock(call->mutex);
    return call->finished;
  }

  void cancel(const std::shared_ptr<Call> & call)
  {
    if (!call) {return;}
    Handle::SharedPtr handle;
    {
      std::lock_guard<std::mutex> lock(call->mutex);
      if (call->cancel_requested) {return;}
      call->cancel_requested = true;
      call->cancel_started = Clock::now();
      if (!call->finished) {handle = call->handle;}
    }
    if (handle && rclcpp::ok(node_->get_node_base_interface()->get_context())) {
      try {client_->async_cancel_goal(handle);} catch (const std::exception &) {}
    }
  }

  std::shared_ptr<Call> call_;
  std::shared_ptr<PathCache> paths_;
  rclcpp::Node::SharedPtr node_;
  Client::SharedPtr client_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread thread_;
  std::chrono::milliseconds timeout_{1000};
};
}  // namespace g1_navigation

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<g1_navigation::GuardedNavigate>("G1GuardedNavigate");
}
