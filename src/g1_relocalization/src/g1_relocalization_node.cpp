#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <g1_relocalization/global_matcher.hpp>
#include <g1_relocalization/candidate_evidence.hpp>
#include <g1_relocalization/pose_buffer.hpp>
#include <g1_relocalization/diagnostics.hpp>

namespace
{
constexpr double kPi = 3.14159265358979323846;
using Cloud = g1_relocalization::GlobalCloud;
using Clock = std::chrono::steady_clock;
int64_t stampNs(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) {return 0;}
  return static_cast<int64_t>(stamp.sec) * 1000000000 + stamp.nanosec;
}
std::string quoted(const std::string & text)
{
  std::ostringstream out;
  out << '"';
  for (const unsigned char c : text) {
    if (c == '"' || c == '\\') {out << '\\' << c;}
    else if (c < 0x20) {out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);}
    else {out << c;}
  }
  out << '"';
  return out.str();
}
std::string jsonNumber(double value)
{
  if (!std::isfinite(value)) {return "null";}
  std::ostringstream out; out << std::setprecision(10) << value; return out.str();
}
Eigen::Matrix4f translation(const Eigen::Vector3f & offset)
{
  auto result = Eigen::Matrix4f::Identity().eval();
  result.block<3, 1>(0, 3) = offset;
  return result;
}
double rotationDistance(const Eigen::Matrix4f & a, const Eigen::Matrix4f & b)
{
  return Eigen::Quaternionf(a.block<3, 3>(0, 0)).angularDistance(
    Eigen::Quaternionf(b.block<3, 3>(0, 0)));
}
bool poseMatrix(const geometry_msgs::msg::Pose & pose, Eigen::Matrix4f & result)
{
  Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  Eigen::Vector3f t(pose.position.x, pose.position.y, pose.position.z);
  if (!q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-6 ||
    !t.allFinite()) {return false;}
  result = Eigen::Matrix4f::Identity();
  result.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  result.block<3, 1>(0, 3) = t;
  return true;
}
geometry_msgs::msg::Pose matrixPose(const Eigen::Matrix4f & matrix)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = matrix(0, 3);
  pose.position.y = matrix(1, 3);
  pose.position.z = matrix(2, 3);
  Eigen::Quaternionf q(matrix.block<3, 3>(0, 0));
  q.normalize();
  pose.orientation.x = q.x(); pose.orientation.y = q.y();
  pose.orientation.z = q.z(); pose.orientation.w = q.w();
  return pose;
}
}  // namespace

class G1RelocalizationNode : public rclcpp::Node
{
public:
  G1RelocalizationNode() : Node("g1_relocalization")
  {
    readParameters();
    base_T_body_.block<3, 3>(0, 0) = Eigen::AngleAxisf(
      body_roll_deg_ * kPi / 180., Eigen::Vector3f::UnitX()).toRotationMatrix();
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/global_pcd_map",
      rclcpp::QoS(1).reliable().transient_local());
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization_pose", 10);
    fitness_pub_ = create_publisher<std_msgs::msg::Float64>("/localization_fitness", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/g1_relocalization/status",
      rclcpp::QoS(1).reliable().transient_local());
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    loadMap();
    matcher_ = std::make_unique<g1_relocalization::GlobalMatcher>(map_, options_);
    prepareDiagnostics();

    // Sensor callbacks remain responsive while the worker runs global search/NDT.
    input_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = input_group_;
    const auto qos = rclcpp::SensorDataQoS().keep_last(2);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic_, qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {cloudCallback(msg);}, subscription_options);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic_, qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {odomCallback(msg);}, subscription_options);
    initial_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(initial_topic_, 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {initialCallback(msg);}, subscription_options);
    status_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {publishStatus();});
    tf_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {publishTf();});
    state_ = mode_ == "global" ? "WAITING_DATA" : "WAITING_INITIAL_POSE";
    reason_ = mode_ == "global" ? "Waiting for fresh synchronized FAST-LIO data" : "Waiting for /initialpose";
    publishStatus();
    worker_ = std::thread([this]() {worker();});
    RCLCPP_INFO(get_logger(), "Relocalization initialization_mode=%s", mode_.c_str());
  }

  ~G1RelocalizationNode() override
  {
    stopping_.store(true);
    wake_.notify_all();
    if (worker_.joinable()) {worker_.join();}
  }

private:
  struct Frame
  {
    Cloud::ConstPtr cloud;
    int64_t stamp = 0;
    Eigen::Matrix4f odom_T_body = Eigen::Matrix4f::Identity();
  };
  using Frames = std::vector<Frame, Eigen::aligned_allocator<Frame>>;

  struct Hypothesis
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f anchor = Eigen::Matrix4f::Identity();
    bool initialized = false;
  };

  void readParameters()
  {
    auto str = [this](const char * key, const std::string & value) {
      return declare_parameter<std::string>(key, value);
    };
    auto number = [this](const char * key, double value) {return declare_parameter<double>(key, value);};
    auto integer = [this](const char * key, int value) {return declare_parameter<int>(key, value);};
    auto boolean = [this](const char * key, bool value) {return declare_parameter<bool>(key, value);};
    map_path_ = str("map_path", ""); mode_ = str("initialization_mode", "manual");
    global_frame_ = str("global_frame", "map"); odom_frame_ = str("odom_frame", "camera_init");
    body_frame_ = str("body_frame", "body"); body_roll_deg_ = number("body_roll_deg", 180.);
    cloud_topic_ = str("input_cloud_topic", "/cloud_registered_body");
    odom_topic_ = str("odometry_topic", "/Odometry"); initial_topic_ = str("initial_pose_topic", "/initialpose");
    options_.map_voxel_size = number("map_voxel_size", .25);
    options_.scan_voxel_size = number("scan_voxel_size", .20);
    options_.ndt_resolution = number("ndt_resolution", 1.);
    options_.ndt_step_size = number("ndt_step_size", .1);
    options_.ndt_transformation_epsilon = number("ndt_transformation_epsilon", .01);
    options_.ndt_max_iterations = integer("ndt_max_iterations", 40);
    tracking_fitness_ = number("fitness_threshold", 2.);
    period_ms_ = integer("localization_period_ms", 500);
    max_scan_age_ = number("max_scan_age_sec", 2.);
    valid_timeout_ = number("localization_valid_timeout_sec", 3.);
    odom_gap_ = number("max_odom_sync_gap_sec", .1);
    collect_seconds_ = number("global_collection_sec", 2.);
    collect_timeout_ = number("global_collection_timeout_sec", 15.);
    collection_frames_ = integer("global_min_scan_count", 10);
    max_collection_points_ = integer("global_max_collection_points", 120000);
    confirmation_frames_ = integer("confirmation_scan_count", 3);
    verification_timeout_ = number("verification_timeout_sec", 20.);
    initialization_timeout_ = number("global_initialization_timeout_sec", 180.);
    max_attempts_ = integer("global_max_attempts", 3);
    max_hypotheses_ = integer("global_verification_candidates", 3);
    confirmation_coverage_gap_ = number("global_confirmation_coverage_gap", .08);
    direct_accept_coverage_ = number("global_direct_accept_coverage", .91);
    diagnostics_directory_ = str("diagnostics_directory", "");
    confirmation_translation_ = number("confirmation_max_translation", .30);
    confirmation_rotation_ = number("confirmation_max_rotation_deg", 10.) * kPi / 180.;
    options_.time_budget_seconds = number("global_search_timeout_sec", 60.);
    options_.use_search_bounds = boolean("global_use_search_bounds", false);
    options_.min_x = number("global_min_x", -8.5); options_.max_x = number("global_max_x", 28.05);
    options_.min_y = number("global_min_y", -26.5); options_.max_y = number("global_max_y", 25.05);
    options_.use_ground_plane = boolean("global_use_ground_plane", true);
    options_.level_from_ground_plane = boolean("global_level_from_ground_plane", false);
    options_.ground_a = number("global_ground_a", -.06203);
    options_.ground_b = number("global_ground_b", -.02687);
    options_.ground_c = number("global_ground_c", -1.51);
    options_.sensor_height = number("global_sensor_height", 1.51);
    options_.initial_z = number("global_initial_z", 0.);
    options_.map_roll_deg = number("global_map_roll_deg", 0.);
    options_.map_pitch_deg = number("global_map_pitch_deg", 0.);
    options_.min_scan_z = number("global_min_scan_z", -.8);
    options_.max_scan_z = number("global_max_scan_z", 2.);
    options_.max_scan_range = number("global_max_scan_range", 20.);
    options_.grid_resolution = number("global_grid_resolution", .25);
    options_.coarse_xy_step = number("global_coarse_xy_step", 1.);
    options_.coarse_yaw_step_deg = number("global_coarse_yaw_step_deg", 10.);
    options_.fine_xy_step = number("global_fine_xy_step", .25);
    options_.fine_yaw_step_deg = number("global_fine_yaw_step_deg", 2.);
    options_.retained_modes = integer("global_retained_modes", 96);
    options_.ndt_candidates = integer("global_ndt_candidates", 12);
    options_.min_correlative_score = number("global_min_correlative_score", .45);
    options_.ambiguity_score_gap = number("global_ambiguity_score_gap", .08);
    options_.unresolved_score_gap = number("global_unresolved_score_gap", .025);
    options_.max_fitness = number("global_max_fitness", .6);
    options_.coverage_distance = number("global_coverage_distance", .45);
    options_.min_coverage = number("global_min_coverage", .65);
    options_.max_inlier_rmse = number("global_max_inlier_rmse", .35);
    if (mode_ != "manual" && mode_ != "global") {throw std::invalid_argument("initialization_mode must be manual or global");}
    for (double v : {tracking_fitness_, max_scan_age_, valid_timeout_, odom_gap_, collect_seconds_,
        collect_timeout_, verification_timeout_, initialization_timeout_,
        confirmation_translation_, confirmation_rotation_, confirmation_coverage_gap_, direct_accept_coverage_})
    {
      if (!std::isfinite(v) || v <= 0) {throw std::invalid_argument("Relocalization time/quality limits must be finite and positive");}
    }
    if (period_ms_ <= 0 || collection_frames_ < 2 || confirmation_frames_ < 2 ||
      max_collection_points_ < 100 || max_attempts_ < 1 || max_hypotheses_ < 1 ||
      max_hypotheses_ > 16 || confirmation_coverage_gap_ > 1. || direct_accept_coverage_ > 1. ||
      !std::isfinite(body_roll_deg_))
    {throw std::invalid_argument("Invalid collection/confirmation counts or body roll");}
    if (max_scan_age_ > valid_timeout_ || valid_timeout_ > 3.) {
      throw std::invalid_argument("Require max_scan_age_sec <= localization_valid_timeout_sec <= 3 seconds");
    }
    if (global_frame_.empty() || odom_frame_.empty() || body_frame_.empty() ||
      cloud_topic_.empty() || odom_topic_.empty() || initial_topic_.empty()) {
      throw std::invalid_argument("Frames and topics must not be empty");
    }
  }

  void prepareDiagnostics()
  {
    map_sha256_ = g1_relocalization::fileSha256(map_path_);
    std::ostringstream config;
    config << '{';
    auto names = list_parameters({}, 0).names;
    std::sort(names.begin(), names.end());
    bool first = true;
    for (const auto & name : names) {
      if (name == "diagnostics_directory") {continue;}
      if (!first) {config << ',';}
      first = false;
      config << quoted(name) << ':' << quoted(get_parameter(name).value_to_string());
    }
    config << '}';
    const auto configuration = config.str();
    std::istringstream input(configuration);
    config_sha256_ = g1_relocalization::sha256(input);
    if (diagnostics_directory_.empty()) {
      const char * workspace = std::getenv("G1_WS");
      diagnostics_directory_ = workspace && *workspace ?
        std::string(workspace) + "/runtime/navigation/localization" : "/tmp/g1_relocalization";
    }
    diagnostics_directory_ += "/session_" + std::to_string(now().nanoseconds()) +
      "_" + std::to_string(::getpid());
    g1_relocalization::makeDirectories(diagnostics_directory_);
    std::ofstream file(diagnostics_directory_ + "/session.json");
    file << "{\"map_path\":" << quoted(map_path_) << ",\"map_sha256\":" << quoted(map_sha256_)
         << ",\"config_sha256\":" << quoted(config_sha256_)
         << ",\"reference_commit\":\"afaf7bc5daefc98cfa045c0b50d05e17b7ac2053\",\"parameters\":"
         << configuration << "}\n";
    file.close();
    if (!file) {throw std::runtime_error("Cannot save localization session diagnostics");}
    RCLCPP_INFO(get_logger(), "Localization diagnostics: %s", diagnostics_directory_.c_str());
  }

  void loadMap()
  {
    if (map_path_.empty()) {throw std::invalid_argument("map_path is empty");}
    auto raw = Cloud::Ptr(new Cloud);
    if (pcl::io::loadPCDFile(map_path_, *raw) < 0) {throw std::runtime_error("Cannot load PCD map: " + map_path_);}
    auto finite = Cloud::Ptr(new Cloud);
    for (const auto & point : *raw) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {finite->push_back(point);}
    }
    if (options_.map_voxel_size <= 0 || !std::isfinite(options_.map_voxel_size)) {
      throw std::invalid_argument("map_voxel_size must be finite and positive");
    }
    map_ = filter(finite, options_.map_voxel_size);
    if (map_->size() < 100) {throw std::invalid_argument("Map has fewer than 100 finite filtered points");}
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*map_, msg); msg.header.frame_id = global_frame_; msg.header.stamp = now();
    map_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Loaded %zu filtered PCD points", map_->size());
  }

  static Cloud::Ptr filter(Cloud::ConstPtr input, double leaf)
  {
    auto output = Cloud::Ptr(new Cloud);
    pcl::VoxelGrid<g1_relocalization::GlobalPoint> voxel;
    voxel.setInputCloud(input); voxel.setLeafSize(leaf, leaf, leaf); voxel.filter(*output);
    return output;
  }

  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    const int64_t stamp = stampNs(msg->header.stamp);
    if (msg->header.frame_id != body_frame_ ||
      !g1_relocalization::freshStamp(stamp, now().nanoseconds(), max_scan_age_)) {return;}
    auto raw = Cloud::Ptr(new Cloud);
    pcl::fromROSMsg(*msg, *raw);
    auto finite = Cloud::Ptr(new Cloud);
    for (const auto & point : *raw) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {finite->push_back(point);}
    }
    if (finite->size() < 100) {return;}
    auto cloud = filter(finite, options_.scan_voxel_size);
    if (cloud->size() < 100) {return;}
    std::lock_guard<std::mutex> lock(mutex_);
    if (stamp <= last_received_scan_) {return;}
    last_received_scan_ = stamp;
    Frame frame; frame.cloud = cloud; frame.stamp = stamp;
    pending_.push_back(frame);
    while (pending_.size() > 30) {pending_.pop_front();}
    wake_.notify_all();
  }

  void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (msg->header.frame_id != odom_frame_ || msg->child_frame_id != body_frame_ ||
      !g1_relocalization::freshStamp(stampNs(msg->header.stamp), now().nanoseconds(), max_scan_age_)) {return;}
    Eigen::Matrix4f pose;
    if (!poseMatrix(msg->pose.pose, pose)) {return;}
    std::lock_guard<std::mutex> lock(mutex_);
    if (poses_.add(stampNs(msg->header.stamp), pose)) {wake_.notify_all();}
  }

  void initialCallback(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != global_frame_) {
      RCLCPP_WARN(get_logger(), "Rejected initialpose in wrong frame"); return;
    }
    Eigen::Matrix4f pose;
    if (!poseMatrix(msg->pose.pose, pose)) {RCLCPP_WARN(get_logger(), "Rejected non-finite/zero-quaternion initialpose"); return;}
    {
      std::lock_guard<std::mutex> lock(mutex_);
      manual_guess_ = pose * base_T_body_;
      has_manual_ = true; generation_.fetch_add(1); valid_ = false;
      last_success_stamp_ = 0; fitness_ = std::numeric_limits<double>::infinity();
      state_ = "VERIFYING"; reason_ = "Verifying manual initialpose on new scans";
      candidate_status_ = "[]"; initialization_attempt_ = 0;
      minimum_scan_stamp_ = now().nanoseconds();
      coverage_ = 0.;
      pending_.clear();
    }
    publishFitness(std::numeric_limits<double>::infinity());
    publishStatus();
    wake_.notify_all();
  }

  bool nextFrame(Frame & frame)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    // Use the newest synchronized scan; do not build a stale processing backlog.
    for (auto it = pending_.rbegin(); it != pending_.rend(); ++it) {
      if (it->stamp <= last_processed_scan_ || it->stamp <= minimum_scan_stamp_ ||
        !g1_relocalization::freshStamp(it->stamp, now().nanoseconds(), max_scan_age_)) {continue;}
      if (!poses_.lookup(it->stamp, odom_gap_, frame.odom_T_body)) {continue;}
      frame.cloud = it->cloud; frame.stamp = it->stamp;
      last_processed_scan_ = frame.stamp;
      while (!pending_.empty() && pending_.front().stamp <= frame.stamp) {pending_.pop_front();}
      return true;
    }
    while (!pending_.empty() && !g1_relocalization::freshStamp(
        pending_.front().stamp, now().nanoseconds(), max_scan_age_)) {pending_.pop_front();}
    wake_.wait_for(lock, std::chrono::milliseconds(30));
    return false;
  }

  Cloud::Ptr gravityScan(const Frame & frame) const
  {
    auto result = Cloud::Ptr(new Cloud);
    auto rotation = frame.odom_T_body;
    rotation.block<3, 1>(0, 3).setZero();
    pcl::transformPointCloud(*frame.cloud, *result, rotation);
    return result;
  }

  void setState(const std::string & state, const std::string & reason, uint64_t generation)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation_.load() != generation) {return;}
    if (state_ != state) {RCLCPP_INFO(get_logger(), "Localization state %s: %s", state.c_str(), reason.c_str());}
    state_ = state; reason_ = reason; valid_ = false;
    std::ofstream file(diagnostics_directory_ + "/states.jsonl", std::ios::app);
    file << "{\"stamp_ns\":" << now().nanoseconds() << ",\"state\":" << quoted(state)
         << ",\"reason\":" << quoted(reason) << ",\"attempt\":" << initialization_attempt_ << "}\n";
  }

  void publishFitness(double fitness)
  {
    std_msgs::msg::Float64 msg; msg.data = fitness; fitness_pub_->publish(msg);
  }

  void worker()
  {
    try {runWorker();}
    catch (const std::exception & error) {
      setState("FAILED", std::string("Localization worker error: ") + error.what(), generation_.load());
      publishFitness(std::numeric_limits<double>::infinity());
      RCLCPP_ERROR(get_logger(), "Localization worker failed: %s", error.what());
    }
  }

  void candidateProgress(int attempt, const std::string & candidates, uint64_t generation)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation != generation_.load()) {return;}
    initialization_attempt_ = attempt; candidate_status_ = candidates;
  }

  void recordSearch(
    Cloud::ConstPtr scan, const g1_relocalization::GlobalMatchResult & result, int attempt,
    int64_t stamp)
  {
    std::ostringstream json;
    json << std::setprecision(10) << "{\"map_path\":" << quoted(map_path_)
         << ",\"map_sha256\":" << quoted(map_sha256_)
         << ",\"config_sha256\":" << quoted(config_sha256_)
         << ",\"elapsed_seconds\":" << jsonNumber(result.elapsed_seconds)
         << ",\"evaluations\":" << result.evaluations
         << ",\"scan_stamp_ns\":" << stamp << ",\"attempt\":" << attempt
         << ",\"status\":" << quoted(g1_relocalization::globalMatchStatusName(result.status))
         << ",\"reason\":" << quoted(result.reason) << ",\"candidates\":[";
    for (std::size_t i = 0; i < result.candidates.size(); ++i) {
      const auto & c = result.candidates[i];
      if (i) {json << ',';}
      json << "{\"rank\":" << i + 1 << ",\"acceptable\":" << (c.acceptable ? "true" : "false")
           << ",\"coarse_score\":" << c.correlative_score << ",\"score\":" << c.quality.score
           << ",\"coverage\":" << c.quality.coverage << ",\"fitness\":";
      if (std::isfinite(c.fitness)) {json << c.fitness;} else {json << "null";}
      json << ",\"inlier_rmse\":";
      if (std::isfinite(c.quality.inlier_rmse)) {json << c.quality.inlier_rmse;} else {json << "null";}
      json << ",\"x\":" << jsonNumber(c.map_T_scan(0, 3)) << ",\"y\":" << jsonNumber(c.map_T_scan(1, 3))
           << ",\"z\":" << jsonNumber(c.map_T_scan(2, 3)) << ",\"yaw\":"
           << jsonNumber(std::atan2(c.map_T_scan(1, 0), c.map_T_scan(0, 0)))
           << ",\"initial_x\":" << jsonNumber(c.initial_map_T_scan(0, 3))
           << ",\"initial_y\":" << jsonNumber(c.initial_map_T_scan(1, 3))
           << ",\"initial_z\":" << jsonNumber(c.initial_map_T_scan(2, 3))
           << ",\"initial_yaw\":" << jsonNumber(std::atan2(c.initial_map_T_scan(1, 0), c.initial_map_T_scan(0, 0))) << '}';
      RCLCPP_INFO(get_logger(), "Candidate %zu accepted=%d score=%.4f coarse=%.4f fitness=%.4f coverage=%.3f rmse=%.3f xy=(%.3f,%.3f)",
        i + 1, c.acceptable, c.quality.score, c.correlative_score, c.fitness,
        c.quality.coverage, c.quality.inlier_rmse, c.map_T_scan(0, 3), c.map_T_scan(1, 3));
    }
    json << "]}";
    if (diagnostics_directory_.empty()) {return;}
    const std::string base = diagnostics_directory_ + "/global_" + std::to_string(stamp) + "_" + std::to_string(attempt);
    if (pcl::io::savePCDFileBinary(base + ".pcd", *scan) < 0) {throw std::runtime_error("PCD write failed");}
    std::ofstream file(base + ".json"); file << json.str() << '\n';
    file.close();
    if (!file) {throw std::runtime_error("Search JSON write failed");}
  }

  void confirmPose(const Frame & frame, const Eigen::Matrix4f & map_T_odom,
    double fitness, const g1_relocalization::MatchQuality & quality, bool first, uint64_t generation)
  {
    // Initialpose invalidation, the last freshness check and publication are
    // serialized so an in-flight optimizer cannot publish an obsolete success.
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation != generation_.load() ||
      !g1_relocalization::freshStamp(frame.stamp, now().nanoseconds(), max_scan_age_)) {return;}
    map_T_odom_ = map_T_odom; last_success_stamp_ = frame.stamp;
    last_success_steady_ = Clock::now(); fitness_ = fitness; coverage_ = quality.coverage;
    valid_ = true; state_ = first ? "LOCALIZED" : "TRACKING";
    reason_ = "Confirmed on distinct, fresh synchronized scans";
    publishFitness(fitness);
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = global_frame_; pose.header.stamp = rclcpp::Time(frame.stamp);
    pose.pose.pose = matrixPose(map_T_odom * frame.odom_T_body * base_T_body_.inverse());
    pose.pose.covariance[0] = pose.pose.covariance[7] = pose.pose.covariance[14] = .04;
    pose.pose.covariance[21] = pose.pose.covariance[28] = pose.pose.covariance[35] = .01;
    pose_pub_->publish(pose);
    if (first) {RCLCPP_INFO(get_logger(), "NDT success: confirmed fitness=%.6f coverage=%.3f", fitness, quality.coverage);}
  }

  void runWorker()
  {
    pcl::NormalDistributionsTransform<g1_relocalization::GlobalPoint, g1_relocalization::GlobalPoint> ndt;
    ndt.setInputTarget(map_); ndt.setResolution(options_.ndt_resolution);
    ndt.setStepSize(options_.ndt_step_size); ndt.setTransformationEpsilon(options_.ndt_transformation_epsilon);
    ndt.setMaximumIterations(options_.ndt_max_iterations);
    uint64_t generation = std::numeric_limits<uint64_t>::max();
    bool seeded = false, attempted = false, confirmed = false;
    int confirmations = 0;
    Frames collection;
    Eigen::Matrix4f map_T_odom = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f anchor = map_T_odom;
    auto collecting_since = Clock::now(), verifying_since = Clock::now(), last_track = Clock::now();
    int64_t verification_after = 0, search_stamp = 0;
    std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>> hypotheses;
    std::vector<g1_relocalization::CandidateEvidence> evidence;
    bool omitted_candidates = false, automatic_done = false, automatic_started = false;
    int attempts = 0;
    auto initialization_since = Clock::now();
    std::string last_failure_state = "FAILED";
    auto expired = [&]() {
        return automatic_started && Clock::now() - initialization_since >=
          std::chrono::duration<double>(initialization_timeout_);
      };
    struct OptimizationInterrupted {};
    std::function<void(const Cloud &, const std::vector<int> &,
      const Cloud &, const std::vector<int> &)> ndt_progress =
      [&](const Cloud &, const std::vector<int> &, const Cloud &, const std::vector<int> &) {
        if (stopping_.load() || !rclcpp::ok() || generation != generation_.load() ||
          (!automatic_done && expired())) {throw OptimizationInterrupted{};}
      };
    ndt.registerVisualizationCallback(ndt_progress);
    auto align = [&](Cloud & output, const Eigen::Matrix4f & guess) {
        try {ndt.align(output, guess); return true;}
        catch (const OptimizationInterrupted &) {return false;}
      };
    auto retry = [&](const std::string & state, const std::string & reason) {
        hypotheses.clear(); evidence.clear(); collection.clear();
        omitted_candidates = false; verification_after = now().nanoseconds();
        candidateProgress(attempts, "[]", generation);
        last_failure_state = state;
        publishFitness(std::numeric_limits<double>::infinity());
        if (attempts >= max_attempts_ || expired()) {
          automatic_done = attempted = true;
          setState(state, reason + "; automatic initialization exhausted after " +
            std::to_string(attempts) + " attempts; set initialpose to continue", generation);
        } else {
          attempted = false;
          collecting_since = Clock::now();
          setState("RETRYING", reason + "; collecting NEW scans for attempt " +
            std::to_string(attempts + 1) + "/" + std::to_string(max_attempts_), generation);
        }
      };
    while (!stopping_.load() && rclcpp::ok()) {
      const uint64_t current_generation = generation_.load();
      if (generation != current_generation) {
        generation = current_generation; seeded = attempted = confirmed = false;
        confirmations = 0; collection.clear(); verification_after = 0;
        collecting_since = Clock::now();
        hypotheses.clear(); evidence.clear(); omitted_candidates = false;
        automatic_done = automatic_started = false; attempts = 0;
        last_failure_state = "FAILED";
      }
      bool manual;
      Eigen::Matrix4f manual_guess;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        manual = has_manual_; manual_guess = manual_guess_;
      }
      if (!manual && mode_ == "global" && !automatic_done && expired()) {
        retry(last_failure_state, "Total automatic initialization time budget expired");
      }
      if (!hypotheses.empty() && Clock::now() - verifying_since >
        std::chrono::duration<double>(verification_timeout_))
      {
        retry("AMBIGUOUS", "Ranked candidates remain competing or unverified on fresh scans");
      }
      // Manual/recovery timeouts stay terminal until a new initialpose. The
      // global candidate bank retries within one shared initialization deadline.
      if (!confirmed && seeded && Clock::now() - verifying_since >
        std::chrono::duration<double>(verification_timeout_))
      {
        seeded = false; attempted = true;
        setState("FAILED", "Candidate did not pass fresh scan confirmation before timeout", generation);
        publishFitness(std::numeric_limits<double>::infinity());
      }
      if (!seeded && !attempted && !collection.empty() && Clock::now() - collecting_since >
        std::chrono::duration<double>(collect_timeout_))
      {
        collection.clear(); attempted = true;
        ++attempts;
        retry("FAILED", "Unable to collect enough fresh synchronized scans");
      }
      Frame frame;
      if (!nextFrame(frame)) {continue;}
      if (generation != generation_.load()) {continue;}
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!valid_ && confirmed) {
          confirmed = false; confirmations = 0; anchor = map_T_odom;
          verifying_since = Clock::now();
        }
      }
      if (!hypotheses.empty()) {
        if (frame.stamp <= verification_after) {continue;}
        const auto gravity_scan = gravityScan(frame);
        std::vector<double> fitnesses;
        std::vector<g1_relocalization::MatchQuality> qualities;
        std::ostringstream progress;
        progress << std::setprecision(6) << '[';
        bool interrupted = false;
        for (std::size_t i = 0; i < hypotheses.size(); ++i) {
          auto & hypothesis = hypotheses[i];
          ndt.setInputSource(frame.cloud);
          Cloud aligned;
          if (!align(aligned, hypothesis.transform * frame.odom_T_body)) {interrupted = true; break;}
          if (stopping_.load() || generation != generation_.load() || expired()) {interrupted = true; break;}
          const double fitness = ndt.hasConverged() ? ndt.getFitnessScore() : std::numeric_limits<double>::infinity();
          const auto fitted = ndt.getFinalTransformation().eval();
          const auto transform = (fitted * frame.odom_T_body.inverse()).eval();
          const auto quality = matcher_->evaluate(gravity_scan,
            transform * translation(frame.odom_T_body.block<3, 1>(0, 3)));
          const bool stable = (transform.block<3, 1>(0, 3) - hypothesis.anchor.block<3, 1>(0, 3)).norm() <=
            (hypothesis.initialized ? confirmation_translation_ : options_.max_refinement_translation) &&
            rotationDistance(transform, hypothesis.anchor) <=
            (hypothesis.initialized ? confirmation_rotation_ : options_.max_refinement_yaw_deg * kPi / 180.);
          const bool accepted = ndt.hasConverged() && fitted.allFinite() && quality.acceptable && stable &&
            std::isfinite(fitness) && fitness <= options_.max_fitness;
          auto support = quality;
          bool geometric_rejection = false;
          if (!accepted) {
            const auto reference = matcher_->evaluate(gravity_scan,
              hypothesis.anchor * translation(frame.odom_T_body.block<3, 1>(0, 3)));
            support.score = std::max(support.score, reference.score);
            support.coverage = std::max(support.coverage, reference.coverage);
            geometric_rejection = hypothesis.initialized && stable && ndt.hasConverged() &&
              !quality.acceptable && !reference.acceptable;
          }
          evidence[i].observe(accepted, support.score, support.coverage, geometric_rejection);
          if (accepted) {
            hypothesis.transform = transform;
            if (!hypothesis.initialized) {hypothesis.anchor = transform; hypothesis.initialized = true;}
          }
          fitnesses.push_back(fitness); qualities.push_back(quality);
          if (i) {progress << ',';}
          progress << "{\"candidate\":" << i + 1 << ",\"score\":" << evidence[i].score()
                   << ",\"samples\":" << evidence[i].count() << ",\"accepted\":" << (accepted ? "true" : "false")
                   << ",\"stable\":" << (evidence[i].stable() ? "true" : "false")
                   << ",\"coverage\":" << support.coverage
                   << ",\"mean_coverage\":" << evidence[i].coverage()
                   << ",\"geometric_rejection\":" << (geometric_rejection ? "true" : "false")
                   << ",\"fitness\":" << jsonNumber(fitness) << ",\"rmse\":" << jsonNumber(quality.inlier_rmse)
                   << ",\"x\":" << jsonNumber(transform(0, 3)) << ",\"y\":" << jsonNumber(transform(1, 3)) << '}';
        }
        if (interrupted) {continue;}
        if (Clock::now() - verifying_since > std::chrono::duration<double>(verification_timeout_)) {
          retry("AMBIGUOUS", "Ranked candidate verification time budget expired"); continue;
        }
        // A slow comparison invalidates the ENTIRE batch; no candidate gets
        // confirmation credit for an expired scan or a longer processing turn.
        if (!g1_relocalization::freshStamp(frame.stamp, now().nanoseconds(), max_scan_age_)) {
          for (auto & item : evidence) {item = g1_relocalization::CandidateEvidence(confirmation_frames_);}
          continue;
        }
        progress << ']'; candidateProgress(attempts, progress.str(), generation);
        if (!diagnostics_directory_.empty()) {
          const std::string sample_path = diagnostics_directory_ + "/verification_" +
            std::to_string(search_stamp) + "_" + std::to_string(attempts) + "_" + std::to_string(frame.stamp) + ".pcd";
          if (pcl::io::savePCDFileBinary(sample_path, *gravity_scan) < 0) {
            throw std::runtime_error("Verification PCD write failed");
          }
          std::ofstream file(diagnostics_directory_ + "/verification_" + std::to_string(search_stamp) +
            "_" + std::to_string(attempts) + ".jsonl", std::ios::app);
          file << "{\"stamp_ns\":" << frame.stamp << ",\"omitted_candidates\":"
               << (omitted_candidates ? "true" : "false") << ",\"input_pcd\":" << quoted(sample_path)
               << ",\"candidates\":" << progress.str() << "}\n";
          file.close();
          if (!file) {throw std::runtime_error("Verification JSON write failed");}
        }
        // Independent seeds can converge onto the same physical pose. Merge
        // only after BOTH have stable evidence, without adding their scan counts.
        std::vector<int> distinct;
        std::vector<g1_relocalization::CandidateEvidence> comparison;
        for (std::size_t i = 0; i < hypotheses.size(); ++i) {
          bool duplicate = false;
          for (int prior : distinct) {
            if (evidence[i].stable() && evidence[prior].stable() &&
              (hypotheses[i].transform.block<3, 1>(0, 3) - hypotheses[prior].transform.block<3, 1>(0, 3)).norm() < confirmation_translation_ &&
              rotationDistance(hypotheses[i].transform, hypotheses[prior].transform) < confirmation_rotation_)
            {duplicate = true; break;}
          }
          if (!duplicate) {distinct.push_back(i); comparison.push_back(evidence[i]);}
        }
        const int selected = g1_relocalization::verifiedWinner(
          comparison, options_.ambiguity_score_gap, omitted_candidates,
          confirmation_coverage_gap_, direct_accept_coverage_);
        const int winner = selected < 0 ? -1 : distinct[selected];
        if (expired()) {
          retry("FAILED", "Total automatic initialization time budget expired"); continue;
        }
        if (winner >= 0) {
          if (winner == 0 && evidence.front().stable() &&
            evidence.front().coverage() > direct_accept_coverage_)
          {
            RCLCPP_INFO(get_logger(), "Primary candidate directly accepted: mean coverage=%.6f > %.6f on %zu fresh scans",
              evidence.front().coverage(), direct_accept_coverage_, evidence.front().count());
          }
          RCLCPP_INFO(get_logger(), "Verified candidate %d selected after ranked fresh-scan comparison: %s", winner + 1, progress.str().c_str());
          map_T_odom = hypotheses[winner].transform; anchor = map_T_odom;
          seeded = confirmed = automatic_done = true;
          confirmPose(frame, map_T_odom, fitnesses[winner], qualities[winner], true, generation);
          hypotheses.clear(); evidence.clear();
        } else {
          setState("VERIFYING_CANDIDATES", "Comparing " + std::to_string(hypotheses.size()) +
            " ranked candidates on the same fresh scans" + (omitted_candidates ? "; additional plausible modes remain unresolved" : ""), generation);
        }
        continue;
      }
      if (!seeded && manual && !attempted) {
        map_T_odom = manual_guess * frame.odom_T_body.inverse();
        anchor = map_T_odom; seeded = true; verifying_since = Clock::now();
        setState("VERIFYING", "Confirming supplied initialpose with independent fresh scans", generation);
      }
      if (!seeded && !manual) {
        if (mode_ == "manual" || attempted || automatic_done || frame.stamp <= verification_after) {continue;}
        if (!automatic_started) {automatic_started = true; initialization_since = Clock::now();}
        if (collection.empty()) {collecting_since = Clock::now();}
        if (!collection.empty() &&
          ((frame.odom_T_body.block<3, 1>(0, 3) - collection.front().odom_T_body.block<3, 1>(0, 3)).norm() > confirmation_translation_ ||
          rotationDistance(frame.odom_T_body, collection.front().odom_T_body) > confirmation_rotation_)) {
          ++attempts;
          retry("FAILED", "Robot moved during stationary scan collection"); continue;
        }
        collection.push_back(frame);
        candidateProgress(attempts + 1, "[]", generation);
        setState("COLLECTING", "Accumulating synchronized scans for attempt " +
          std::to_string(attempts + 1) + "/" + std::to_string(max_attempts_), generation);
        const double span = (collection.back().stamp - collection.front().stamp) / 1e9;
        if (Clock::now() - collecting_since > std::chrono::duration<double>(collect_timeout_)) {
          ++attempts; retry("FAILED", "Unable to collect enough fresh synchronized scans"); continue;
        }
        if (span < collect_seconds_ || int(collection.size()) < collection_frames_) {continue;}
        auto accumulated = Cloud::Ptr(new Cloud);
        const Eigen::Vector3f origin = frame.odom_T_body.block<3, 1>(0, 3);
        for (const auto & item : collection) {
          Cloud moved;
          pcl::transformPointCloud(*item.cloud, moved, translation(-origin) * item.odom_T_body);
          *accumulated += moved;
          if (accumulated->size() > static_cast<std::size_t>(max_collection_points_)) {
            accumulated = filter(accumulated, options_.scan_voxel_size);
            if (accumulated->size() > static_cast<std::size_t>(max_collection_points_)) {break;}
          }
        }
        accumulated = filter(accumulated, options_.scan_voxel_size);
        if (accumulated->size() > static_cast<std::size_t>(max_collection_points_)) {
          ++attempts; retry("FAILED", "Accumulated scan exceeds configured point budget"); continue;
        }
        ++attempts;
        search_stamp = frame.stamp;
        candidateProgress(attempts, "[]", generation);
        setState("SEARCHING", "Searching the full configured map and all yaw angles", generation);
        const auto result = matcher_->search(accumulated, [this, generation, &expired]() {
            return stopping_.load() || generation != generation_.load() || !rclcpp::ok() || expired();});
        if (generation != generation_.load() || stopping_.load()) {continue;}
        RCLCPP_INFO(get_logger(), "Global search %s in %.2fs (%zu evaluations): %s",
          g1_relocalization::globalMatchStatusName(result.status), result.elapsed_seconds,
          result.evaluations, result.reason.c_str());
        recordSearch(accumulated, result, attempts, frame.stamp);
        if (expired() || (result.status != g1_relocalization::GlobalMatchStatus::SUCCESS &&
          result.status != g1_relocalization::GlobalMatchStatus::AMBIGUOUS) ||
          result.candidates.empty() || !result.candidates.front().acceptable)
        {
          retry("FAILED", expired() ? "Total automatic initialization time budget expired" : result.reason);
          continue;
        }
        const auto & best = result.candidates.front();
        for (const auto & candidate : result.candidates) {
          const bool plausible = candidate.acceptable ?
            best.quality.score - candidate.quality.score < options_.ambiguity_score_gap :
            candidate.correlative_score >= best.correlative_score - options_.unresolved_score_gap;
          if (!plausible && !candidate.acceptable) {continue;}
          const auto pose = (candidate.acceptable ? candidate.map_T_scan : candidate.initial_map_T_scan).eval();
          Hypothesis hypothesis;
          hypothesis.transform = pose * translation(-origin); hypothesis.anchor = hypothesis.transform;
          hypothesis.initialized = candidate.acceptable;
          bool duplicate = false;
          for (const auto & prior : hypotheses) {
            if ((hypothesis.transform.block<3, 1>(0, 3) - prior.transform.block<3, 1>(0, 3)).norm() < confirmation_translation_ &&
              rotationDistance(hypothesis.transform, prior.transform) < confirmation_rotation_) {duplicate = true; break;}
          }
          if (duplicate) {continue;}
          if (hypotheses.size() >= static_cast<std::size_t>(max_hypotheses_)) {
            // Acceptable locations still need the SAME new scan comparison;
            // different NDT residuals alone cannot eliminate a duplicate room.
            if (plausible || candidate.acceptable) {omitted_candidates = true;}
            continue;
          }
          hypotheses.push_back(hypothesis); evidence.emplace_back(confirmation_frames_);
        }
        // New data must have been acquired AFTER search, not merely queued
        // while the search used its original input.
        verification_after = now().nanoseconds(); verifying_since = Clock::now(); collection.clear();
        last_failure_state = "AMBIGUOUS";
        setState("VERIFYING_CANDIDATES", "Verifying ranked candidates against independent fresh scans", generation);
        continue;
      }
      if (!seeded || frame.stamp <= verification_after) {continue;}
      if (confirmed && Clock::now() - last_track < std::chrono::milliseconds(period_ms_)) {continue;}
      last_track = Clock::now();
      const auto guess = (map_T_odom * frame.odom_T_body).eval();
      ndt.setInputSource(frame.cloud);
      Cloud aligned;
      if (!align(aligned, guess)) {continue;}
      if (stopping_.load() || generation != generation_.load()) {continue;}
      const double fitness = ndt.hasConverged() ? ndt.getFitnessScore() : std::numeric_limits<double>::infinity();
      const auto fitted = ndt.getFinalTransformation().eval();
      const auto candidate = (fitted * frame.odom_T_body.inverse()).eval();
      auto quality = matcher_->evaluate(gravityScan(frame),
        candidate * translation(frame.odom_T_body.block<3, 1>(0, 3)));
      const bool stable = (candidate.block<3, 1>(0, 3) -
          (confirmed ? map_T_odom : anchor).block<3, 1>(0, 3)).norm() <=
        (confirmed ? .75 : confirmation_translation_) &&
        rotationDistance(candidate, confirmed ? map_T_odom : anchor) <=
        (confirmed ? .4 : confirmation_rotation_);
      const bool accepted = ndt.hasConverged() && fitted.allFinite() && quality.acceptable && stable &&
        std::isfinite(fitness) && fitness <= (confirmed ? tracking_fitness_ : options_.max_fitness) &&
        g1_relocalization::freshStamp(frame.stamp, now().nanoseconds(), max_scan_age_);
      if (!accepted) {
        confirmations = 0;
        setState(confirmed ? "LOST" : "VERIFYING", "NDT/coverage/stability/freshness confirmation rejected", generation);
        if (confirmed) {confirmed = false; anchor = map_T_odom; verifying_since = Clock::now();}
        publishFitness(std::numeric_limits<double>::infinity());
        continue;
      }
      map_T_odom = candidate;
      ++confirmations;
      if (!confirmed && confirmations < confirmation_frames_) {
        setState("VERIFYING", "Fresh independent confirmation " + std::to_string(confirmations) +
          "/" + std::to_string(confirmation_frames_), generation); continue;
      }
      const bool first_success = !confirmed;
      confirmed = true;
      confirmPose(frame, map_T_odom, fitness, quality, first_success, generation);
    }
  }

  void expireLocked(int64_t now_ns)
  {
    if (valid_ && (!g1_relocalization::freshStamp(last_success_stamp_, now_ns, valid_timeout_) ||
      Clock::now() - last_success_steady_ > std::chrono::duration<double>(valid_timeout_)))
    {
      valid_ = false; state_ = "LOST"; reason_ = "Last successful sensor scan expired";
    }
  }

  void publishStatus()
  {
    std_msgs::msg::String msg;
    std::lock_guard<std::mutex> lock(mutex_);
    const auto time = now();
      expireLocked(time.nanoseconds());
      std::ostringstream out;
      out << std::setprecision(10) << "{\"state\":" << quoted(state_) << ",\"valid\":" << (valid_ ? "true" : "false")
          << ",\"reason\":" << quoted(reason_) << ",\"stamp_ns\":" << time.nanoseconds()
          << ",\"last_success_stamp_ns\":" << last_success_stamp_
          << ",\"fitness\":";
      if (std::isfinite(fitness_)) {out << fitness_;} else {out << "null";}
      out << ",\"coverage\":" << coverage_ << ",\"initialization_mode\":" << quoted(mode_)
          << ",\"diagnostics_directory\":" << quoted(diagnostics_directory_)
          << ",\"attempt\":" << initialization_attempt_ << ",\"max_attempts\":" << max_attempts_
          << ",\"candidates\":" << candidate_status_ << "}";
      msg.data = out.str();
    status_pub_->publish(msg);
  }

  void publishTf()
  {
    const auto time = now();
    Eigen::Matrix4f transform;
    std::lock_guard<std::mutex> lock(mutex_);
    expireLocked(time.nanoseconds());
    if (!valid_) {return;}
    transform = map_T_odom_;
    const auto pose = matrixPose(transform);
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = time; msg.header.frame_id = global_frame_; msg.child_frame_id = odom_frame_;
    msg.transform.translation.x = pose.position.x; msg.transform.translation.y = pose.position.y;
    msg.transform.translation.z = pose.position.z; msg.transform.rotation = pose.orientation;
    tf_->sendTransform(msg);
  }

  std::string map_path_, mode_, global_frame_, odom_frame_, body_frame_, cloud_topic_, odom_topic_, initial_topic_;
  std::string diagnostics_directory_, candidate_status_ = "[]";
  std::string map_sha256_, config_sha256_;
  int max_attempts_ = 3, max_hypotheses_ = 3, initialization_attempt_ = 0;
  double initialization_timeout_ = 180.;
  double confirmation_coverage_gap_ = .08;
  double direct_accept_coverage_ = .91;
  double body_roll_deg_ = 180., tracking_fitness_ = 2., max_scan_age_ = 2., valid_timeout_ = 3., odom_gap_ = .1;
  double collect_seconds_ = 2., collect_timeout_ = 15., verification_timeout_ = 20.;
  double confirmation_translation_ = .3, confirmation_rotation_ = .1745;
  int period_ms_ = 500, collection_frames_ = 10, confirmation_frames_ = 3, max_collection_points_ = 120000;
  g1_relocalization::GlobalMatcherOptions options_;
  Cloud::Ptr map_;
  std::unique_ptr<g1_relocalization::GlobalMatcher> matcher_;
  Eigen::Matrix4f base_T_body_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f map_T_odom_ = Eigen::Matrix4f::Identity(), manual_guess_ = Eigen::Matrix4f::Identity();
  g1_relocalization::PoseBuffer poses_;
  std::deque<Frame, Eigen::aligned_allocator<Frame>> pending_;
  std::mutex mutex_; std::condition_variable wake_;
  std::atomic<bool> stopping_{false}; std::atomic<uint64_t> generation_{0};
  std::thread worker_;
  bool has_manual_ = false, valid_ = false;
  int64_t last_received_scan_ = 0, last_processed_scan_ = 0, last_success_stamp_ = 0;
  int64_t minimum_scan_stamp_ = 0;
  Clock::time_point last_success_steady_{};
  std::string state_ = "WAITING_DATA", reason_;
  double fitness_ = std::numeric_limits<double>::infinity(), coverage_ = 0.;
  rclcpp::CallbackGroup::SharedPtr input_group_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fitness_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr status_timer_, tf_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<G1RelocalizationNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node); executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("g1_relocalization"), "%s", error.what()); result = 1;
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return result;
}
