#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include "g1_relocalization/pose_buffer.hpp"

// P2 retains manual initialization. Global search and coverage scoring belong to P3.
class G1RelocalizationNode : public rclcpp::Node
{
  using Clock = std::chrono::steady_clock;
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<Point>;
  struct Frame {Cloud::Ptr cloud; int64_t stamp;};

public:
  G1RelocalizationNode() : Node("g1_relocalization")
  {
    readParameters();
    base_T_body_.block<3, 3>(0, 0) = Eigen::AngleAxisf(
      body_roll_deg_ * 3.14159265358979323846 / 180., Eigen::Vector3f::UnitX()).toRotationMatrix();
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/global_pcd_map", rclcpp::QoS(1).reliable().transient_local());
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization_pose", 10);
    fitness_pub_ = create_publisher<std_msgs::msg::Float64>("/localization_fitness", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/g1_relocalization/status", rclcpp::QoS(1).reliable().transient_local());
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    loadMap();

    input_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = input_group_;
    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(2);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic_, qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {cloudCallback(msg);}, options);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic_, qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {odomCallback(msg);}, options);
    initial_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(initial_topic_, 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {initialCallback(msg);}, options);
    match_timer_ = create_wall_timer(std::chrono::milliseconds(period_ms_),
      [this]() {match();});
    // Health and TF must expire even while a slow NDT call is still running.
    health_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    status_timer_ = create_wall_timer(std::chrono::milliseconds(200),
      [this]() {publishStatus();}, health_group_);
    tf_timer_ = create_wall_timer(std::chrono::milliseconds(100),
      [this]() {publishTf();}, health_group_);
    publishStatus();
    RCLCPP_INFO(get_logger(), "Manual relocalization: waiting for /initialpose and fresh synchronized scans");
  }

private:
  void readParameters()
  {
    map_path_ = declare_parameter<std::string>("map_path", "");
    const auto mode = declare_parameter<std::string>("initialization_mode", "manual");
    if (mode != "manual") {throw std::invalid_argument("P2 supports initialization_mode=manual only");}
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "camera_init");
    body_frame_ = declare_parameter<std::string>("body_frame", "body");
    cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/cloud_registered_body");
    odom_topic_ = declare_parameter<std::string>("odometry_topic", "/Odometry");
    initial_topic_ = declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    body_roll_deg_ = declare_parameter<double>("body_roll_deg", 180.);
    map_leaf_ = declare_parameter<double>("map_voxel_size", .25);
    scan_leaf_ = declare_parameter<double>("scan_voxel_size", .20);
    fitness_limit_ = declare_parameter<double>("fitness_threshold", 2.);
    max_scan_age_ = declare_parameter<double>("max_scan_age_sec", 2.);
    valid_timeout_ = declare_parameter<double>("localization_valid_timeout_sec", 3.);
    odom_gap_ = declare_parameter<double>("max_odom_sync_gap_sec", .1);
    confirmation_frames_ = declare_parameter<int>("confirmation_scan_count", 3);
    confirmation_translation_ = declare_parameter<double>("confirmation_max_translation", .30);
    confirmation_rotation_ = declare_parameter<double>("confirmation_max_rotation_deg", 10.) *
      3.14159265358979323846 / 180.;
    period_ms_ = declare_parameter<int>("localization_period_ms", 500);
    const double resolution = declare_parameter<double>("ndt_resolution", 1.);
    const double step = declare_parameter<double>("ndt_step_size", .1);
    const double epsilon = declare_parameter<double>("ndt_transformation_epsilon", .01);
    const int iterations = declare_parameter<int>("ndt_max_iterations", 40);
    for (double value : {map_leaf_, scan_leaf_, fitness_limit_, max_scan_age_, valid_timeout_,
        odom_gap_, confirmation_translation_, confirmation_rotation_, resolution, step, epsilon})
    {
      if (!std::isfinite(value) || value <= 0) {throw std::invalid_argument("Limits must be finite and positive");}
    }
    if (!std::isfinite(body_roll_deg_) || period_ms_ <= 0 || iterations <= 0 || confirmation_frames_ < 2 ||
      max_scan_age_ > valid_timeout_ || valid_timeout_ > 3. || map_path_.empty() ||
      global_frame_.empty() || odom_frame_.empty() || body_frame_.empty() ||
      cloud_topic_.empty() || odom_topic_.empty() || initial_topic_.empty())
    {throw std::invalid_argument("Invalid frames, timing, map or confirmation parameters");}
    ndt_.setResolution(resolution);
    ndt_.setStepSize(step);
    ndt_.setTransformationEpsilon(epsilon);
    ndt_.setMaximumIterations(iterations);
  }

  static int64_t stampNs(const builtin_interfaces::msg::Time & stamp)
  {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) {return 0;}
    return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  }

  static bool poseMatrix(const geometry_msgs::msg::Pose & pose, Eigen::Matrix4f & result)
  {
    Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    if (!q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-6f) {return false;}
    result = Eigen::Matrix4f::Identity();
    result.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    result.block<3, 1>(0, 3) = Eigen::Vector3f(pose.position.x, pose.position.y, pose.position.z);
    return result.allFinite();
  }

  static geometry_msgs::msg::Pose matrixPose(const Eigen::Matrix4f & matrix)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = matrix(0, 3); pose.position.y = matrix(1, 3); pose.position.z = matrix(2, 3);
    Eigen::Quaternionf q(matrix.block<3, 3>(0, 0));
    q.normalize();
    pose.orientation.x = q.x(); pose.orientation.y = q.y();
    pose.orientation.z = q.z(); pose.orientation.w = q.w();
    return pose;
  }

  static Cloud::Ptr filter(const Cloud::Ptr & input, double leaf)
  {
    auto finite = Cloud::Ptr(new Cloud);
    for (const auto & point : *input) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {finite->push_back(point);}
    }
    auto output = Cloud::Ptr(new Cloud);
    pcl::VoxelGrid<Point> voxel;
    voxel.setInputCloud(finite);
    const float size = static_cast<float>(leaf);
    voxel.setLeafSize(size, size, size);
    voxel.filter(*output);
    return output;
  }

  void loadMap()
  {
    auto raw = Cloud::Ptr(new Cloud);
    if (pcl::io::loadPCDFile<Point>(map_path_, *raw) < 0) {throw std::runtime_error("Cannot load map: " + map_path_);}
    map_ = filter(raw, map_leaf_);
    if (map_->size() < 100) {throw std::runtime_error("Map has fewer than 100 finite filtered points");}
    ndt_.setInputTarget(map_);
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*map_, msg);
    msg.header.frame_id = global_frame_; msg.header.stamp = now();
    map_pub_->publish(msg);
  }

  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    const auto stamp = stampNs(msg->header.stamp);
    if (msg->header.frame_id != body_frame_ ||
      !g1_relocalization::freshStamp(stamp, now().nanoseconds(), max_scan_age_)) {return;}
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stamp <= last_received_scan_) {return;}
    }
    auto raw = Cloud::Ptr(new Cloud);
    pcl::fromROSMsg(*msg, *raw);
    auto cloud = filter(raw, scan_leaf_);
    if (cloud->size() < 100) {return;}
    std::lock_guard<std::mutex> lock(mutex_);
    last_received_scan_ = stamp;
    pending_.push_back({cloud, stamp});
    while (pending_.size() > 30) {pending_.pop_front();}
  }

  void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    const auto stamp = stampNs(msg->header.stamp);
    Eigen::Matrix4f pose;
    if (msg->header.frame_id != odom_frame_ || msg->child_frame_id != body_frame_ ||
      !g1_relocalization::freshStamp(stamp, now().nanoseconds(), max_scan_age_) ||
      !poseMatrix(msg->pose.pose, pose)) {return;}
    std::lock_guard<std::mutex> lock(mutex_);
    poses_.add(stamp, pose);
  }

  void initialCallback(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    Eigen::Matrix4f pose;
    if ((!msg->header.frame_id.empty() && msg->header.frame_id != global_frame_) ||
      !poseMatrix(msg->pose.pose, pose))
    {RCLCPP_WARN(get_logger(), "Rejected invalid initialpose/frame"); return;}
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++generation_;
      initial_guess_ = pose * base_T_body_;
      has_initial_ = true; has_candidate_ = false; valid_ = false;
      confirmations_ = 0; last_success_stamp_ = 0;
      // No cached scan acquired before this initialization may confirm it.
      verification_after_ = now().nanoseconds(); pending_.clear();
      state_ = "VERIFYING"; reason_ = "Verifying manual initialpose on distinct new scans";
      fitness_ = std::numeric_limits<double>::infinity();
      publishFitness(fitness_);
    }
    publishStatus();
  }

  void expireLocked(int64_t ros_now)
  {
    if (valid_ && (!g1_relocalization::freshStamp(last_success_stamp_, ros_now, valid_timeout_) ||
      Clock::now() - last_success_steady_ > std::chrono::duration<double>(valid_timeout_)))
    {
      valid_ = false; confirmations_ = 0;
      state_ = "LOST"; reason_ = "Last successful sensor scan expired";
    }
  }

  void rejectLocked(const std::string & reason)
  {
    state_ = valid_ || last_success_stamp_ > 0 ? "LOST" : "VERIFYING";
    reason_ = reason; valid_ = false; confirmations_ = 0;
    fitness_ = std::numeric_limits<double>::infinity();
    publishFitness(fitness_);
  }

  void match()
  {
    Frame frame;
    Eigen::Matrix4f odom_body, guess;
    uint64_t generation;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      expireLocked(now().nanoseconds());
      if (!has_initial_) {return;}
      bool found = false;
      for (auto it = pending_.rbegin(); it != pending_.rend(); ++it) {
        if (it->stamp <= last_processed_scan_ || it->stamp <= verification_after_ ||
          !g1_relocalization::freshStamp(it->stamp, now().nanoseconds(), max_scan_age_) ||
          !poses_.lookup(it->stamp, odom_gap_, odom_body)) {continue;}
        frame = *it; found = true; break;
      }
      if (!found) {return;}
      last_processed_scan_ = frame.stamp;
      while (!pending_.empty() && pending_.front().stamp <= frame.stamp) {pending_.pop_front();}
      guess = has_candidate_ ? (map_T_odom_ * odom_body).eval() : initial_guess_;
      generation = generation_;
    }
    ndt_.setInputSource(frame.cloud);
    Cloud aligned;
    ndt_.align(aligned, guess);
    const double fitness = ndt_.hasConverged() ? ndt_.getFitnessScore() : std::numeric_limits<double>::infinity();
    const Eigen::Matrix4f fitted = ndt_.getFinalTransformation();
    const Eigen::Matrix4f candidate = fitted * odom_body.inverse();
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation != generation_) {return;}
    expireLocked(now().nanoseconds());
    if (!ndt_.hasConverged() || !candidate.allFinite() || !std::isfinite(fitness) ||
      fitness < 0 || fitness > fitness_limit_ ||
      !g1_relocalization::freshStamp(frame.stamp, now().nanoseconds(), max_scan_age_))
    {rejectLocked("NDT/fitness/scan freshness rejected"); return;}
    if (has_candidate_) {
      const auto & reference = valid_ ? map_T_odom_ : anchor_;
      const double translation = (candidate.block<3, 1>(0, 3) - reference.block<3, 1>(0, 3)).norm();
      const Eigen::Quaternionf first(candidate.block<3, 3>(0, 0)), second(reference.block<3, 3>(0, 0));
      const double rotation = first.angularDistance(second);
      if (translation > (valid_ ? .75 : confirmation_translation_) ||
        rotation > (valid_ ? .4 : confirmation_rotation_))
      {rejectLocked("NDT transform stability rejected"); return;}
    } else {
      anchor_ = candidate; has_candidate_ = true;
    }
    map_T_odom_ = candidate;
    confirmations_ = std::min(confirmations_ + 1, confirmation_frames_);
    if (!valid_ && confirmations_ < confirmation_frames_) {
      state_ = "VERIFYING";
      reason_ = "Fresh independent confirmation " + std::to_string(confirmations_) + "/" + std::to_string(confirmation_frames_);
      return;
    }
    const bool first = !valid_;
    valid_ = true; state_ = first ? "LOCALIZED" : "TRACKING";
    reason_ = "Confirmed on distinct fresh synchronized scans";
    anchor_ = candidate; last_success_stamp_ = frame.stamp;
    last_success_steady_ = Clock::now(); fitness_ = fitness;
    publishFitness(fitness);
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = global_frame_; pose.header.stamp = rclcpp::Time(frame.stamp);
    pose.pose.pose = matrixPose(fitted * base_T_body_.inverse());
    pose.pose.covariance[0] = pose.pose.covariance[7] = pose.pose.covariance[14] = .04;
    pose.pose.covariance[21] = pose.pose.covariance[28] = pose.pose.covariance[35] = .01;
    pose_pub_->publish(pose);
    if (first) {RCLCPP_INFO(get_logger(), "NDT success: confirmed fitness=%.6f", fitness);}
  }

  void publishFitness(double fitness)
  {
    std_msgs::msg::Float64 msg; msg.data = fitness; fitness_pub_->publish(msg);
  }

  void publishStatus()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto ros_now = now().nanoseconds();
    expireLocked(ros_now);
    std::ostringstream out;
    // All dynamic text is internal; none contains JSON control characters.
    out << std::setprecision(10) << "{\"state\":\"" << state_ << "\",\"valid\":" << (valid_ ? "true" : "false")
        << ",\"reason\":\"" << reason_ << "\",\"stamp_ns\":" << ros_now
        << ",\"last_success_stamp_ns\":" << last_success_stamp_ << ",\"fitness\":";
    if (std::isfinite(fitness_)) {out << fitness_;} else {out << "null";}
    out << ",\"coverage\":0,\"initialization_mode\":\"manual\",\"attempt\":0,\"max_attempts\":0,\"candidates\":[]}";
    std_msgs::msg::String msg; msg.data = out.str(); status_pub_->publish(msg);
  }

  void publishTf()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto ros_now = now();
    expireLocked(ros_now.nanoseconds());
    if (!valid_) {return;}
    const auto pose = matrixPose(map_T_odom_);
    geometry_msgs::msg::TransformStamped msg;
    msg.header.frame_id = global_frame_; msg.child_frame_id = odom_frame_; msg.header.stamp = ros_now;
    msg.transform.translation.x = pose.position.x; msg.transform.translation.y = pose.position.y;
    msg.transform.translation.z = pose.position.z; msg.transform.rotation = pose.orientation;
    tf_->sendTransform(msg);
  }

  std::string map_path_, global_frame_, odom_frame_, body_frame_, cloud_topic_, odom_topic_, initial_topic_;
  double body_roll_deg_, map_leaf_, scan_leaf_, fitness_limit_, max_scan_age_, valid_timeout_, odom_gap_;
  double confirmation_translation_, confirmation_rotation_;
  int period_ms_, confirmation_frames_, confirmations_ = 0;
  Cloud::Ptr map_;
  pcl::NormalDistributionsTransform<Point, Point> ndt_;
  Eigen::Matrix4f base_T_body_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f initial_guess_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f map_T_odom_ = Eigen::Matrix4f::Identity(), anchor_ = Eigen::Matrix4f::Identity();
  g1_relocalization::PoseBuffer poses_;
  std::deque<Frame> pending_;
  std::mutex mutex_;
  uint64_t generation_ = 0;
  int64_t last_received_scan_ = 0, last_processed_scan_ = 0, last_success_stamp_ = 0, verification_after_ = 0;
  bool has_initial_ = false, has_candidate_ = false, valid_ = false;
  double fitness_ = std::numeric_limits<double>::infinity();
  Clock::time_point last_success_steady_{};
  std::string state_ = "WAITING_INITIAL_POSE", reason_ = "Waiting for /initialpose";
  rclcpp::CallbackGroup::SharedPtr input_group_, health_group_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fitness_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_sub_;
  rclcpp::TimerBase::SharedPtr match_timer_, status_timer_, tf_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<G1RelocalizationNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node); executor.spin();
  executor.remove_node(node); node.reset();
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return 0;
}
