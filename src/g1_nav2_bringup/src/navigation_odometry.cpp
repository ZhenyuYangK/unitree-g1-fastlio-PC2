#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "g1_nav2_bringup/navigation_odometry.hpp"

class NavigationOdometry : public rclcpp::Node
{
public:
  NavigationOdometry() : Node("navigation_odometry")
  {
    const auto input_topic = declare_fixed_parameter<std::string>("input_topic", "/Odometry");
    const auto output_topic = declare_fixed_parameter<std::string>("output_topic", "/Odometry_nav");
    body_frame_ = declare_fixed_parameter<std::string>("body_frame", "body");
    navigation_frame_ = declare_fixed_parameter<std::string>("navigation_frame", "nav_base");
    const auto rotation = declare_fixed_parameter<std::vector<double>>(
      "body_to_nav_rotation_xyzw", {1.0, 0.0, 0.0, 0.0});
    if (rotation.size() != 4) {
      throw std::invalid_argument("body_to_nav_rotation_xyzw must contain four values");
    }
    body_from_nav_ = Eigen::Quaterniond(rotation[3], rotation[0], rotation[1], rotation[2]);
    if (!body_from_nav_.coeffs().allFinite() || body_from_nav_.norm() < 1e-6) {
      throw std::invalid_argument("body_to_nav_rotation_xyzw must be a finite quaternion");
    }
    if (body_frame_.empty() || navigation_frame_.empty() || input_topic == output_topic) {
      throw std::invalid_argument("Frames must be nonempty and odometry topics must differ");
    }
    body_from_nav_.normalize();
    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, 10);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        const auto & q = message->pose.pose.orientation;
        const auto & v = message->twist.twist.linear;
        const auto & w = message->twist.twist.angular;
        const Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
        if (message->header.frame_id.empty() || message->child_frame_id != body_frame_ ||
          !orientation.coeffs().allFinite() || orientation.norm() < 1e-6 ||
          !Eigen::Vector3d(v.x, v.y, v.z).allFinite() ||
          !Eigen::Vector3d(w.x, w.y, w.z).allFinite())
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Discarding odometry with invalid frames, orientation or velocity");
          return;
        }
        const auto output = g1_nav2_bringup::navigation_odometry(
          *message, body_from_nav_, navigation_frame_);
        publisher_->publish(output);
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "Navigation velocity feedback: vx=%.3f vy=%.3f yaw=%.3f in %s",
          output.twist.twist.linear.x, output.twist.twist.linear.y,
          output.twist.twist.angular.z, navigation_frame_.c_str());
      });
    RCLCPP_INFO(
      get_logger(), "Odometry: %s (%s) -> %s (%s), co-located frame rotation",
      input_topic.c_str(), body_frame_.c_str(), output_topic.c_str(), navigation_frame_.c_str());
  }

private:
  template<typename T>
  T declare_fixed_parameter(const std::string & name, const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }

  std::string body_frame_;
  std::string navigation_frame_;
  Eigen::Quaterniond body_from_nav_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationOdometry>());
  rclcpp::shutdown();
  return 0;
}
