#include <cmath>
#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/progress_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace g1_nav2_bringup
{
class GoalProgressChecker : public nav2_core::ProgressChecker
{
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & /*name*/) override
  {
    RCLCPP_INFO(node->get_logger(), "Navigation progress timeouts disabled by user request.");
  }
  void reset() override
  {
  }
  bool check(geometry_msgs::msg::PoseStamped & pose) override
  {
    // Foxy requires a ProgressChecker plugin. The operator disabled all
    // displacement/heading/stall deadlines. Keep the original finite-pose
    // rejection; this interface no longer measures elapsed time or progress.
    const auto & p = pose.pose.position;
    const auto & q = pose.pose.orientation;
    const double yaw = std::atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z));
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(yaw);
  }
};
}  // namespace g1_nav2_bringup
PLUGINLIB_EXPORT_CLASS(g1_nav2_bringup::GoalProgressChecker, nav2_core::ProgressChecker)
