#ifndef G1_NAV2_BRINGUP_NAVIGATION_ODOMETRY_HPP_
#define G1_NAV2_BRINGUP_NAVIGATION_ODOMETRY_HPP_

#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>

namespace g1_nav2_bringup
{
// body and nav_base share an origin. body_from_nav is the same fixed rotation
// as the body -> nav_base static TF; this function does not publish a TF.
inline nav_msgs::msg::Odometry navigation_odometry(
  const nav_msgs::msg::Odometry & input,
  const Eigen::Quaterniond & body_from_nav,
  const std::string & navigation_frame)
{
  auto output = input;
  output.child_frame_id = navigation_frame;
  const auto & orientation = input.pose.pose.orientation;
  const Eigen::Quaterniond world_from_body(
    orientation.w, orientation.x, orientation.y, orientation.z);
  const Eigen::Quaterniond world_from_nav =
    world_from_body.normalized() * body_from_nav;
  output.pose.pose.orientation.x = world_from_nav.x();
  output.pose.pose.orientation.y = world_from_nav.y();
  output.pose.pose.orientation.z = world_from_nav.z();
  output.pose.pose.orientation.w = world_from_nav.w();

  const Eigen::Matrix3d nav_from_body = body_from_nav.toRotationMatrix().transpose();
  const auto & twist = input.twist.twist;
  const Eigen::Vector3d linear = nav_from_body * Eigen::Vector3d(
    twist.linear.x, twist.linear.y, twist.linear.z);
  const Eigen::Vector3d angular = nav_from_body * Eigen::Vector3d(
    twist.angular.x, twist.angular.y, twist.angular.z);
  output.twist.twist.linear.x = linear.x();
  output.twist.twist.linear.y = linear.y();
  output.twist.twist.linear.z = linear.z();
  output.twist.twist.angular.x = angular.x();
  output.twist.twist.angular.y = angular.y();
  output.twist.twist.angular.z = angular.z();

  Eigen::Matrix<double, 6, 6> rotation = Eigen::Matrix<double, 6, 6>::Zero();
  rotation.block<3, 3>(0, 0) = nav_from_body;
  rotation.block<3, 3>(3, 3) = nav_from_body;
  using RowMajorCovariance = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;
  const Eigen::Map<const RowMajorCovariance> covariance(input.twist.covariance.data());
  Eigen::Map<RowMajorCovariance> output_covariance(output.twist.covariance.data());
  output_covariance = rotation * covariance * rotation.transpose();
  // Pose covariance is expressed in the parent frame with fixed-axis errors.
  // A co-located child-frame rotation leaves those errors unchanged.
  return output;
}
}  // namespace g1_nav2_bringup

#endif  // G1_NAV2_BRINGUP_NAVIGATION_ODOMETRY_HPP_
