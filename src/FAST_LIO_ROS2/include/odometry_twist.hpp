#ifndef FAST_LIO_ODOMETRY_TWIST_HPP_
#define FAST_LIO_ODOMETRY_TWIST_HPP_

#include <Eigen/Core>
#include <nav_msgs/msg/odometry.hpp>

namespace fast_lio
{
// FAST-LIO velocity is in the world frame. Odometry twist must instead be
// expressed in child_frame_id (the IMU/body frame), including its covariance.
template<typename StateCovariance>
void set_body_twist(
  nav_msgs::msg::Odometry & odometry,
  const Eigen::Matrix3d & world_from_body,
  const Eigen::Vector3d & world_velocity,
  const Eigen::Vector3d & measured_gyro,
  const Eigen::Vector3d & gyro_bias,
  const StateCovariance & state_covariance,
  const Eigen::Vector3d & gyro_variance)
{
  const Eigen::Matrix3d body_from_world = world_from_body.transpose();
  const Eigen::Vector3d linear = body_from_world * world_velocity;
  const Eigen::Vector3d angular = measured_gyro - gyro_bias;
  odometry.twist.twist.linear.x = linear.x();
  odometry.twist.twist.linear.y = linear.y();
  odometry.twist.twist.linear.z = linear.z();
  odometry.twist.twist.angular.x = angular.x();
  odometry.twist.twist.angular.y = angular.y();
  odometry.twist.twist.angular.z = angular.z();

  // IKFoM error-state indices: attitude 3, world velocity 12, gyro bias 15.
  // Attitude uses a right perturbation: d(R^T v)/d(theta) = skew(R^T v).
  Eigen::Matrix<double, 6, StateCovariance::ColsAtCompileTime> jacobian =
    Eigen::Matrix<double, 6, StateCovariance::ColsAtCompileTime>::Zero();
  Eigen::Matrix3d skew;
  skew << 0.0, -linear.z(), linear.y(),
    linear.z(), 0.0, -linear.x(),
    -linear.y(), linear.x(), 0.0;
  jacobian.template block<3, 3>(0, 3) = skew;
  jacobian.template block<3, 3>(0, 12) = body_from_world;
  jacobian.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 6, 6> covariance =
    jacobian * state_covariance * jacobian.transpose();
  covariance.template block<3, 3>(3, 3).diagonal() += gyro_variance;
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      odometry.twist.covariance[row * 6 + col] = covariance(row, col);
    }
  }
}
}  // namespace fast_lio

#endif  // FAST_LIO_ODOMETRY_TWIST_HPP_
