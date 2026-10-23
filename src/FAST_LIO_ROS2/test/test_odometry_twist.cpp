#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include "odometry_twist.hpp"

using Covariance = Eigen::Matrix<double, 23, 23>;

TEST(OdometryTwist, StationaryGyroBiasDoesNotBecomeRotation)
{
  nav_msgs::msg::Odometry odometry;
  const Eigen::Vector3d bias(0.02, -0.01, 0.03);
  fast_lio::set_body_twist(
    odometry, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
    bias, bias, Covariance::Zero().eval(), Eigen::Vector3d::Zero());
  EXPECT_DOUBLE_EQ(odometry.twist.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(odometry.twist.twist.angular.z, 0.0);
}

TEST(OdometryTwist, WorldVelocityIsRotatedIntoBodyAtArbitraryHeading)
{
  nav_msgs::msg::Odometry odometry;
  const auto rotation = Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d body_velocity(0.7, -0.2, 0.1);
  fast_lio::set_body_twist(
    odometry, rotation, rotation * body_velocity, Eigen::Vector3d(0.02, -0.01, 0.43),
    Eigen::Vector3d(0.02, -0.01, 0.03), Covariance::Zero().eval(), Eigen::Vector3d::Zero());
  EXPECT_NEAR(odometry.twist.twist.linear.x, 0.7, 1e-12);
  EXPECT_NEAR(odometry.twist.twist.linear.y, -0.2, 1e-12);
  EXPECT_NEAR(odometry.twist.twist.linear.z, 0.1, 1e-12);
  EXPECT_NEAR(odometry.twist.twist.angular.z, 0.4, 1e-12);
}

TEST(OdometryTwist, UpsideDownBodyKeepsItsOwnAngularConvention)
{
  nav_msgs::msg::Odometry odometry;
  const auto rotation = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0).toRotationMatrix();
  fast_lio::set_body_twist(
    odometry, rotation, Eigen::Vector3d(0.6, 0.2, 0.0),
    Eigen::Vector3d(0.0, 0.0, -0.4), Eigen::Vector3d::Zero(),
    Covariance::Zero().eval(), Eigen::Vector3d::Zero());
  EXPECT_NEAR(odometry.twist.twist.linear.x, 0.6, 1e-12);
  EXPECT_NEAR(odometry.twist.twist.linear.y, -0.2, 1e-12);
  EXPECT_NEAR(odometry.twist.twist.angular.z, -0.4, 1e-12);
}

TEST(OdometryTwist, CovarianceIncludesHeadingBiasAndCrossCorrelation)
{
  nav_msgs::msg::Odometry odometry;
  Covariance state_covariance = Covariance::Zero();
  state_covariance(5, 5) = 0.1;
  state_covariance.block<3, 3>(12, 12).diagonal() << 1.0, 4.0, 9.0;
  state_covariance.block<3, 3>(15, 15).diagonal() << 0.01, 0.02, 0.03;
  state_covariance(12, 15) = state_covariance(15, 12) = 0.05;
  fast_lio::set_body_twist(
    odometry, Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
    Eigen::Vector3d(0.0, 0.6, 0.0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    state_covariance, Eigen::Vector3d(0.1, 0.2, 0.3));
  EXPECT_NEAR(odometry.twist.covariance[0], 4.0, 1e-12);
  EXPECT_NEAR(odometry.twist.covariance[7], 1.036, 1e-12);
  EXPECT_NEAR(odometry.twist.covariance[14], 9.0, 1e-12);
  EXPECT_NEAR(odometry.twist.covariance[21], 0.11, 1e-12);
  EXPECT_NEAR(odometry.twist.covariance[35], 0.33, 1e-12);
  EXPECT_NEAR(odometry.twist.covariance[9], 0.05, 1e-12);
  EXPECT_NEAR(odometry.twist.covariance[19], 0.05, 1e-12);
}
