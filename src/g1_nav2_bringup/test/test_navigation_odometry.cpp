#include <gtest/gtest.h>
#include <dwb_plugins/one_d_velocity_iterator.hpp>
#include "g1_nav2_bringup/navigation_odometry.hpp"

namespace
{
const Eigen::Quaterniond body_from_nav(0.0, 1.0, 0.0, 0.0);

nav_msgs::msg::Odometry input_odometry()
{
  nav_msgs::msg::Odometry input;
  input.header.frame_id = "camera_init";
  input.header.stamp.sec = 42;
  input.header.stamp.nanosec = 1234;
  input.child_frame_id = "body";
  input.pose.pose.position.x = 2.0;
  input.pose.pose.orientation.x = 1.0;
  input.pose.pose.orientation.w = 0.0;
  input.twist.twist.linear.x = 0.7;
  input.twist.twist.linear.y = -0.2;
  input.twist.twist.linear.z = -0.1;
  input.twist.twist.angular.x = 0.01;
  input.twist.twist.angular.y = -0.02;
  input.twist.twist.angular.z = -0.4;
  return input;
}
}  // namespace

TEST(NavigationOdometry, CorrectsUpsideDownTwistAndPoseTogether)
{
  const auto output = g1_nav2_bringup::navigation_odometry(
    input_odometry(), body_from_nav, "nav_base");
  EXPECT_EQ(output.child_frame_id, "nav_base");
  EXPECT_NEAR(std::abs(output.pose.pose.orientation.w), 1.0, 1e-12);
  EXPECT_NEAR(output.pose.pose.orientation.x, 0.0, 1e-12);
  EXPECT_NEAR(output.twist.twist.linear.x, 0.7, 1e-12);
  EXPECT_NEAR(output.twist.twist.linear.y, 0.2, 1e-12);
  EXPECT_NEAR(output.twist.twist.linear.z, 0.1, 1e-12);
  EXPECT_NEAR(output.twist.twist.angular.x, 0.01, 1e-12);
  EXPECT_NEAR(output.twist.twist.angular.y, 0.02, 1e-12);
  EXPECT_NEAR(output.twist.twist.angular.z, 0.4, 1e-12);
}

TEST(NavigationOdometry, PreservesSensorTimestampParentAndPosition)
{
  auto input = input_odometry();
  input.pose.covariance[7] = 0.25;
  const auto output = g1_nav2_bringup::navigation_odometry(input, body_from_nav, "nav_base");
  EXPECT_EQ(output.header, input.header);
  EXPECT_EQ(output.pose.pose.position, input.pose.pose.position);
  EXPECT_EQ(output.pose.covariance, input.pose.covariance);
  EXPECT_EQ(input.child_frame_id, "body");
  EXPECT_DOUBLE_EQ(input.twist.twist.angular.z, -0.4);
}

TEST(NavigationOdometry, RotatesAllTwistCovarianceBlocks)
{
  auto input = input_odometry();
  for (int axis = 0; axis < 6; ++axis) {
    input.twist.covariance[axis * 6 + axis] = axis + 1.0;
  }
  input.twist.covariance[5] = input.twist.covariance[30] = 0.25;
  const auto output = g1_nav2_bringup::navigation_odometry(input, body_from_nav, "nav_base");
  for (int axis = 0; axis < 6; ++axis) {
    EXPECT_NEAR(output.twist.covariance[axis * 6 + axis], axis + 1.0, 1e-12);
  }
  EXPECT_NEAR(output.twist.covariance[5], -0.25, 1e-12);
  EXPECT_NEAR(output.twist.covariance[30], -0.25, 1e-12);
}

TEST(NavigationOdometry, MeasuredForwardVelocityUnlocksOneMeterPerSecondCandidate)
{
  const auto output = g1_nav2_bringup::navigation_odometry(
    input_odometry(), body_from_nav, "nav_base");
  dwb_plugins::OneDVelocityIterator iterator(
    output.twist.twist.linear.x, 0.0, 1.0, 0.2, -0.3, 1.5, 5);
  double maximum = 0.0;
  for (; !iterator.isFinished(); ++iterator) {
    maximum = iterator.getVelocity();
  }
  EXPECT_NEAR(maximum, 1.0, 1e-12);
}

TEST(NavigationOdometry, NonDefaultMountingRotatesVelocityAndCovariance)
{
  auto input = input_odometry();
  input.pose.pose.orientation.x = 0.0;
  input.pose.pose.orientation.w = 1.0;
  input.twist.twist.linear.x = 1.0;
  input.twist.twist.linear.y = 2.0;
  input.twist.covariance[0] = 1.0;
  input.twist.covariance[7] = 4.0;
  const Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  const auto output = g1_nav2_bringup::navigation_odometry(input, rotation, "nav_base");
  EXPECT_NEAR(output.twist.twist.linear.x, 2.0, 1e-12);
  EXPECT_NEAR(output.twist.twist.linear.y, -1.0, 1e-12);
  EXPECT_NEAR(output.twist.covariance[0], 4.0, 1e-12);
  EXPECT_NEAR(output.twist.covariance[7], 1.0, 1e-12);
  EXPECT_NEAR(output.pose.pose.orientation.z, std::sin(M_PI / 4), 1e-12);
}

TEST(NavigationOdometry, LateralSamplingIsFiniteAndAlwaysZero)
{
  dwb_plugins::OneDVelocityIterator iterator(0.0, 0.0, 0.0, 0.2, -0.3, 1.5, 1);
  int samples = 0;
  for (; !iterator.isFinished(); ++iterator) {
    EXPECT_DOUBLE_EQ(iterator.getVelocity(), 0.0);
    ASSERT_LT(++samples, 4);
  }
  EXPECT_GT(samples, 0);
}
