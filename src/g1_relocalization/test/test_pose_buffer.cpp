#include <gtest/gtest.h>
#include <g1_relocalization/pose_buffer.hpp>
#include <limits>

using g1_relocalization::PoseBuffer;

TEST(PoseBuffer, InterpolatesScanTimeRatherThanUsingLatestPose)
{
  PoseBuffer buffer;
  auto first = Eigen::Matrix4f::Identity().eval();
  auto second = first;
  second(0, 3) = 2.f;
  second.block<3, 3>(0, 0) = Eigen::AngleAxisf(1.f, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  ASSERT_TRUE(buffer.add(1000000000, first));
  ASSERT_TRUE(buffer.add(1200000000, second));
  Eigen::Matrix4f result;
  ASSERT_TRUE(buffer.lookup(1100000000, .15, result));
  EXPECT_NEAR(result(0, 3), 1.f, 1e-5);
  EXPECT_NEAR(std::atan2(result(1, 0), result(0, 0)), .5f, 1e-5);
  EXPECT_FALSE(buffer.lookup(1600000000, .15, result));
  EXPECT_FALSE(buffer.lookup(500000000, .15, result));
}

TEST(PoseBuffer, RejectsOutOfOrderAndNonFiniteOdom)
{
  PoseBuffer buffer;
  auto pose = Eigen::Matrix4f::Identity().eval();
  EXPECT_FALSE(buffer.add(0, pose));
  ASSERT_TRUE(buffer.add(1000000000, pose));
  EXPECT_FALSE(buffer.add(1000000000, pose));
  EXPECT_FALSE(buffer.add(900000000, pose));
  pose(0, 3) = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(buffer.add(1100000000, pose));
  Eigen::Matrix4f result;
  buffer.clear();
  EXPECT_FALSE(buffer.lookup(1000000000, .1, result));
}

TEST(ScanFreshness, RejectsZeroOldAndFutureSensorTimestamps)
{
  EXPECT_TRUE(g1_relocalization::freshStamp(1000000000, 1100000000, .5));
  EXPECT_FALSE(g1_relocalization::freshStamp(0, 1100000000, .5));
  EXPECT_FALSE(g1_relocalization::freshStamp(1000000000, 2000000000, .5));
  EXPECT_FALSE(g1_relocalization::freshStamp(1500000000, 1100000000, .5));
}
