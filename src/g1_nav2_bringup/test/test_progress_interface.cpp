#include <limits>
#include <memory>
#include <gtest/gtest.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/progress_checker.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class ProgressInterfaceTest : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
  static void TearDownTestCase() {rclcpp::shutdown();}
  void SetUp() override
  {
    loader_.reset(new pluginlib::ClassLoader<nav2_core::ProgressChecker>(
      "nav2_core", "nav2_core::ProgressChecker"));
    checker_ = loader_->createSharedInstance("g1_nav2_bringup/GoalProgressChecker");
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("progress_interface_test");
    checker_->initialize(node_, "progress_checker");
    pose_.pose.orientation.w = 1.0;
  }
  void TearDown() override
  {
    checker_.reset();
    node_.reset();
    loader_.reset();
  }
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::ProgressChecker>> loader_;
  std::shared_ptr<nav2_core::ProgressChecker> checker_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  geometry_msgs::msg::PoseStamped pose_;
};

TEST_F(ProgressInterfaceTest, StationaryValidPoseRemainsAccepted)
{
  for (int i = 0; i < 100; ++i) {EXPECT_TRUE(checker_->check(pose_));}
  checker_->reset();
  EXPECT_TRUE(checker_->check(pose_));
}
TEST_F(ProgressInterfaceTest, InvalidPoseStillRejected)
{
  pose_.pose.position.x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(checker_->check(pose_));
  pose_.pose.position.x = 0.0;
  pose_.pose.orientation.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(checker_->check(pose_));
}
