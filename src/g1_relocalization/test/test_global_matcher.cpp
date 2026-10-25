#include "g1_relocalization/global_matcher.hpp"

#include <cmath>
#include <limits>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>

namespace
{
using namespace g1_relocalization;
constexpr double kPi = 3.14159265358979323846;

void wall(GlobalCloud & cloud, float x0, float y0, float x1, float y1, float offset)
{
  const int steps = std::ceil(std::hypot(x1 - x0, y1 - y0) / 0.12);
  for (int i = 0; i <= steps; ++i) {
    const float fraction = (i + offset) / steps;
    for (float z = -0.6 + offset * 0.05; z <= 1.4; z += 0.14) {
      GlobalPoint point;
      point.x = x0 + fraction * (x1 - x0);
      point.y = y0 + fraction * (y1 - y0);
      point.z = z;
      point.intensity = 1.0;
      cloud.push_back(point);
    }
  }
}

GlobalCloud::Ptr room(float offset = 0.0f)
{
  auto cloud = GlobalCloud::Ptr(new GlobalCloud);
  wall(*cloud, -4, -3, 5, -3, offset);
  wall(*cloud, -4, -3, -4, 4, offset);
  wall(*cloud, -4, 4, 1, 4, offset);
  wall(*cloud, 1, 4, 1, 1, offset);
  wall(*cloud, 1, 1, 5, 1, offset);
  wall(*cloud, 5, 1, 5, -3, offset);
  // Two interior structures of different shape make pose/yaw identifiable.
  wall(*cloud, -1.5, -1, -0.5, -1, offset);
  wall(*cloud, -1.5, -1, -1.5, 0.5, offset);
  wall(*cloud, 2.5, -1.8, 2.5, -0.4, offset);
  wall(*cloud, 2.5, -0.4, 3.8, -0.4, offset);
  return cloud;
}

GlobalMatcherOptions settings()
{
  GlobalMatcherOptions options;
  options.use_search_bounds = true;
  options.min_x = -5;
  options.max_x = 6;
  options.min_y = -4;
  options.max_y = 5;
  options.coarse_yaw_step_deg = 15;
  options.retained_modes = 40;
  options.ndt_candidates = 10;
  options.min_scan_points = 80;
  options.time_budget_seconds = 20;
  options.scan_voxel_size = 0.18;
  options.ndt_resolution = 0.8;
  options.max_fitness = 0.4;
  options.unresolved_score_gap = 0.015;
  return options;
}

Eigen::Matrix4f pose(float x, float y, float yaw, float z = 0.0)
{
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  transform.block<3, 1>(0, 3) = Eigen::Vector3f(x, y, z);
  return transform;
}

GlobalCloud::Ptr independentScan(const Eigen::Matrix4f & map_T_scan, float sample_offset = 0.41f)
{
  auto scan = GlobalCloud::Ptr(new GlobalCloud);
  // Different sample positions from map points, with deterministic sensor noise
  // and incomplete visibility. This is deliberately not the map as input scan.
  auto independent = room(sample_offset);
  pcl::transformPointCloud(*independent, *scan, map_T_scan.inverse());
  GlobalCloud visible;
  for (std::size_t i = 0; i < scan->size(); ++i) {
    if (i % 7 == 0) {continue;}
    auto point = (*scan)[i];
    point.x += 0.012f * std::sin(i * 0.61f + sample_offset * 17.0f);
    point.y += 0.009f * std::cos(i * 0.37f + sample_offset * 19.0f);
    point.z += 0.008f * std::sin(i * 0.29f + sample_offset * 11.0f);
    visible.push_back(point);
  }
  *scan = visible;
  return scan;
}

TEST(GlobalMatcher, FindsColdGlobalPoseWithFullYawAndIndependentPoints)
{
  GlobalMatcher matcher(room(), settings());
  const auto expected = pose(2.1f, -1.2f, 137.0f * kPi / 180.0f);
  const auto scan = independentScan(expected);
  const auto result = matcher.search(scan);
  ASSERT_EQ(result.status, GlobalMatchStatus::SUCCESS) << result.reason;
  ASSERT_FALSE(result.candidates.empty());
  const auto & match = result.candidates.front();
  EXPECT_LT((match.map_T_scan.block<3, 1>(0, 3) - expected.block<3, 1>(0, 3)).norm(), 0.2);
  const Eigen::Matrix3f rotation = expected.block<3, 3>(0, 0).transpose() *
    match.map_T_scan.block<3, 3>(0, 0);
  EXPECT_LT(std::abs(std::atan2(rotation(1, 0), rotation(0, 0))), 3.0 * kPi / 180.0);
  EXPECT_GT(match.quality.coverage, 0.9);
  // A distinct new observation must independently support the output pose.
  EXPECT_TRUE(matcher.evaluate(independentScan(expected, 0.73f), match.map_T_scan).acceptable);
}

TEST(GlobalMatcher, DuplicateRoomsRemainAmbiguous)
{
  auto map = room();
  GlobalCloud duplicate;
  pcl::transformPointCloud(*map, duplicate, pose(14, 0, 0));
  *map += duplicate;
  auto options = settings();
  options.max_x = 20;
  options.retained_modes = 64;
  options.ndt_candidates = 16;
  GlobalMatcher matcher(map, options);
  const auto result = matcher.search(independentScan(pose(2.1f, -1.2f, -1.07f)));
  EXPECT_EQ(result.status, GlobalMatchStatus::AMBIGUOUS) << result.reason;
  std::size_t accepted = 0;
  for (const auto & candidate : result.candidates) {
    if (candidate.acceptable) {++accepted;}
  }
  EXPECT_GE(accepted, 2u);
}

TEST(GlobalMatcher, ColdPositionsAcrossYawWrapUseFullMapBounds)
{
  auto options = settings();
  options.use_search_bounds = false;
  GlobalMatcher matcher(room(), options);
  for (const auto & expected : {pose(-2.1f, 2.2f, 179.f * kPi / 180.f),
      pose(3.2f, -2.f, -179.f * kPi / 180.f), pose(0.f, .2f, -75.f * kPi / 180.f)})
  {
    const auto result = matcher.search(independentScan(expected));
    ASSERT_EQ(result.status, GlobalMatchStatus::SUCCESS) << result.reason;
    const auto & actual = result.candidates.front().map_T_scan;
    EXPECT_LT((actual.block<3, 1>(0, 3) - expected.block<3, 1>(0, 3)).norm(), .2);
    const Eigen::Matrix3f relative = expected.block<3, 3>(0, 0).transpose() * actual.block<3, 3>(0, 0);
    // Across this wider set of viewpoints the default 0.8 m NDT cells can
    // introduce a few degrees of residual yaw; live confirmation still follows.
    EXPECT_LT(std::abs(std::atan2(relative(1, 0), relative(0, 0))), 5.f * kPi / 180.f);
    EXPECT_GT(result.candidates.front().quality.coverage, .9);
  }
}

TEST(GlobalMatcher, RejectsUnrelatedGeometryAndInvalidPoints)
{
  GlobalMatcher matcher(room(), settings());
  auto scan = GlobalCloud::Ptr(new GlobalCloud);
  // A circular surface differs from the map's rectangular vertical structures.
  for (int i = 0; i < 100; ++i) {
    for (int j = 0; j < 10; ++j) {
      GlobalPoint point;
      point.x = 7.0 * std::cos(i * 2.0 * kPi / 100);
      point.y = 7.0 * std::sin(i * 2.0 * kPi / 100);
      point.z = -0.5 + j * 0.17;
      scan->push_back(point);
    }
  }
  GlobalPoint invalid;
  invalid.x = std::numeric_limits<float>::quiet_NaN();
  invalid.y = invalid.z = 0;
  scan->push_back(invalid);
  const auto result = matcher.search(scan);
  EXPECT_EQ(result.status, GlobalMatchStatus::NO_MATCH) << result.reason;
  EXPECT_FALSE(matcher.evaluate(scan, Eigen::Matrix4f::Identity()).acceptable);
}

TEST(GlobalMatcher, PlausibleModesOutsideInitialOptimizerCountAreRefined)
{
  auto map = room();
  GlobalCloud duplicate;
  pcl::transformPointCloud(*map, duplicate, pose(14, 0, 0));
  *map += duplicate;
  auto options = settings();
  options.max_x = 20;
  options.retained_modes = 64;
  options.ndt_candidates = 2;
  // Require resolution of more than the initial two modes, under one budget.
  options.unresolved_score_gap = .8;
  GlobalMatcher matcher(map, options);
  const auto result = matcher.search(independentScan(pose(2.1f, -1.2f, -1.07f)));
  EXPECT_GT(result.candidates.size(), 2u);
  EXPECT_EQ(result.status, GlobalMatchStatus::AMBIGUOUS) << result.reason;
}

TEST(GlobalMatcher, CancellationAndBudgetsNeverReturnPartialSuccess)
{
  auto options = settings();
  GlobalMatcher matcher(room(), options);
  const auto scan = independentScan(pose(1, -1, 0.8));
  const auto immediately_cancelled = matcher.search(scan, []() {return true;});
  EXPECT_EQ(immediately_cancelled.status, GlobalMatchStatus::CANCELLED);
  int checks = 0;
  const auto cancelled_during_search = matcher.search(scan, [&checks]() {return ++checks > 8;});
  EXPECT_EQ(cancelled_during_search.status,
    GlobalMatchStatus::CANCELLED);
  options.max_search_evaluations = 100;
  GlobalMatcher limited(room(), options);
  EXPECT_EQ(limited.search(scan).status, GlobalMatchStatus::SEARCH_LIMIT);
  options.time_budget_seconds = 0.000001;
  GlobalMatcher timed(room(), options);
  EXPECT_EQ(timed.search(scan).status, GlobalMatchStatus::TIMEOUT);
}

TEST(GlobalMatcher, InsufficientScanAndInvalidConfigurationFail)
{
  auto options = settings();
  GlobalMatcher matcher(room(), options);
  EXPECT_EQ(matcher.search(GlobalCloud::Ptr(new GlobalCloud)).status,
    GlobalMatchStatus::INSUFFICIENT_SCAN);
  EXPECT_THROW(GlobalMatcher(GlobalCloud::Ptr(new GlobalCloud), options), std::invalid_argument);
  options.grid_resolution = 0;
  EXPECT_THROW(GlobalMatcher(room(), options), std::invalid_argument);
  options = settings();
  options.max_grid_cells = 100;
  EXPECT_THROW(GlobalMatcher(room(), options), std::invalid_argument);
  options = settings();
  options.min_x = 100;
  options.max_x = 110;
  options.min_y = 100;
  options.max_y = 110;
  EXPECT_THROW(GlobalMatcher(room(), options), std::invalid_argument);
}

TEST(GlobalMatcher, GroundPlaneHeightAndGravityLevelAreRespected)
{
  auto options = settings();
  options.use_ground_plane = true;
  options.level_from_ground_plane = true;
  options.ground_a = 0.045;
  options.ground_b = -0.035;
  options.ground_c = -1.2;
  options.sensor_height = 1.6;
  const Eigen::Vector3f normal(-options.ground_a, -options.ground_b, 1);
  const Eigen::Matrix3f map_R_level = Eigen::Quaternionf::FromTwoVectors(
    Eigen::Vector3f::UnitZ(), normal.normalized()).toRotationMatrix();
  Eigen::Matrix4f map_T_level = Eigen::Matrix4f::Identity();
  map_T_level.block<3, 3>(0, 0) = map_R_level;
  const float height = options.ground_c / normal.norm() + options.sensor_height;
  map_T_level.block<3, 1>(0, 3) = map_R_level * Eigen::Vector3f(0, 0, height);
  auto map = GlobalCloud::Ptr(new GlobalCloud);
  pcl::transformPointCloud(*room(), *map, map_T_level);
  const Eigen::Matrix4f level_T_scan = pose(2.1, -1.2, 0.73);
  const Eigen::Matrix4f expected = map_T_level * level_T_scan;
  GlobalMatcher matcher(map, options);
  const auto result = matcher.search(independentScan(level_T_scan));
  ASSERT_EQ(result.status, GlobalMatchStatus::SUCCESS) << result.reason;
  EXPECT_LT((result.candidates.front().map_T_scan.block<3, 1>(0, 3) -
    expected.block<3, 1>(0, 3)).norm(), 0.2);
  EXPECT_TRUE(matcher.evaluate(independentScan(level_T_scan), expected).acceptable);
}
}  // namespace
