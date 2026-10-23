#pragma once

#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace g1_relocalization
{

using GlobalPoint = pcl::PointXYZI;
using GlobalCloud = pcl::PointCloud<GlobalPoint>;

struct GlobalMatcherOptions
{
  // Explicit bounds are in the levelled map frame. Without them all finite map
  // features are searched; no percentile crop silently removes mapped areas.
  bool use_search_bounds = false;
  double min_x = -8.5;
  double max_x = 28.05;
  double min_y = -26.5;
  double max_y = 25.05;
  double map_roll_deg = 0.0;
  double map_pitch_deg = 0.0;
  bool level_from_ground_plane = false;
  bool use_ground_plane = false;
  double ground_a = -0.06203;
  double ground_b = -0.02687;
  double ground_c = -1.51;
  double sensor_height = 1.51;
  double initial_z = 0.0;

  double min_scan_z = -0.8;
  double max_scan_z = 2.0;
  double min_scan_range = 0.8;
  double max_scan_range = 20.0;
  double map_voxel_size = 0.25;
  double scan_voxel_size = 0.20;
  double grid_resolution = 0.25;
  double coarse_xy_step = 1.0;
  double coarse_yaw_step_deg = 10.0;
  double fine_xy_step = 0.25;
  double fine_yaw_step_deg = 2.0;
  int coarse_sample_count = 180;
  int fine_sample_count = 400;
  int retained_modes = 96;
  int ndt_candidates = 12;
  double mode_distance = 1.0;
  double mode_yaw_deg = 20.0;
  double min_correlative_score = 0.45;
  double ambiguity_score_gap = 0.08;
  double unresolved_score_gap = 0.025;
  double time_budget_seconds = 30.0;
  std::size_t max_grid_cells = 2000000;
  std::size_t max_search_evaluations = 3000000;
  int min_scan_points = 100;

  double ndt_resolution = 1.0;
  double ndt_step_size = 0.1;
  double ndt_transformation_epsilon = 0.01;
  int ndt_max_iterations = 40;
  double max_fitness = 0.6;
  double coverage_distance = 0.45;
  double min_coverage = 0.65;
  double max_inlier_rmse = 0.35;
  double max_refinement_translation = 2.0;
  double max_refinement_yaw_deg = 25.0;
  double max_refinement_tilt_deg = 15.0;
};

struct MatchQuality
{
  double coverage = 0.0;
  double inlier_rmse = std::numeric_limits<double>::infinity();
  double score = 0.0;
  std::size_t evaluated_points = 0;
  bool acceptable = false;
};

struct GlobalMatchCandidate
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix4f map_T_scan = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f initial_map_T_scan = Eigen::Matrix4f::Identity();
  double correlative_score = 0.0;
  double fitness = std::numeric_limits<double>::infinity();
  MatchQuality quality;
  bool converged = false;
  bool acceptable = false;
};

enum class GlobalMatchStatus
{
  SUCCESS,
  AMBIGUOUS,
  NO_MATCH,
  INSUFFICIENT_SCAN,
  CANCELLED,
  TIMEOUT,
  SEARCH_LIMIT
};

const char * globalMatchStatusName(GlobalMatchStatus status);

struct GlobalMatchResult
{
  GlobalMatchStatus status = GlobalMatchStatus::NO_MATCH;
  std::string reason;
  std::vector<GlobalMatchCandidate, Eigen::aligned_allocator<GlobalMatchCandidate>> candidates;
  double elapsed_seconds = 0.0;
  std::size_t evaluations = 0;
};

class GlobalMatcher
{
public:
  // map_cloud is copied and voxel filtered. A map-empty/invalid option/grid-too-
  // large error throws std::invalid_argument; it must fail before navigation.
  GlobalMatcher(GlobalCloud::ConstPtr map_cloud, const GlobalMatcherOptions & options);
  ~GlobalMatcher();
  GlobalMatcher(const GlobalMatcher &) = delete;
  GlobalMatcher & operator=(const GlobalMatcher &) = delete;

  // scan axes must have upward gravity and origin at the reference body. The
  // output maps these scan coordinates into the ORIGINAL map coordinates.
  // Cancellation is checked between scoring blocks and NDT iterations.
  GlobalMatchResult search(
    GlobalCloud::ConstPtr scan, const std::function<bool()> & cancelled = {});

  // Independent nearest-neighbour 3-D coverage on structural (non-floor) scan
  // points. Suitable for a NEW scan confirmation and subsequent tracking.
  MatchQuality evaluate(GlobalCloud::ConstPtr scan, const Eigen::Matrix4f & map_T_scan) const;
  const GlobalMatcherOptions & options() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace g1_relocalization
