#include "g1_relocalization/global_matcher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ndt.h>

namespace g1_relocalization
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
double radians(double degrees) {return degrees * kPi / 180.0;}
double angleDistance(double a, double b)
{
  return std::abs(std::remainder(a - b, 2.0 * kPi));
}
bool finitePoint(const GlobalPoint & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
GlobalCloud::Ptr voxelized(GlobalCloud::ConstPtr input, double leaf)
{
  auto finite = GlobalCloud::Ptr(new GlobalCloud);
  for (const auto & point : *input) {
    if (finitePoint(point)) {finite->push_back(point);}
  }
  auto output = GlobalCloud::Ptr(new GlobalCloud);
  pcl::VoxelGrid<GlobalPoint> filter;
  filter.setInputCloud(finite);
  filter.setLeafSize(leaf, leaf, leaf);
  filter.filter(*output);
  return output;
}
struct PoseScore
{
  double x, y, yaw, score;
};
bool higherScore(const PoseScore & a, const PoseScore & b)
{
  if (a.score != b.score) {return a.score > b.score;}
  if (a.x != b.x) {return a.x < b.x;}
  if (a.y != b.y) {return a.y < b.y;}
  return a.yaw < b.yaw;
}
struct SearchInterrupted
{
  GlobalMatchStatus status;
};

struct Grid
{
  double resolution, min_x, min_y, min_z, layer_height;
  int width = 0, height = 0, layers = 0;
  std::vector<float> flat;
  std::vector<float> layered;

  float lookup(double x, double y, double z) const
  {
    const int ix = static_cast<int>(std::floor((x - min_x) / resolution));
    const int iy = static_cast<int>(std::floor((y - min_y) / resolution));
    if (ix < 0 || iy < 0 || ix >= width || iy >= height) {return 0.0f;}
    const std::size_t cell = static_cast<std::size_t>(iy) * width + ix;
    const int iz = static_cast<int>(std::floor((z - min_z) / layer_height));
    float vertical = 0.0f;
    // Adjacent height bands tolerate voxel boundaries and modest map drift.
    for (int dz = -1; dz <= 1; ++dz) {
      const int layer = iz + dz;
      if (layer >= 0 && layer < layers) {
        vertical = std::max(vertical, layered[
            static_cast<std::size_t>(layer) * width * height + cell]);
      }
    }
    return 0.75f * flat[cell] + 0.25f * vertical;
  }
};
}  // namespace

struct GlobalMatcher::Impl
{
  GlobalMatcherOptions options;
  GlobalCloud::Ptr map;
  pcl::KdTreeFLANN<GlobalPoint> tree;
  Eigen::Matrix3f map_R_level = Eigen::Matrix3f::Identity();
  std::vector<Grid> grids;
  double min_x, max_x, min_y, max_y;

  explicit Impl(GlobalCloud::ConstPtr input, const GlobalMatcherOptions & configured)
  : options(configured)
  {
    validateOptions();
    if (!input || input->empty()) {throw std::invalid_argument("Global map is empty");}
    map = voxelized(input, options.map_voxel_size);
    if (map->size() < 10) {throw std::invalid_argument("Global map has too few finite points");}
    tree.setInputCloud(map);
    if (options.level_from_ground_plane) {
      const Eigen::Vector3f normal(-options.ground_a, -options.ground_b, 1.0f);
      map_R_level = Eigen::Quaternionf::FromTwoVectors(
        Eigen::Vector3f::UnitZ(), normal.normalized()).toRotationMatrix();
    } else {
      map_R_level = (Eigen::AngleAxisf(radians(options.map_pitch_deg),
          Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(radians(options.map_roll_deg),
          Eigen::Vector3f::UnitX())).toRotationMatrix();
    }
    prepareGrids();
  }

  void validateOptions() const
  {
    const auto positive = [](double value) {return std::isfinite(value) && value > 0.0;};
    if (!positive(options.map_voxel_size) || !positive(options.scan_voxel_size) ||
      !positive(options.grid_resolution) || !positive(options.coarse_xy_step) ||
      !positive(options.coarse_yaw_step_deg) || options.coarse_yaw_step_deg > 90.0 ||
      !positive(options.fine_xy_step) || options.fine_xy_step > options.coarse_xy_step ||
      !positive(options.fine_yaw_step_deg) ||
      options.fine_yaw_step_deg > options.coarse_yaw_step_deg ||
      !positive(options.time_budget_seconds) || !positive(options.ndt_resolution) ||
      !positive(options.ndt_step_size) || !positive(options.ndt_transformation_epsilon) ||
      !positive(options.coverage_distance) || !positive(options.max_inlier_rmse) ||
      !positive(options.max_fitness) || !positive(options.mode_distance) ||
      !positive(options.mode_yaw_deg) || !positive(options.max_refinement_translation) ||
      !positive(options.max_refinement_yaw_deg) || !positive(options.max_refinement_tilt_deg) ||
      options.coarse_sample_count < 10 || options.fine_sample_count < 10 ||
      options.retained_modes < 2 || options.ndt_candidates < 2 ||
      options.ndt_candidates > options.retained_modes || options.min_scan_points < 10 ||
      options.ndt_max_iterations < 1 || options.max_grid_cells < 100 ||
      options.max_search_evaluations < 100 ||
      !std::isfinite(options.min_scan_z) || !std::isfinite(options.max_scan_z) ||
      options.max_scan_z <= options.min_scan_z ||
      !std::isfinite(options.min_scan_range) || options.min_scan_range < 0.0 ||
      !positive(options.max_scan_range) || options.max_scan_range <= options.min_scan_range ||
      !std::isfinite(options.initial_z) || !std::isfinite(options.ground_a) ||
      !std::isfinite(options.ground_b) || !std::isfinite(options.ground_c) ||
      !std::isfinite(options.sensor_height) || !std::isfinite(options.map_roll_deg) ||
      !std::isfinite(options.map_pitch_deg) ||
      !std::isfinite(options.min_coverage) || options.min_coverage <= 0.0 ||
      options.min_coverage > 1.0 || !std::isfinite(options.min_correlative_score) ||
      options.min_correlative_score < 0.0 || options.min_correlative_score > 1.0 ||
      !std::isfinite(options.ambiguity_score_gap) || options.ambiguity_score_gap <= 0.0 ||
      !std::isfinite(options.unresolved_score_gap) || options.unresolved_score_gap < 0.0)
    {
      throw std::invalid_argument("Invalid global matcher options");
    }
    if (options.use_search_bounds &&
      (!std::isfinite(options.min_x) || !std::isfinite(options.max_x) ||
      !std::isfinite(options.min_y) || !std::isfinite(options.max_y) ||
      options.max_x <= options.min_x || options.max_y <= options.min_y))
    {
      throw std::invalid_argument("Invalid global search bounds");
    }
  }

  double originHeight(double x, double y) const
  {
    if (!options.use_ground_plane) {return options.initial_z;}
    // Intersect the vertical level-frame ray with z=a*x+b*y+c in the map.
    const Eigen::Vector3f normal(-options.ground_a, -options.ground_b, 1.0f);
    const Eigen::Vector3f level_normal = map_R_level.transpose() * normal;
    if (std::abs(level_normal.z()) < 0.25f) {
      throw std::invalid_argument("Ground plane incompatible with gravity frame");
    }
    return (options.ground_c - level_normal.x() * x - level_normal.y() * y) /
           level_normal.z() + options.sensor_height;
  }

  void prepareGrids()
  {
    GlobalCloud features;
    for (const auto & point : *map) {
      const Eigen::Vector3f position = map_R_level.transpose() * point.getVector3fMap();
      const double relative_z = position.z() - originHeight(position.x(), position.y());
      if (relative_z < options.min_scan_z - 0.3 || relative_z > options.max_scan_z + 0.3) {
        continue;
      }
      GlobalPoint feature = point;
      feature.getVector3fMap() = position;
      features.push_back(feature);
    }
    if (features.size() < 10) {
      throw std::invalid_argument("No structural map points within configured height band");
    }
    double feature_min_x = std::numeric_limits<double>::infinity();
    double feature_max_x = -feature_min_x;
    double feature_min_y = feature_min_x, feature_max_y = -feature_min_x;
    double feature_min_z = feature_min_x, feature_max_z = -feature_min_x;
    for (const auto & point : features) {
      feature_min_x = std::min<double>(feature_min_x, point.x);
      feature_max_x = std::max<double>(feature_max_x, point.x);
      feature_min_y = std::min<double>(feature_min_y, point.y);
      feature_max_y = std::max<double>(feature_max_y, point.y);
      feature_min_z = std::min<double>(feature_min_z, point.z);
      feature_max_z = std::max<double>(feature_max_z, point.z);
    }
    min_x = options.use_search_bounds ? options.min_x : feature_min_x;
    max_x = options.use_search_bounds ? options.max_x : feature_max_x;
    min_y = options.use_search_bounds ? options.min_y : feature_min_y;
    max_y = options.use_search_bounds ? options.max_y : feature_max_y;
    // Keep all features observable from the bounds, including distant walls.
    feature_min_x = std::max(feature_min_x - 3.0, min_x - options.max_scan_range - 3.0);
    feature_max_x = std::min(feature_max_x + 3.0, max_x + options.max_scan_range + 3.0);
    feature_min_y = std::max(feature_min_y - 3.0, min_y - options.max_scan_range - 3.0);
    feature_max_y = std::min(feature_max_y + 3.0, max_y + options.max_scan_range + 3.0);
    Grid prototype;
    prototype.resolution = options.grid_resolution;
    prototype.min_x = std::floor(feature_min_x / prototype.resolution) * prototype.resolution;
    prototype.min_y = std::floor(feature_min_y / prototype.resolution) * prototype.resolution;
    prototype.min_z = feature_min_z - 0.75;
    prototype.layer_height = 0.75;
    const double width = std::ceil((feature_max_x - prototype.min_x) / prototype.resolution) + 1;
    const double height = std::ceil((feature_max_y - prototype.min_y) / prototype.resolution) + 1;
    const double layers = std::ceil((feature_max_z - prototype.min_z) / prototype.layer_height) + 2;
    if (!(width >= 1.0 && height >= 1.0 && layers >= 1.0)) {
      throw std::invalid_argument("Search bounds contain no observable map features");
    }
    if (!std::isfinite(width * height * (layers + 1.0)) ||
      width * height * (layers + 1.0) > options.max_grid_cells || layers > 64.0)
    {
      throw std::invalid_argument("Map distance field exceeds max_grid_cells; set valid bounds/height");
    }
    prototype.width = static_cast<int>(width);
    prototype.height = static_cast<int>(height);
    prototype.layers = static_cast<int>(layers);
    const std::size_t cells = static_cast<std::size_t>(prototype.width) * prototype.height;
    std::vector<std::uint8_t> occupied(cells * prototype.layers, 0);
    for (const auto & point : features) {
      const int ix = std::floor((point.x - prototype.min_x) / prototype.resolution);
      const int iy = std::floor((point.y - prototype.min_y) / prototype.resolution);
      const int iz = std::floor((point.z - prototype.min_z) / prototype.layer_height);
      if (ix >= 0 && iy >= 0 && iz >= 0 && ix < prototype.width &&
        iy < prototype.height && iz < prototype.layers)
      {
        occupied[static_cast<std::size_t>(iz) * cells + iy * prototype.width + ix] = 1;
      }
    }
    const double sigmas[] = {
      std::max(0.8, options.coarse_xy_step),
      std::max(0.45, options.coarse_xy_step * 0.55),
      std::max(0.25, options.fine_xy_step)};
    for (const double sigma : sigmas) {
      Grid grid = prototype;
      grid.flat.assign(cells, 0.0f);
      grid.layered.assign(cells * grid.layers, 0.0f);
      const int radius = std::ceil(2.5 * sigma / grid.resolution);
      for (int iz = 0; iz < grid.layers; ++iz) {
        for (int iy = 0; iy < grid.height; ++iy) {
          for (int ix = 0; ix < grid.width; ++ix) {
            if (!occupied[static_cast<std::size_t>(iz) * cells + iy * grid.width + ix]) {
              continue;
            }
            for (int dy = -radius; dy <= radius; ++dy) {
              if (iy + dy < 0 || iy + dy >= grid.height) {continue;}
              for (int dx = -radius; dx <= radius; ++dx) {
                if (ix + dx < 0 || ix + dx >= grid.width) {continue;}
                const float value = std::exp(-0.5 * grid.resolution * grid.resolution *
                  (dx * dx + dy * dy) / (sigma * sigma));
                const std::size_t index = static_cast<std::size_t>(iy + dy) * grid.width + ix + dx;
                grid.flat[index] = std::max(grid.flat[index], value);
                auto & layer_cell = grid.layered[static_cast<std::size_t>(iz) * cells + index];
                layer_cell = std::max(layer_cell, value);
              }
            }
          }
        }
      }
      grids.push_back(std::move(grid));
    }
  }

  GlobalCloud::Ptr filterScan(GlobalCloud::ConstPtr scan) const
  {
    auto structural = GlobalCloud::Ptr(new GlobalCloud);
    if (!scan) {return structural;}
    for (const auto & point : *scan) {
      if (!finitePoint(point) || point.z < options.min_scan_z || point.z > options.max_scan_z) {
        continue;
      }
      const double range = std::hypot(point.x, point.y);
      if (range >= options.min_scan_range && range <= options.max_scan_range) {
        structural->push_back(point);
      }
    }
    return voxelized(structural, options.scan_voxel_size);
  }

  std::vector<Eigen::Vector3f> samples(const GlobalCloud & scan, int requested) const
  {
    std::vector<Eigen::Vector3f> output;
    const std::size_t count = std::min<std::size_t>(scan.size(), requested);
    output.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      output.push_back(scan[(i * scan.size()) / count].getVector3fMap());
    }
    return output;
  }

  double correlate(
    const std::vector<Eigen::Vector3f> & points, const Grid & grid,
    double x, double y, double yaw) const
  {
    const double cosine = std::cos(yaw), sine = std::sin(yaw), z = originHeight(x, y);
    double score = 0.0;
    for (const auto & point : points) {
      score += grid.lookup(x + cosine * point.x() - sine * point.y(),
        y + sine * point.x() + cosine * point.y(), z + point.z());
    }
    return score / points.size();
  }

  std::vector<PoseScore> selectModes(std::vector<PoseScore> scores) const
  {
    std::sort(scores.begin(), scores.end(), higherScore);
    std::vector<PoseScore> selected;
    for (const auto & score : scores) {
      bool distinct = true;
      for (const auto & prior : selected) {
        if (std::hypot(score.x - prior.x, score.y - prior.y) < options.mode_distance &&
          angleDistance(score.yaw, prior.yaw) < radians(options.mode_yaw_deg))
        {
          distinct = false;
          break;
        }
      }
      if (distinct) {selected.push_back(score);}
      if (selected.size() >= static_cast<std::size_t>(options.retained_modes)) {break;}
    }
    return selected;
  }

  Eigen::Matrix4f transform(const PoseScore & score) const
  {
    Eigen::Matrix4f output = Eigen::Matrix4f::Identity();
    output.block<3, 3>(0, 0) = map_R_level * Eigen::AngleAxisf(
      score.yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    output.block<3, 1>(0, 3) = map_R_level * Eigen::Vector3f(
      score.x, score.y, originHeight(score.x, score.y));
    return output;
  }

  MatchQuality evaluateFiltered(const GlobalCloud & scan, const Eigen::Matrix4f & pose) const
  {
    MatchQuality result;
    if (scan.empty() || !pose.allFinite()) {return result;}
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    std::size_t inliers = 0;
    double sum_squared = 0.0;
    const double maximum_squared = options.coverage_distance * options.coverage_distance;
    for (const auto & point : scan) {
      GlobalPoint transformed = point;
      transformed.getVector3fMap() = pose.block<3, 3>(0, 0) * point.getVector3fMap() +
        pose.block<3, 1>(0, 3);
      ++result.evaluated_points;
      if (tree.nearestKSearch(transformed, 1, indices, distances) > 0 &&
        distances[0] <= maximum_squared)
      {
        ++inliers;
        sum_squared += distances[0];
      }
    }
    result.coverage = static_cast<double>(inliers) / result.evaluated_points;
    if (inliers) {result.inlier_rmse = std::sqrt(sum_squared / inliers);}
    result.score = result.coverage * std::exp(-result.inlier_rmse / options.coverage_distance);
    result.acceptable = result.evaluated_points >= static_cast<std::size_t>(options.min_scan_points) &&
      result.coverage >= options.min_coverage && result.inlier_rmse <= options.max_inlier_rmse;
    return result;
  }

  bool sameMode(const Eigen::Matrix4f & a, const Eigen::Matrix4f & b) const
  {
    const Eigen::Matrix3f relative = a.block<3, 3>(0, 0).transpose() * b.block<3, 3>(0, 0);
    return (a.block<2, 1>(0, 3) - b.block<2, 1>(0, 3)).norm() < options.mode_distance &&
           std::abs(std::atan2(relative(1, 0), relative(0, 0))) < radians(options.mode_yaw_deg);
  }
};

GlobalMatcher::GlobalMatcher(GlobalCloud::ConstPtr cloud, const GlobalMatcherOptions & options)
: impl_(new Impl(cloud, options)) {}
GlobalMatcher::~GlobalMatcher() = default;
const GlobalMatcherOptions & GlobalMatcher::options() const {return impl_->options;}

MatchQuality GlobalMatcher::evaluate(GlobalCloud::ConstPtr scan, const Eigen::Matrix4f & pose) const
{
  return impl_->evaluateFiltered(*impl_->filterScan(scan), pose);
}

GlobalMatchResult GlobalMatcher::search(
  GlobalCloud::ConstPtr input, const std::function<bool()> & cancelled)
{
  GlobalMatchResult result;
  const auto started = std::chrono::steady_clock::now();
  const auto & options = impl_->options;
  auto elapsed = [&]() {
      return std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    };
  auto check = [&]() {
      if (cancelled && cancelled()) {throw SearchInterrupted{GlobalMatchStatus::CANCELLED};}
      if (elapsed() >= options.time_budget_seconds) {throw SearchInterrupted{GlobalMatchStatus::TIMEOUT};}
      if (result.evaluations >= options.max_search_evaluations) {
        throw SearchInterrupted{GlobalMatchStatus::SEARCH_LIMIT};
      }
    };
  try {
    check();
    auto scan = impl_->filterScan(input);
    check();
    if (scan->size() < static_cast<std::size_t>(options.min_scan_points)) {
      result.status = GlobalMatchStatus::INSUFFICIENT_SCAN;
      result.reason = "Too few finite structural scan points after range/height/voxel filtering";
      result.elapsed_seconds = elapsed();
      return result;
    }
    const auto coarse_samples = impl_->samples(*scan, options.coarse_sample_count);
    const auto fine_samples = impl_->samples(*scan, options.fine_sample_count);
    std::vector<PoseScore> scores;
    auto append = [&](double x, double y, double yaw, const Grid & grid,
        const std::vector<Eigen::Vector3f> & points) {
        if ((result.evaluations % 64) == 0) {check();}
        if (x < impl_->min_x || x > impl_->max_x || y < impl_->min_y || y > impl_->max_y) {
          return;
        }
        ++result.evaluations;
        scores.push_back({x, y, std::remainder(yaw, 2.0 * kPi),
          impl_->correlate(points, grid, x, y, yaw)});
      };
    const int yaw_count = std::ceil(360.0 / options.coarse_yaw_step_deg);
    for (int angle = 0; angle < yaw_count; ++angle) {
      const double yaw = -kPi + angle * 2.0 * kPi / yaw_count;
      for (double x = impl_->min_x; x <= impl_->max_x + 1e-9; x += options.coarse_xy_step) {
        for (double y = impl_->min_y; y <= impl_->max_y + 1e-9; y += options.coarse_xy_step) {
          append(x, y, yaw, impl_->grids[0], coarse_samples);
        }
      }
    }
    auto modes = impl_->selectModes(std::move(scores));
    // Refine every retained mode, preserving widely separated locations/yaws.
    const double xy_steps[] = {options.coarse_xy_step * 0.5, options.fine_xy_step};
    const double yaw_steps[] = {radians(options.coarse_yaw_step_deg) * 0.5,
      radians(options.fine_yaw_step_deg)};
    double xy_radius = options.coarse_xy_step, yaw_radius = radians(options.coarse_yaw_step_deg);
    for (int level = 0; level < 2; ++level) {
      scores.clear();
      const int xy_count = std::ceil(xy_radius / xy_steps[level]);
      const int angle_count = std::ceil(yaw_radius / yaw_steps[level]);
      for (const auto & mode : modes) {
        for (int da = -angle_count; da <= angle_count; ++da) {
          for (int dx = -xy_count; dx <= xy_count; ++dx) {
            for (int dy = -xy_count; dy <= xy_count; ++dy) {
              append(mode.x + dx * xy_steps[level], mode.y + dy * xy_steps[level],
                mode.yaw + da * yaw_steps[level], impl_->grids[level + 1], fine_samples);
            }
          }
        }
      }
      modes = impl_->selectModes(std::move(scores));
      xy_radius = xy_steps[level];
      yaw_radius = yaw_steps[level];
    }
    check();
    if (modes.empty() || modes.front().score < options.min_correlative_score) {
      result.reason = "Global distance-field match below minimum score";
      result.elapsed_seconds = elapsed();
      return result;
    }
    pcl::NormalDistributionsTransform<GlobalPoint, GlobalPoint> ndt;
    ndt.setResolution(options.ndt_resolution);
    ndt.setStepSize(options.ndt_step_size);
    ndt.setTransformationEpsilon(options.ndt_transformation_epsilon);
    ndt.setMaximumIterations(options.ndt_max_iterations);
    ndt.setInputTarget(impl_->map);
    ndt.setInputSource(scan);
    std::function<void(const GlobalCloud &, const std::vector<int> &,
      const GlobalCloud &, const std::vector<int> &)> ndt_progress =
      [&](const GlobalCloud &, const std::vector<int> &, const GlobalCloud &, const std::vector<int> &) {
        check();
      };
    ndt.registerVisualizationCallback(ndt_progress);
    // Start with the configured count, then refine any remaining mode that
    // could still challenge the best result. Never discard it merely because
    // it fell outside the first N optimizer calls. The original deadline holds.
    std::size_t refine_count = 0;
    for (std::size_t i = 0; i < modes.size(); ++i) {
      if (i >= static_cast<std::size_t>(options.ndt_candidates)) {
        const GlobalMatchCandidate * best = nullptr;
        for (const auto & prior : result.candidates) {
          if (prior.acceptable && (!best || prior.quality.score > best->quality.score)) {best = &prior;}
        }
        if (modes[i].score < options.min_correlative_score ||
          (best && modes[i].score < best->correlative_score - options.unresolved_score_gap)) {break;}
      }
      check();
      GlobalMatchCandidate candidate;
      candidate.correlative_score = modes[i].score;
      const Eigen::Matrix4f initial = impl_->transform(modes[i]);
      candidate.map_T_scan = initial;
      candidate.initial_map_T_scan = initial;
      if (modes[i].score >= options.min_correlative_score) {
        GlobalCloud aligned;
        ndt.align(aligned, initial);
        check();
        candidate.converged = ndt.hasConverged();
        candidate.map_T_scan = ndt.getFinalTransformation();
        candidate.fitness = ndt.getFitnessScore();
        candidate.quality = impl_->evaluateFiltered(*scan, candidate.map_T_scan);
        const Eigen::Matrix3f relative = initial.block<3, 3>(0, 0).transpose() *
          candidate.map_T_scan.block<3, 3>(0, 0);
        const double yaw_shift = std::abs(std::atan2(relative(1, 0), relative(0, 0)));
        const double tilt_shift = std::acos(std::max(-1.0f, std::min(1.0f, relative(2, 2))));
        candidate.acceptable = candidate.converged && candidate.map_T_scan.allFinite() &&
          std::isfinite(candidate.fitness) && candidate.fitness <= options.max_fitness &&
          candidate.quality.acceptable &&
          (candidate.map_T_scan.block<3, 1>(0, 3) - initial.block<3, 1>(0, 3)).norm() <=
          options.max_refinement_translation && yaw_shift <= radians(options.max_refinement_yaw_deg) &&
          tilt_shift <= radians(options.max_refinement_tilt_deg);
      }
      result.candidates.push_back(candidate);
      ++refine_count;
    }
    check();
    std::stable_sort(result.candidates.begin(), result.candidates.end(),
      [](const GlobalMatchCandidate & a, const GlobalMatchCandidate & b) {
        if (a.acceptable != b.acceptable) {return a.acceptable;}
        if (a.quality.score != b.quality.score) {return a.quality.score > b.quality.score;}
        return a.correlative_score > b.correlative_score;
      });
    if (result.candidates.empty() || !result.candidates.front().acceptable) {
      result.reason = "No NDT candidate passes independent 3-D coverage and refinement limits";
    } else {
      const auto & best = result.candidates.front();
      bool ambiguous = false;
      for (std::size_t i = 1; i < result.candidates.size(); ++i) {
        const auto & alternative = result.candidates[i];
        const Eigen::Matrix4f & alternative_pose = alternative.acceptable ?
          alternative.map_T_scan : alternative.initial_map_T_scan;
        if (impl_->sameMode(best.map_T_scan, alternative_pose)) {continue;}
        // A plausible geometric mode with a failed optimizer remains unresolved.
        const bool unresolved = !alternative.acceptable &&
          alternative.correlative_score >= best.correlative_score - options.unresolved_score_gap;
        const bool competing = alternative.acceptable &&
          best.quality.score - alternative.quality.score < options.ambiguity_score_gap;
        if (unresolved || competing) {ambiguous = true; break;}
      }
      for (std::size_t i = refine_count; i < modes.size(); ++i) {
        if (modes[i].score >= options.min_correlative_score &&
          modes[i].score >= best.correlative_score - options.unresolved_score_gap) {
          ambiguous = true;
          break;
        }
      }
      result.status = ambiguous ? GlobalMatchStatus::AMBIGUOUS : GlobalMatchStatus::SUCCESS;
      result.reason = ambiguous ? "Competing or unrefined plausible global modes remain" :
        "Unique global mode passes NDT and independent 3-D coverage; confirm with a new scan";
    }
  } catch (const SearchInterrupted & interrupted) {
    result.status = interrupted.status;
    result.reason = "Global search stopped; incomplete search cannot establish a unique pose";
  }
  result.elapsed_seconds = elapsed();
  return result;
}

const char * globalMatchStatusName(GlobalMatchStatus status)
{
  switch (status) {
    case GlobalMatchStatus::SUCCESS: return "success";
    case GlobalMatchStatus::AMBIGUOUS: return "ambiguous";
    case GlobalMatchStatus::NO_MATCH: return "no_match";
    case GlobalMatchStatus::INSUFFICIENT_SCAN: return "insufficient_scan";
    case GlobalMatchStatus::CANCELLED: return "cancelled";
    case GlobalMatchStatus::TIMEOUT: return "timeout";
    case GlobalMatchStatus::SEARCH_LIMIT: return "search_limit";
  }
  return "unknown";
}
}  // namespace g1_relocalization
