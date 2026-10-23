#include "g1_relocalization/global_matcher.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

namespace
{
volatile std::sig_atomic_t interrupted = 0;
void onSignal(int) {interrupted = 1;}
constexpr double kPi = 3.14159265358979323846;

void usage()
{
  std::cout <<
    "Usage: global_matcher_check --map MAP.pcd [--scan SCAN.pcd] [options]\n"
    "\n"
    "Without --scan, only map loading and distance-field construction are checked.\n"
    "With --scan, run the same full XY/yaw search and NDT/coverage/ambiguity checks\n"
    "as automatic initialization. A unique candidate still requires confirmation\n"
    "using NEW live scans before navigation; this tool does not publish ROS topics.\n"
    "\n"
    "SCAN.pcd must be an independently recorded local observation centered on the\n"
    "reference body origin, with gravity pointing down the negative Z axis.\n"
    "Do not supply the map as the scan or use points already in map coordinates.\n"
    "A /cloud_registered_body PCD can be rotated using its matching Odometry\n"
    "orientation (camera_init <- body), without applying odometry translation:\n"
    "  --scan-rotation-wxyz W X Y Z\n"
    "Multiple scans must be deskewed/accumulated relative to one body origin first.\n"
    "\n"
    "Options:\n"
    "  --bounds MIN_X MAX_X MIN_Y MAX_Y   Restrict body XY search in levelled map\n"
    "                                     frame (default: all structural map bounds)\n"
    "  --timeout SECONDS                 Search budget (default: 60)\n"
    "  --ground-plane A B C HEIGHT       z=A*x+B*y+C; body height above floor\n"
    "                                     (default: -0.06203 -0.02687 -1.51 1.51)\n"
    "  --initial-z Z                     Fixed body Z; disables ground plane\n"
    "  --level-from-ground-plane         Derive gravity orientation from floor\n"
    "                                     (default: false, as in live node)\n"
    "  --map-roll-pitch-deg ROLL PITCH    Map tilt; default 0 0\n"
    "  --scan-rotation-wxyz W X Y Z       Rotate raw body scan into gravity axes\n"
    "                                     (default: identity)\n"
    "  --help                            Show this text\n"
    "\n"
    "Exit codes: 0=map index ready or unique scan candidate; 2=no unique match;\n"
    "3=invalid arguments/map/configuration; 130=interrupted.\n";
}

double number(const char * text)
{
  std::size_t end = 0;
  const std::string value(text);
  const double result = std::stod(value, &end);
  if (end != value.size() || !std::isfinite(result)) {
    throw std::invalid_argument("Expected a finite number: " + value);
  }
  return result;
}

g1_relocalization::GlobalCloud::Ptr loadCloud(const std::string & path)
{
  auto cloud = g1_relocalization::GlobalCloud::Ptr(new g1_relocalization::GlobalCloud);
  if (pcl::io::loadPCDFile(path, *cloud) < 0) {
    throw std::runtime_error("Cannot load PCD: " + path);
  }
  return cloud;
}
}  // namespace

int main(int argc, char ** argv)
{
  using namespace g1_relocalization;
  try {
    GlobalMatcherOptions options;
    // Keep these defaults in sync with the live node's installed-map defaults.
    options.use_ground_plane = true;
    options.time_budget_seconds = 60.0;
    std::string map_path, scan_path;
    Eigen::Matrix4f scan_rotation = Eigen::Matrix4f::Identity();
    for (int i = 1; i < argc; ++i) {
      const std::string key(argv[i]);
      auto require = [&](int count) {
          if (i + count >= argc) {throw std::invalid_argument("Missing value after " + key);}
        };
      if (key == "--help") {usage(); return 0;}
      if (key == "--map" || key == "--scan") {
        require(1);
        (key == "--map" ? map_path : scan_path) = argv[++i];
      } else if (key == "--bounds") {
        require(4); options.use_search_bounds = true;
        options.min_x = number(argv[++i]); options.max_x = number(argv[++i]);
        options.min_y = number(argv[++i]); options.max_y = number(argv[++i]);
      } else if (key == "--timeout") {
        require(1); options.time_budget_seconds = number(argv[++i]);
      } else if (key == "--ground-plane") {
        require(4); options.use_ground_plane = true;
        options.ground_a = number(argv[++i]); options.ground_b = number(argv[++i]);
        options.ground_c = number(argv[++i]); options.sensor_height = number(argv[++i]);
      } else if (key == "--initial-z") {
        require(1); options.initial_z = number(argv[++i]); options.use_ground_plane = false;
      } else if (key == "--level-from-ground-plane") {
        options.level_from_ground_plane = true;
      } else if (key == "--map-roll-pitch-deg") {
        require(2); options.map_roll_deg = number(argv[++i]); options.map_pitch_deg = number(argv[++i]);
      } else if (key == "--scan-rotation-wxyz") {
        require(4);
        const double w = number(argv[++i]), x = number(argv[++i]);
        const double y = number(argv[++i]), z = number(argv[++i]);
        Eigen::Quaternionf q(w, x, y, z);
        if (!q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-6f) {
          throw std::invalid_argument("Scan rotation must have a finite nonzero quaternion");
        }
        scan_rotation.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
      } else {
        throw std::invalid_argument("Unknown argument: " + key);
      }
    }
    if (map_path.empty()) {throw std::invalid_argument("--map is required (use --help)");}
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    const auto started = std::chrono::steady_clock::now();
    auto map = loadCloud(map_path);
    GlobalMatcher matcher(map, options);
    const double index_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();
    std::cout << std::setprecision(8) << "MAP_INDEX_READY input_points=" << map->size()
              << " elapsed_seconds=" << index_seconds << '\n';
    if (scan_path.empty()) {
      std::cout << "No independent scan supplied; localization has not been tested.\n";
      return interrupted ? 130 : 0;
    }
    auto raw_scan = loadCloud(scan_path);
    auto scan = GlobalCloud::Ptr(new GlobalCloud);
    pcl::transformPointCloud(*raw_scan, *scan, scan_rotation);
    const auto result = matcher.search(scan, []() {return interrupted != 0;});
    std::cout << "status=" << globalMatchStatusName(result.status)
              << " elapsed_seconds=" << result.elapsed_seconds
              << " evaluations=" << result.evaluations << "\nreason=" << result.reason << '\n';
    for (std::size_t i = 0; i < result.candidates.size(); ++i) {
      const auto & candidate = result.candidates[i];
      const auto & transform = candidate.map_T_scan;
      Eigen::Quaternionf q(transform.block<3, 3>(0, 0));
      std::cout << "candidate=" << i << " acceptable=" << candidate.acceptable
                << " converged=" << candidate.converged
                << " correlative_score=" << candidate.correlative_score
                << " quality_score=" << candidate.quality.score
                << " fitness=" << candidate.fitness << " coverage=" << candidate.quality.coverage
                << " inlier_rmse=" << candidate.quality.inlier_rmse
                << " evaluated_points=" << candidate.quality.evaluated_points
                << " xyz=" << transform(0, 3) << ',' << transform(1, 3) << ',' << transform(2, 3)
                << " yaw_deg=" << std::atan2(transform(1, 0), transform(0, 0)) * 180.0 / kPi
                << " quaternion_wxyz=" << q.w() << ',' << q.x() << ',' << q.y() << ',' << q.z() << '\n';
    }
    if (result.status == GlobalMatchStatus::SUCCESS) {
      std::cout << "Unique candidate found; live confirmation on new scans is still required.\n";
    }
    if (interrupted || result.status == GlobalMatchStatus::CANCELLED) {return 130;}
    return result.status == GlobalMatchStatus::SUCCESS ? 0 : 2;
  } catch (const std::exception & error) {
    std::cerr << "global_matcher_check: " << error.what() << '\n';
    return 3;
  }
}
