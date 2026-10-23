#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iterator>
#include <Eigen/Geometry>

namespace g1_relocalization
{

// All timestamps belong to the ROS sensor clock. Callers provide locking.
class PoseBuffer
{
public:
  bool add(int64_t stamp, const Eigen::Matrix4f & pose)
  {
    if (stamp <= 0 || !pose.allFinite() ||
      (!poses_.empty() && stamp <= poses_.back().stamp))
    {
      return false;
    }
    poses_.push_back({stamp, pose});
    while (poses_.size() > 400) {poses_.pop_front();}
    return true;
  }

  bool lookup(int64_t stamp, double max_gap_sec, Eigen::Matrix4f & result) const
  {
    if (poses_.empty() || stamp <= 0 || !std::isfinite(max_gap_sec) || max_gap_sec <= 0) {
      return false;
    }
    const int64_t gap = static_cast<int64_t>(max_gap_sec * 1e9);
    auto upper = std::lower_bound(poses_.begin(), poses_.end(), stamp,
      [](const Entry & entry, int64_t value) {return entry.stamp < value;});
    if (upper != poses_.end() && upper->stamp == stamp) {
      result = upper->pose;
      return true;
    }
    if (upper == poses_.begin()) {
      if (upper->stamp - stamp > gap) {return false;}
      result = upper->pose;
      return true;
    }
    if (upper == poses_.end()) {
      if (stamp - poses_.back().stamp > gap) {return false;}
      result = poses_.back().pose;
      return true;
    }
    const auto lower = std::prev(upper);
    if (stamp - lower->stamp > gap || upper->stamp - stamp > gap) {return false;}
    const float fraction = static_cast<float>(stamp - lower->stamp) /
      static_cast<float>(upper->stamp - lower->stamp);
    Eigen::Quaternionf first(lower->pose.block<3, 3>(0, 0));
    Eigen::Quaternionf second(upper->pose.block<3, 3>(0, 0));
    result = Eigen::Matrix4f::Identity();
    result.block<3, 3>(0, 0) = first.normalized().slerp(fraction, second.normalized()).toRotationMatrix();
    result.block<3, 1>(0, 3) = (1.f - fraction) * lower->pose.block<3, 1>(0, 3) +
      fraction * upper->pose.block<3, 1>(0, 3);
    return result.allFinite();
  }

  void clear() {poses_.clear();}

private:
  struct Entry {int64_t stamp; Eigen::Matrix4f pose;};
  std::deque<Entry> poses_;
};

inline bool freshStamp(int64_t stamp, int64_t now, double max_age_sec)
{
  const double age = static_cast<double>(now - stamp) / 1e9;
  return stamp > 0 && now > 0 && std::isfinite(max_age_sec) && max_age_sec > 0 &&
    age >= 0.0 && age <= max_age_sec;
}

}  // namespace g1_relocalization
