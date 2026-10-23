#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <stdexcept>
#include <vector>

namespace g1_relocalization
{
// Every hypothesis receives one observation from the SAME fresh scan. A window
// is evidence for comparison, not a calibrated probability of the robot's pose.
class CandidateEvidence
{
public:
  explicit CandidateEvidence(std::size_t window) : window_(window)
  {
    if (window < 2) {throw std::invalid_argument("Candidate evidence requires at least two scans");}
  }

  void observe(bool accepted, double score, double coverage = 0., bool geometric_rejection = false)
  {
    const bool known = std::isfinite(score) && score >= 0. && score <= 1. &&
      std::isfinite(coverage) && coverage >= 0. && coverage <= 1.;
    // Optimizer failure is NOT evidence that a location is impossible. Preserve
    // its geometric support; unknown scores conservatively block selection.
    samples_.push_back({accepted && known, known ? score : 1., known ? coverage : 1.,
      geometric_rejection && known && !accepted});
    while (samples_.size() > window_) {samples_.pop_front();}
  }

  bool complete() const {return samples_.size() == window_;}
  bool stable() const
  {
    return complete() && std::all_of(samples_.begin(), samples_.end(),
      [](const Sample & sample) {return sample.accepted;});
  }
  bool rejected() const
  {
    return complete() && std::all_of(samples_.begin(), samples_.end(),
      [](const Sample & sample) {return sample.rejected;});
  }
  double score() const
  {
    double total = 0.;
    for (const auto & sample : samples_) {total += sample.score;}
    return samples_.empty() ? 0. : total / samples_.size();
  }
  std::size_t count() const {return samples_.size();}
  double coverage() const
  {
    double total = 0.;
    for (const auto & sample : samples_) {total += sample.coverage;}
    return samples_.empty() ? 0. : total / samples_.size();
  }

private:
  struct Sample {bool accepted; double score; double coverage; bool rejected;};
  std::size_t window_;
  std::deque<Sample> samples_;
};

// A stable primary candidate above direct_coverage is accepted without a
// uniqueness comparison. Otherwise, retain the score/coverage gap rules.
inline int verifiedWinner(
  const std::vector<CandidateEvidence> & evidence, double gap, bool omitted_candidates = false,
  double coverage_gap = .08, double direct_coverage = .91)
{
  if (!std::isfinite(gap) || gap <= 0. || !std::isfinite(coverage_gap) || coverage_gap <= 0. ||
    !std::isfinite(direct_coverage) || direct_coverage <= 0. || direct_coverage > 1.) {return -1;}
  if (!evidence.empty() && evidence.front().stable() &&
    evidence.front().coverage() > direct_coverage) {return 0;}
  if (omitted_candidates) {return -1;}
  int best = -1;
  for (std::size_t i = 0; i < evidence.size(); ++i) {
    if (evidence[i].stable() && (best < 0 || evidence[i].score() > evidence[best].score())) {
      best = static_cast<int>(i);
    }
  }
  if (best < 0) {return -1;}
  for (std::size_t i = 0; i < evidence.size(); ++i) {
    if (static_cast<int>(i) == best || evidence[i].rejected()) {continue;}
    // A different NDT local optimum can improve residuals in a duplicated
    // room without adding any location information. Require coverage evidence
    // as well as score separation while both places still pass geometry checks.
    if (!evidence[i].complete() || evidence[best].score() - evidence[i].score() < gap ||
      evidence[best].coverage() - evidence[i].coverage() < coverage_gap) {return -1;}
  }
  return best;
}
}  // namespace g1_relocalization
