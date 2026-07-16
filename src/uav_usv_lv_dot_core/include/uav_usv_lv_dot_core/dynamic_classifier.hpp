#ifndef UAV_USV_LV_DOT_CORE__DYNAMIC_CLASSIFIER_HPP_
#define UAV_USV_LV_DOT_CORE__DYNAMIC_CLASSIFIER_HPP_

#include <cstdint>
#include <memory>
#include <vector>

#include "uav_usv_lv_dot_core/types.hpp"

namespace uav_usv_lv_dot_core {

struct DynamicClassificationConfiguration {
  std::uint32_t frame_skip{2};
  double velocity_threshold{0.05};
  double voting_threshold{0.15};
  std::uint32_t force_dynamic_frames{3};
  std::uint32_t force_dynamic_check_range{12};
  std::uint32_t consistency_threshold{2};
  std::uint32_t history_size{100};
};

struct DynamicClassificationResult {
  std::vector<DynamicTrackEstimate> classified_tracks;
  std::vector<TrackEstimate> dynamic_tracks;
  DynamicClassificationStatistics statistics;
};

class DynamicClassifier {
public:
  DynamicClassifier();
  ~DynamicClassifier();

  DynamicClassifier(const DynamicClassifier &) = delete;
  DynamicClassifier &operator=(const DynamicClassifier &) = delete;
  DynamicClassifier(DynamicClassifier &&) noexcept;
  DynamicClassifier &operator=(DynamicClassifier &&) noexcept;

  void configure(const DynamicClassificationConfiguration &configuration);
  void reset();
  DynamicClassificationResult classify(const std::vector<TrackEstimate> &tracks,
                                       std::int64_t stamp_nanoseconds);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace uav_usv_lv_dot_core

#endif // UAV_USV_LV_DOT_CORE__DYNAMIC_CLASSIFIER_HPP_
