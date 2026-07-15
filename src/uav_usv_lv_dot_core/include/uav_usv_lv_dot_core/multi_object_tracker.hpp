#ifndef UAV_USV_LV_DOT_CORE__MULTI_OBJECT_TRACKER_HPP_
#define UAV_USV_LV_DOT_CORE__MULTI_OBJECT_TRACKER_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "uav_usv_lv_dot_core/kalman_filter.hpp"
#include "uav_usv_lv_dot_core/types.hpp"

namespace uav_usv_lv_dot_core {

struct TrackingConfiguration {
  double max_match_range{2.0};
  double max_size_difference{8.0};
  std::array<double, 9> feature_weights{
      {3.0, 3.0, 0.1, 0.5, 0.5, 0.05, 0.0, 0.0, 0.0}};
  std::uint32_t history_size{100};
  std::uint32_t fix_size_history_threshold{10};
  double fix_size_dimension_threshold{0.4};
  std::uint32_t kalman_averaging_frames{3};
  std::uint32_t confirmation_hits{3};
  std::uint32_t maximum_missed_frames{5};
  KalmanNoise kalman_noise;
};

struct TrackingResult {
  std::vector<TrackEstimate> tracks;
  TrackingStatistics statistics;
};

class MultiObjectTracker {
public:
  MultiObjectTracker();
  ~MultiObjectTracker();

  MultiObjectTracker(const MultiObjectTracker &) = delete;
  MultiObjectTracker &operator=(const MultiObjectTracker &) = delete;
  MultiObjectTracker(MultiObjectTracker &&) noexcept;
  MultiObjectTracker &operator=(MultiObjectTracker &&) noexcept;

  void configure(const TrackingConfiguration &configuration);
  void reset();
  TrackingResult update(const std::vector<LidarCluster> &detections,
                        std::int64_t stamp_nanoseconds,
                        const std::array<double, 3> &sensor_position);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace uav_usv_lv_dot_core

#endif // UAV_USV_LV_DOT_CORE__MULTI_OBJECT_TRACKER_HPP_
