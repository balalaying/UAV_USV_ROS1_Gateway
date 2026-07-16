#ifndef UAV_USV_LV_DOT_CORE__DETECTOR_CORE_HPP_
#define UAV_USV_LV_DOT_CORE__DETECTOR_CORE_HPP_

#include <array>
#include <cstdint>
#include <vector>

#include "uav_usv_lv_dot_core/dynamic_classifier.hpp"
#include "uav_usv_lv_dot_core/multi_object_tracker.hpp"
#include "uav_usv_lv_dot_core/types.hpp"

namespace uav_usv_lv_dot_core {

struct CoreConfiguration {
  // Repeats the accepted Mid-360 input contract so direct raw-cloud use remains
  // safe.
  double input_min_range{0.5};
  double input_max_range{20.0};
  double input_min_z{-1.75};
  double input_max_z{4.0};
  double input_voxel_size{0.04};
  bool crop_self{true};
  std::array<double, 6> self_bounds{{-4.3, 2.5, -1.8, 1.8, -2.4, 0.35}};

  // Native LV-DOT LiDAR preprocessing parameters from the accepted tuning
  // report.
  double local_range_x{10.0};
  double local_range_y{10.0};
  double ground_height{0.22};
  double roof_height{6.0};
  std::uint64_t downsample_threshold{12000};
  double adaptive_voxel_initial_size{0.1};
  double gaussian_downsample_sigma{16.0};
  std::uint32_t random_seed{1};

  // Upstream DBSCAN compares squared Euclidean distance directly with epsilon.
  double dbscan_epsilon_squared{0.65};
  std::uint32_t dbscan_min_points{3};
  std::array<double, 3> maximum_object_size{{30.0, 15.0, 12.0}};

  TrackingConfiguration tracking;
  DynamicClassificationConfiguration dynamic_classification;
};

class DetectorCore {
public:
  DetectorCore() = default;

  void configure(const CoreConfiguration &configuration);
  void reset();
  DetectionResult process(const PointCloudFrame &frame);

  bool configured() const noexcept;
  std::uint64_t processed_frames() const noexcept;

private:
  std::vector<PointXYZI> preprocess(const PointCloudFrame &frame) const;

  CoreConfiguration configuration_;
  MultiObjectTracker tracker_;
  DynamicClassifier dynamic_classifier_;
  bool configured_{false};
  std::uint64_t processed_frames_{0};
};

} // namespace uav_usv_lv_dot_core

#endif // UAV_USV_LV_DOT_CORE__DETECTOR_CORE_HPP_
