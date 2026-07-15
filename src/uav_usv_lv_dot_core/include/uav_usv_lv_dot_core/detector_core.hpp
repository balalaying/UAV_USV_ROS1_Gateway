#ifndef UAV_USV_LV_DOT_CORE__DETECTOR_CORE_HPP_
#define UAV_USV_LV_DOT_CORE__DETECTOR_CORE_HPP_

#include <cstdint>

#include "uav_usv_lv_dot_core/types.hpp"

namespace uav_usv_lv_dot_core
{

struct CoreConfiguration
{
  // Algorithm parameters are intentionally deferred until Phase 2.
};

class DetectorCore
{
public:
  DetectorCore() = default;

  void configure(const CoreConfiguration & configuration);
  void reset();
  DetectionResult process(const PointCloudFrame & frame);

  bool configured() const noexcept;
  std::uint64_t processed_frames() const noexcept;

private:
  bool configured_{false};
  std::uint64_t processed_frames_{0};
};

}  // namespace uav_usv_lv_dot_core

#endif  // UAV_USV_LV_DOT_CORE__DETECTOR_CORE_HPP_
