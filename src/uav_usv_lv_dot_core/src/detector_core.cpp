#include "uav_usv_lv_dot_core/detector_core.hpp"

#include <stdexcept>

namespace uav_usv_lv_dot_core
{

void DetectorCore::configure(const CoreConfiguration &)
{
  configured_ = true;
  processed_frames_ = 0;
}

void DetectorCore::reset()
{
  configured_ = false;
  processed_frames_ = 0;
}

DetectionResult DetectorCore::process(const PointCloudFrame & frame)
{
  if (!configured_) {
    throw std::logic_error("DetectorCore must be configured before processing frames");
  }

  ++processed_frames_;
  DetectionResult result;
  result.stamp_nanoseconds = frame.context.stamp_nanoseconds;
  result.output_frame = frame.context.output_frame;
  return result;
}

bool DetectorCore::configured() const noexcept
{
  return configured_;
}

std::uint64_t DetectorCore::processed_frames() const noexcept
{
  return processed_frames_;
}

}  // namespace uav_usv_lv_dot_core
