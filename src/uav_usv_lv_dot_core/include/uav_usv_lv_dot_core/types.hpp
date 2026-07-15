#ifndef UAV_USV_LV_DOT_CORE__TYPES_HPP_
#define UAV_USV_LV_DOT_CORE__TYPES_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace uav_usv_lv_dot_core
{

enum class ObservationSource : std::uint8_t
{
  kUnknown = 0,
  kLidar = 1,
  kCamera = 2,
  kAis = 4,
  kFused = 8,
};

enum class ObjectClass : std::uint8_t
{
  kUnknown = 0,
  kVessel = 1,
  kBuoy = 2,
  kDebris = 3,
  kLandmark = 4,
};

struct PointXYZI
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
  bool has_intensity{false};
};

struct RigidTransform
{
  std::array<double, 3> translation{{0.0, 0.0, 0.0}};
  std::array<double, 4> rotation_xyzw{{0.0, 0.0, 0.0, 1.0}};
};

struct FrameContext
{
  std::int64_t stamp_nanoseconds{0};
  double dt_seconds{0.0};
  std::string sensor_frame;
  std::string output_frame;
  RigidTransform sensor_to_output;
};

struct PointCloudFrame
{
  FrameContext context;
  std::vector<PointXYZI> points;
  bool is_dense{false};
};

struct TrackEstimate
{
  std::array<std::uint8_t, 16> uuid{};
  std::string track_id;
  std::int64_t first_seen_nanoseconds{0};
  std::int64_t last_update_nanoseconds{0};
  ObservationSource source{ObservationSource::kUnknown};
  ObjectClass classification{ObjectClass::kUnknown};
  std::array<double, 3> position{{0.0, 0.0, 0.0}};
  std::array<double, 4> orientation_xyzw{{0.0, 0.0, 0.0, 1.0}};
  std::array<double, 36> pose_covariance{};
  std::array<double, 3> linear_velocity{{0.0, 0.0, 0.0}};
  std::array<double, 3> angular_velocity{{0.0, 0.0, 0.0}};
  std::array<double, 36> twist_covariance{};
  std::array<double, 3> dimensions{{0.0, 0.0, 0.0}};
  float confidence{0.0F};
  std::uint32_t mmsi{0};
};

struct DetectionResult
{
  std::int64_t stamp_nanoseconds{0};
  std::string output_frame;
  std::vector<TrackEstimate> tracks;
};

}  // namespace uav_usv_lv_dot_core

#endif  // UAV_USV_LV_DOT_CORE__TYPES_HPP_
