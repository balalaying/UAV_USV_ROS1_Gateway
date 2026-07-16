#ifndef UAV_USV_LV_DOT_CORE__TYPES_HPP_
#define UAV_USV_LV_DOT_CORE__TYPES_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace uav_usv_lv_dot_core {

enum class ObservationSource : std::uint8_t {
  kUnknown = 0,
  kLidar = 1,
  kCamera = 2,
  kAis = 4,
  kFused = 8,
};

enum class ObjectClass : std::uint8_t {
  kUnknown = 0,
  kVessel = 1,
  kBuoy = 2,
  kDebris = 3,
  kLandmark = 4,
};

enum class TrackLifecycle : std::uint8_t {
  kNew = 0,
  kConfirmed = 1,
  kLost = 2,
  kRemoved = 3,
};

enum class DynamicMotionState : std::uint8_t {
  kStatic = 0,
  kMovingCandidate = 1,
  kConfirmedDynamic = 2,
};

struct PointXYZI {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
  bool has_intensity{false};
};

struct RigidTransform {
  std::array<double, 3> translation{{0.0, 0.0, 0.0}};
  std::array<double, 4> rotation_xyzw{{0.0, 0.0, 0.0, 1.0}};
};

struct FrameContext {
  std::int64_t stamp_nanoseconds{0};
  double dt_seconds{0.0};
  std::string sensor_frame;
  std::string output_frame;
  RigidTransform sensor_to_output;
};

struct PointCloudFrame {
  FrameContext context;
  std::vector<PointXYZI> points;
  bool is_dense{false};
};

struct TrackEstimate {
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
  std::array<double, 3> linear_acceleration{{0.0, 0.0, 0.0}};
  float confidence{0.0F};
  std::uint32_t mmsi{0};
  TrackLifecycle lifecycle{TrackLifecycle::kNew};
  std::uint64_t age{0};
  std::uint32_t missed_count{0};
};

struct LidarCluster {
  std::int32_t cluster_id{-1};
  std::array<double, 3> center{{0.0, 0.0, 0.0}};
  std::array<double, 3> dimensions{{0.0, 0.0, 0.0}};
  std::uint64_t point_count{0};
  std::int64_t stamp_nanoseconds{0};
};

struct ClusteringStatistics {
  std::uint64_t input_point_count{0};
  std::uint64_t finite_point_count{0};
  std::uint64_t preprocessed_point_count{0};
  std::uint64_t clustered_point_count{0};
  std::uint64_t noise_point_count{0};
  double preprocessing_time_ms{0.0};
  double clustering_time_ms{0.0};
};

struct TrackingStatistics {
  std::uint64_t detection_count{0};
  std::uint64_t matched_count{0};
  std::uint64_t created_track_count{0};
  std::uint64_t removed_track_count{0};
  std::uint64_t active_track_count{0};
  std::uint64_t confirmed_track_count{0};
  std::uint64_t lost_track_count{0};
  std::uint64_t id_switch_count{0};
  double match_success_rate{0.0};
  double average_match_distance{0.0};
  double average_velocity{0.0};
  double kalman_update_time_ms{0.0};
};

struct MotionHistorySample {
  std::int64_t stamp_nanoseconds{0};
  std::array<double, 3> position{{0.0, 0.0, 0.0}};
  std::array<double, 3> velocity{{0.0, 0.0, 0.0}};
  double speed{0.0};
  double displacement_speed{0.0};
  double direction_similarity{0.0};
  double vote_ratio{0.0};
  bool is_candidate{false};
  bool is_dynamic{false};
};

struct DynamicTrackEstimate {
  std::string track_id;
  TrackEstimate track;
  double dynamic_probability{0.0};
  bool is_dynamic{false};
  DynamicMotionState motion_state{DynamicMotionState::kStatic};
  std::vector<MotionHistorySample> motion_history;
  float confidence{0.0F};
};

struct DynamicClassificationStatistics {
  std::uint64_t total_track_count{0};
  std::uint64_t static_track_count{0};
  std::uint64_t candidate_track_count{0};
  std::uint64_t confirmed_dynamic_count{0};
  std::uint64_t unclassified_track_count{0};
  double dynamic_ratio{0.0};
  double average_velocity{0.0};
  double classification_time_ms{0.0};
};

struct DetectionResult {
  std::int64_t stamp_nanoseconds{0};
  std::string output_frame;
  std::vector<LidarCluster> lidar_clusters;
  ClusteringStatistics clustering_statistics;
  std::vector<TrackEstimate> tracks;
  TrackingStatistics tracking_statistics;
  std::vector<DynamicTrackEstimate> classified_tracks;
  std::vector<TrackEstimate> dynamic_tracks;
  DynamicClassificationStatistics dynamic_statistics;
};

} // namespace uav_usv_lv_dot_core

#endif // UAV_USV_LV_DOT_CORE__TYPES_HPP_
