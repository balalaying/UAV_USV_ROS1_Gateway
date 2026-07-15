#include "uav_usv_lv_dot_core/multi_object_tracker.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace uav_usv_lv_dot_core {
namespace {

struct HistorySample {
  std::int64_t stamp_nanoseconds{0};
  std::array<double, 3> position{{0.0, 0.0, 0.0}};
  std::array<double, 3> velocity{{0.0, 0.0, 0.0}};
};

struct InternalTrack {
  std::uint64_t numeric_id{0};
  std::int64_t first_seen_nanoseconds{0};
  std::int64_t last_update_nanoseconds{0};
  std::int64_t last_frame_nanoseconds{0};
  ConstantAccelerationKalman x_filter;
  ConstantAccelerationKalman y_filter;
  double z{0.0};
  std::array<double, 3> dimensions{{0.0, 0.0, 0.0}};
  std::array<double, 3> previous_position{{0.0, 0.0, 0.0}};
  std::deque<HistorySample> history;
  TrackLifecycle lifecycle{TrackLifecycle::kNew};
  std::uint64_t age{1};
  std::uint32_t hit_count{1};
  std::uint32_t missed_count{0};
  float confidence{0.0F};
};

struct AssociationCandidate {
  std::size_t detection_index{0};
  std::size_t track_index{0};
  double score{-std::numeric_limits<double>::infinity()};
  double distance{std::numeric_limits<double>::infinity()};
};

double elapsed_seconds(std::int64_t current, std::int64_t previous) {
  if (current <= previous) {
    return 0.0;
  }
  return static_cast<double>(current - previous) * 1.0e-9;
}

std::array<double, 9>
feature(const std::array<double, 3> &position,
        const std::array<double, 3> &dimensions,
        const std::array<double, 3> &sensor_position,
        const std::array<double, 9> &weights) {
  std::array<double, 9> result{
      {(position[0] - sensor_position[0]) * weights[0],
       (position[1] - sensor_position[1]) * weights[1],
       (position[2] - sensor_position[2]) * weights[2],
       dimensions[0] * weights[3], dimensions[1] * weights[4],
       dimensions[2] * weights[5], position[0] * weights[6],
       position[1] * weights[7], position[2] * weights[8]}};
  for (auto &value : result) {
    if (!std::isfinite(value)) {
      value = 0.0;
    }
  }
  return result;
}

double cosine_similarity(const std::array<double, 9> &left,
                         const std::array<double, 9> &right) {
  double dot = 0.0;
  double left_norm = 0.0;
  double right_norm = 0.0;
  for (std::size_t index = 0; index < left.size(); ++index) {
    dot += left[index] * right[index];
    left_norm += left[index] * left[index];
    right_norm += right[index] * right[index];
  }
  if (left_norm <= 1.0e-12 || right_norm <= 1.0e-12) {
    return -1.0;
  }
  return dot / std::sqrt(left_norm * right_norm);
}

std::string track_name(std::uint64_t identifier) {
  std::ostringstream stream;
  stream << "lv_dot_ros2_track_" << std::setw(6) << std::setfill('0')
         << identifier;
  return stream.str();
}

std::array<std::uint8_t, 16> track_uuid(std::uint64_t identifier) {
  std::array<std::uint8_t, 16> result{};
  result[0] = 'L';
  result[1] = 'V';
  result[2] = 'D';
  result[3] = '3';
  for (std::size_t index = 0; index < sizeof(identifier); ++index) {
    result[result.size() - 1U - index] =
        static_cast<std::uint8_t>((identifier >> (index * 8U)) & 0xffU);
  }
  return result;
}

void validate(const TrackingConfiguration &configuration) {
  const auto &noise = configuration.kalman_noise;
  if (configuration.max_match_range <= 0.0 ||
      configuration.max_size_difference <= 0.0 ||
      configuration.history_size == 0 ||
      configuration.fix_size_history_threshold == 0 ||
      configuration.fix_size_dimension_threshold < 0.0 ||
      configuration.kalman_averaging_frames == 0 ||
      configuration.confirmation_hits == 0 ||
      configuration.maximum_missed_frames == 0 ||
      noise.initial_covariance <= 0.0 || noise.process_position < 0.0 ||
      noise.process_velocity < 0.0 || noise.process_acceleration < 0.0 ||
      noise.measurement_position <= 0.0 ||
      noise.measurement_velocity <= 0.0 ||
      noise.measurement_acceleration <= 0.0) {
    throw std::invalid_argument("Invalid LV-DOT tracking configuration");
  }
}

} // namespace

class MultiObjectTracker::Impl {
public:
  void configure(const TrackingConfiguration &configuration) {
    validate(configuration);
    configuration_ = configuration;
    configured_ = true;
    reset();
  }

  void reset() {
    tracks_.clear();
    next_track_id_ = 1;
    last_stamp_nanoseconds_ = 0;
    has_last_stamp_ = false;
  }

  TrackingResult update(const std::vector<LidarCluster> &detections,
                        std::int64_t stamp_nanoseconds,
                        const std::array<double, 3> &sensor_position) {
    if (!configured_) {
      throw std::logic_error(
          "MultiObjectTracker must be configured before update");
    }
    if (stamp_nanoseconds < 0 ||
        (has_last_stamp_ && stamp_nanoseconds <= last_stamp_nanoseconds_)) {
      throw std::invalid_argument(
          "Tracking timestamps must be positive and strictly monotonic");
    }

    TrackingResult result;
    auto &statistics = result.statistics;
    statistics.detection_count = detections.size();
    for (auto &track : tracks_) {
      track.previous_position = {
          {track.x_filter.position(), track.y_filter.position(), track.z}};
      const double dt = elapsed_seconds(stamp_nanoseconds,
                                        track.last_frame_nanoseconds);
      track.x_filter.predict(dt);
      track.y_filter.predict(dt);
      track.last_frame_nanoseconds = stamp_nanoseconds;
      ++track.age;
    }

    const std::size_t existing_track_count = tracks_.size();
    std::vector<AssociationCandidate> candidates;
    std::vector<std::size_t> preferred_track(
        detections.size(), std::numeric_limits<std::size_t>::max());
    std::vector<double> preferred_score(
        detections.size(), -std::numeric_limits<double>::infinity());
    for (std::size_t detection_index = 0;
         detection_index < detections.size(); ++detection_index) {
      const auto &detection = detections[detection_index];
      const auto current_feature = feature(
          detection.center, detection.dimensions, sensor_position,
          configuration_.feature_weights);
      for (std::size_t track_index = 0; track_index < tracks_.size();
           ++track_index) {
        const auto &track = tracks_[track_index];
        const std::array<double, 3> predicted_position{
            {track.x_filter.position(), track.y_filter.position(), track.z}};
        const double dx = predicted_position[0] - detection.center[0];
        const double dy = predicted_position[1] - detection.center[1];
        const double distance = std::hypot(dx, dy);
        const double predicted_width =
            std::max(track.dimensions[0], track.dimensions[1]);
        const double detected_width =
            std::max(detection.dimensions[0], detection.dimensions[1]);
        if (distance >= configuration_.max_match_range ||
            std::abs(predicted_width - detected_width) >=
                configuration_.max_size_difference) {
          continue;
        }
        const auto previous_feature = feature(
            track.previous_position, track.dimensions, sensor_position,
            configuration_.feature_weights);
        const auto predicted_feature = feature(
            predicted_position, track.dimensions, sensor_position,
            configuration_.feature_weights);
        const double score =
            cosine_similarity(previous_feature, current_feature) +
            cosine_similarity(predicted_feature, current_feature);
        candidates.push_back(
            AssociationCandidate{detection_index, track_index, score,
                                 distance});
        if (score > preferred_score[detection_index]) {
          preferred_score[detection_index] = score;
          preferred_track[detection_index] = track_index;
        }
      }
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const AssociationCandidate &left,
                 const AssociationCandidate &right) {
                if (left.score != right.score) {
                  return left.score > right.score;
                }
                return left.distance < right.distance;
              });

    std::vector<int> detection_to_track(detections.size(), -1);
    std::vector<bool> track_assigned(tracks_.size(), false);
    double match_distance_sum = 0.0;
    for (const auto &candidate : candidates) {
      if (detection_to_track[candidate.detection_index] >= 0 ||
          track_assigned[candidate.track_index]) {
        continue;
      }
      detection_to_track[candidate.detection_index] =
          static_cast<int>(candidate.track_index);
      track_assigned[candidate.track_index] = true;
      match_distance_sum += candidate.distance;
      ++statistics.matched_count;
      if (preferred_track[candidate.detection_index] !=
          candidate.track_index) {
        ++statistics.id_switch_count;
      }
    }

    const auto kalman_start = std::chrono::steady_clock::now();
    for (std::size_t detection_index = 0;
         detection_index < detections.size(); ++detection_index) {
      const int matched_index = detection_to_track[detection_index];
      if (matched_index < 0) {
        create_track(detections[detection_index], stamp_nanoseconds);
        ++statistics.created_track_count;
        if (preferred_track[detection_index] !=
            std::numeric_limits<std::size_t>::max()) {
          ++statistics.id_switch_count;
        }
        continue;
      }
      update_track(tracks_[static_cast<std::size_t>(matched_index)],
                   detections[detection_index], stamp_nanoseconds);
    }
    const auto kalman_end = std::chrono::steady_clock::now();
    statistics.kalman_update_time_ms =
        std::chrono::duration<double, std::milli>(kalman_end - kalman_start)
            .count();

    for (std::size_t index = 0; index < existing_track_count; ++index) {
      if (index < track_assigned.size() && track_assigned[index]) {
        continue;
      }
      auto &track = tracks_[index];
      ++track.missed_count;
      track.lifecycle = TrackLifecycle::kLost;
      track.confidence = std::max(0.0F, track.confidence * 0.8F);
    }
    const auto old_size = tracks_.size();
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
                       [this](InternalTrack &track) {
                         if (track.missed_count >
                             configuration_.maximum_missed_frames) {
                           track.lifecycle = TrackLifecycle::kRemoved;
                           return true;
                         }
                         return false;
                       }),
        tracks_.end());
    statistics.removed_track_count = old_size - tracks_.size();

    result.tracks.reserve(tracks_.size());
    double velocity_sum = 0.0;
    for (const auto &track : tracks_) {
      result.tracks.push_back(to_estimate(track));
      velocity_sum +=
          std::hypot(track.x_filter.velocity(), track.y_filter.velocity());
      ++statistics.active_track_count;
      if (track.lifecycle == TrackLifecycle::kConfirmed) {
        ++statistics.confirmed_track_count;
      } else if (track.lifecycle == TrackLifecycle::kLost) {
        ++statistics.lost_track_count;
      }
    }
    statistics.match_success_rate =
        detections.empty()
            ? 1.0
            : static_cast<double>(statistics.matched_count) /
                  static_cast<double>(detections.size());
    statistics.average_match_distance =
        statistics.matched_count == 0
            ? 0.0
            : match_distance_sum /
                  static_cast<double>(statistics.matched_count);
    statistics.average_velocity =
        tracks_.empty() ? 0.0
                        : velocity_sum / static_cast<double>(tracks_.size());
    last_stamp_nanoseconds_ = stamp_nanoseconds;
    has_last_stamp_ = true;
    return result;
  }

private:
  void create_track(const LidarCluster &detection,
                    std::int64_t stamp_nanoseconds) {
    InternalTrack track;
    track.numeric_id = next_track_id_++;
    track.first_seen_nanoseconds = stamp_nanoseconds;
    track.last_update_nanoseconds = stamp_nanoseconds;
    track.last_frame_nanoseconds = stamp_nanoseconds;
    track.x_filter.initialize(detection.center[0],
                              configuration_.kalman_noise);
    track.y_filter.initialize(detection.center[1],
                              configuration_.kalman_noise);
    track.z = detection.center[2];
    track.dimensions = detection.dimensions;
    track.previous_position = detection.center;
    track.confidence = static_cast<float>(
        std::min(1.0, 1.0 / static_cast<double>(
                              configuration_.confirmation_hits)));
    track.history.push_front(HistorySample{
        stamp_nanoseconds, detection.center, {{0.0, 0.0, 0.0}}});
    tracks_.push_back(std::move(track));
  }

  void update_track(InternalTrack &track, const LidarCluster &detection,
                    std::int64_t stamp_nanoseconds) {
    const std::size_t history_index =
        std::min<std::size_t>(configuration_.kalman_averaging_frames - 1U,
                              track.history.size() - 1U);
    const auto &previous = track.history[history_index];
    const double measurement_dt =
        elapsed_seconds(stamp_nanoseconds, previous.stamp_nanoseconds);
    double measured_vx = track.x_filter.velocity();
    double measured_vy = track.y_filter.velocity();
    double measured_ax = track.x_filter.acceleration();
    double measured_ay = track.y_filter.acceleration();
    if (measurement_dt > 0.0) {
      measured_vx =
          (detection.center[0] - previous.position[0]) / measurement_dt;
      measured_vy =
          (detection.center[1] - previous.position[1]) / measurement_dt;
      measured_ax =
          (measured_vx - previous.velocity[0]) / measurement_dt;
      measured_ay =
          (measured_vy - previous.velocity[1]) / measurement_dt;
    }
    track.x_filter.update(detection.center[0], measured_vx, measured_ax);
    track.y_filter.update(detection.center[1], measured_vy, measured_ay);
    track.z = detection.center[2];

    bool keep_previous_size =
        track.history.size() >=
        configuration_.fix_size_history_threshold;
    for (std::size_t axis = 0; axis < 3 && keep_previous_size; ++axis) {
      const double denominator = std::max(1.0e-6, track.dimensions[axis]);
      keep_previous_size =
          std::abs(detection.dimensions[axis] - track.dimensions[axis]) /
              denominator <=
          configuration_.fix_size_dimension_threshold;
    }
    if (!keep_previous_size) {
      track.dimensions = detection.dimensions;
    }
    ++track.hit_count;
    track.missed_count = 0;
    track.lifecycle =
        track.hit_count >= configuration_.confirmation_hits
            ? TrackLifecycle::kConfirmed
            : TrackLifecycle::kNew;
    track.confidence = static_cast<float>(std::min(
        1.0, static_cast<double>(track.hit_count) /
                 static_cast<double>(configuration_.confirmation_hits)));
    track.last_update_nanoseconds = stamp_nanoseconds;
    track.history.push_front(HistorySample{
        stamp_nanoseconds,
        {{track.x_filter.position(), track.y_filter.position(), track.z}},
        {{track.x_filter.velocity(), track.y_filter.velocity(), 0.0}}});
    while (track.history.size() > configuration_.history_size) {
      track.history.pop_back();
    }
  }

  TrackEstimate to_estimate(const InternalTrack &track) const {
    TrackEstimate estimate;
    estimate.uuid = track_uuid(track.numeric_id);
    estimate.track_id = track_name(track.numeric_id);
    estimate.first_seen_nanoseconds = track.first_seen_nanoseconds;
    estimate.last_update_nanoseconds = track.last_update_nanoseconds;
    estimate.source = ObservationSource::kLidar;
    estimate.classification = ObjectClass::kUnknown;
    estimate.position =
        {{track.x_filter.position(), track.y_filter.position(), track.z}};
    estimate.linear_velocity = {{track.x_filter.velocity(),
                                 track.y_filter.velocity(), 0.0}};
    estimate.linear_acceleration = {{track.x_filter.acceleration(),
                                     track.y_filter.acceleration(), 0.0}};
    estimate.dimensions = track.dimensions;
    estimate.pose_covariance[0] = track.x_filter.covariance(0, 0);
    estimate.pose_covariance[7] = track.y_filter.covariance(0, 0);
    estimate.pose_covariance[14] =
        configuration_.kalman_noise.measurement_position;
    estimate.twist_covariance[0] = track.x_filter.covariance(1, 1);
    estimate.twist_covariance[7] = track.y_filter.covariance(1, 1);
    estimate.twist_covariance[14] =
        configuration_.kalman_noise.measurement_velocity;
    estimate.confidence = track.confidence;
    estimate.lifecycle = track.lifecycle;
    estimate.age = track.age;
    estimate.missed_count = track.missed_count;
    return estimate;
  }

  TrackingConfiguration configuration_;
  std::vector<InternalTrack> tracks_;
  std::uint64_t next_track_id_{1};
  std::int64_t last_stamp_nanoseconds_{0};
  bool has_last_stamp_{false};
  bool configured_{false};
};

MultiObjectTracker::MultiObjectTracker() : impl_(std::make_unique<Impl>()) {}

MultiObjectTracker::~MultiObjectTracker() = default;

MultiObjectTracker::MultiObjectTracker(MultiObjectTracker &&) noexcept =
    default;

MultiObjectTracker &
MultiObjectTracker::operator=(MultiObjectTracker &&) noexcept = default;

void MultiObjectTracker::configure(
    const TrackingConfiguration &configuration) {
  impl_->configure(configuration);
}

void MultiObjectTracker::reset() { impl_->reset(); }

TrackingResult MultiObjectTracker::update(
    const std::vector<LidarCluster> &detections,
    std::int64_t stamp_nanoseconds,
    const std::array<double, 3> &sensor_position) {
  return impl_->update(detections, stamp_nanoseconds, sensor_position);
}

} // namespace uav_usv_lv_dot_core
