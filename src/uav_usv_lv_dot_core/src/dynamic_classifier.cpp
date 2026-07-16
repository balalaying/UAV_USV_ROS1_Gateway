#include "uav_usv_lv_dot_core/dynamic_classifier.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace uav_usv_lv_dot_core {
namespace {

struct ClassificationHistory {
  std::deque<MotionHistorySample> samples;
};

double elapsed_seconds(std::int64_t current, std::int64_t previous) {
  return current > previous ? static_cast<double>(current - previous) * 1.0e-9
                            : 0.0;
}

double horizontal_speed(const std::array<double, 3> &velocity) {
  return std::hypot(velocity[0], velocity[1]);
}

double clamp_probability(double value) {
  return std::max(0.0, std::min(1.0, value));
}

void validate(const DynamicClassificationConfiguration &configuration) {
  if (configuration.frame_skip == 0 ||
      configuration.velocity_threshold <= 0.0 ||
      configuration.voting_threshold < 0.0 ||
      configuration.voting_threshold > 1.0 ||
      configuration.force_dynamic_frames == 0 ||
      configuration.force_dynamic_check_range == 0 ||
      configuration.force_dynamic_frames >
          configuration.force_dynamic_check_range ||
      configuration.consistency_threshold == 0 ||
      configuration.history_size < configuration.frame_skip ||
      configuration.history_size < configuration.consistency_threshold ||
      configuration.history_size < configuration.force_dynamic_check_range) {
    throw std::invalid_argument(
        "Invalid LV-DOT dynamic classification configuration");
  }
}

} // namespace

class DynamicClassifier::Impl {
public:
  void configure(const DynamicClassificationConfiguration &configuration) {
    validate(configuration);
    configuration_ = configuration;
    configured_ = true;
    reset();
  }

  void reset() {
    histories_.clear();
    last_stamp_nanoseconds_ = 0;
    has_last_stamp_ = false;
  }

  DynamicClassificationResult classify(const std::vector<TrackEstimate> &tracks,
                                       std::int64_t stamp_nanoseconds) {
    if (!configured_) {
      throw std::logic_error(
          "DynamicClassifier must be configured before classify");
    }
    if (stamp_nanoseconds < 0 ||
        (has_last_stamp_ && stamp_nanoseconds <= last_stamp_nanoseconds_)) {
      throw std::invalid_argument(
          "Dynamic classification timestamps must be strictly monotonic");
    }

    const auto start = std::chrono::steady_clock::now();
    DynamicClassificationResult result;
    auto &statistics = result.statistics;
    statistics.total_track_count = tracks.size();
    std::unordered_set<std::string> active_track_ids;
    active_track_ids.reserve(tracks.size());
    double velocity_sum = 0.0;

    for (const auto &track : tracks) {
      if (track.track_id.empty()) {
        continue;
      }
      active_track_ids.insert(track.track_id);
      velocity_sum += horizontal_speed(track.linear_velocity);
      auto &history = histories_[track.track_id].samples;
      DynamicTrackEstimate classification;
      classification.track_id = track.track_id;
      classification.track = track;

      if (track.lifecycle == TrackLifecycle::kLost) {
        classification.motion_state = DynamicMotionState::kStatic;
        classification.dynamic_probability = 0.0;
        classification.confidence = 0.0F;
        classification.motion_history.assign(history.begin(), history.end());
        result.classified_tracks.push_back(std::move(classification));
        ++statistics.unclassified_track_count;
        continue;
      }

      MotionHistorySample sample;
      sample.stamp_nanoseconds = stamp_nanoseconds;
      sample.position = track.position;
      sample.velocity = track.linear_velocity;
      sample.speed = horizontal_speed(track.linear_velocity);

      const std::size_t motion_window_size = std::min<std::size_t>(
          configuration_.force_dynamic_check_range, history.size() + 1U);
      if (motion_window_size == configuration_.force_dynamic_check_range) {
        const auto &oldest = history[motion_window_size - 2U];
        const double window_dt =
            elapsed_seconds(stamp_nanoseconds, oldest.stamp_nanoseconds);
        const double net_dx = track.position[0] - oldest.position[0];
        const double net_dy = track.position[1] - oldest.position[1];
        const double net_displacement = std::hypot(net_dx, net_dy);
        if (window_dt > 0.0) {
          sample.displacement_speed = net_displacement / window_dt;
        }
        if (net_displacement > 1.0e-9 && sample.speed > 1.0e-9) {
          sample.direction_similarity = (net_dx * track.linear_velocity[0] +
                                         net_dy * track.linear_velocity[1]) /
                                        (net_displacement * sample.speed);
        }

        std::uint32_t moving_votes = 0;
        std::uint32_t eligible_votes = 0;
        std::array<double, 3> newer_position = track.position;
        std::int64_t newer_stamp = stamp_nanoseconds;
        for (std::size_t index = configuration_.frame_skip - 1U;
             index < motion_window_size - 1U;
             index += configuration_.frame_skip) {
          const auto &older = history[index];
          const double segment_dt =
              elapsed_seconds(newer_stamp, older.stamp_nanoseconds);
          const double segment_dx = newer_position[0] - older.position[0];
          const double segment_dy = newer_position[1] - older.position[1];
          const double segment_displacement =
              std::hypot(segment_dx, segment_dy);
          const double segment_speed =
              segment_dt > 0.0 ? segment_displacement / segment_dt : 0.0;
          const double direction_dot =
              segment_dx * net_dx + segment_dy * net_dy;
          if (segment_speed >= configuration_.velocity_threshold &&
              direction_dot >= 0.0) {
            ++moving_votes;
          }
          ++eligible_votes;
          newer_position = older.position;
          newer_stamp = older.stamp_nanoseconds;
        }
        sample.vote_ratio = eligible_votes == 0
                                ? 0.0
                                : static_cast<double>(moving_votes) /
                                      static_cast<double>(eligible_votes);
        sample.is_candidate =
            sample.speed >= configuration_.velocity_threshold &&
            sample.displacement_speed >= configuration_.velocity_threshold &&
            sample.direction_similarity >= 0.0 &&
            sample.vote_ratio >= configuration_.voting_threshold;
      }

      std::uint32_t consistency_votes = sample.is_candidate ? 1U : 0U;
      const std::size_t consistency_history =
          configuration_.consistency_threshold > 0
              ? std::min<std::size_t>(configuration_.consistency_threshold - 1U,
                                      history.size())
              : 0U;
      for (std::size_t index = 0; index < consistency_history; ++index) {
        if (history[index].is_candidate || history[index].is_dynamic) {
          ++consistency_votes;
        }
      }
      const bool consistency_confirmed =
          consistency_votes >= configuration_.consistency_threshold;

      const std::size_t force_history = std::min<std::size_t>(
          configuration_.force_dynamic_check_range, history.size());
      std::uint32_t previous_dynamic_frames = 0;
      for (std::size_t index = 0; index < force_history; ++index) {
        previous_dynamic_frames += history[index].is_dynamic ? 1U : 0U;
      }
      const bool force_dynamic =
          previous_dynamic_frames >= configuration_.force_dynamic_frames;
      sample.is_dynamic = consistency_confirmed || force_dynamic;

      const double speed_evidence =
          clamp_probability(sample.speed / configuration_.velocity_threshold);
      const double consistency_ratio =
          static_cast<double>(consistency_votes) /
          static_cast<double>(configuration_.consistency_threshold);
      classification.dynamic_probability =
          clamp_probability(0.4 * speed_evidence + 0.35 * sample.vote_ratio +
                            0.25 * clamp_probability(consistency_ratio));
      classification.is_dynamic = sample.is_dynamic;
      if (sample.is_dynamic) {
        classification.motion_state = DynamicMotionState::kConfirmedDynamic;
        ++statistics.confirmed_dynamic_count;
      } else if (sample.is_candidate) {
        classification.motion_state = DynamicMotionState::kMovingCandidate;
        ++statistics.candidate_track_count;
      } else {
        classification.motion_state = DynamicMotionState::kStatic;
        ++statistics.static_track_count;
      }
      classification.confidence = static_cast<float>(
          clamp_probability(static_cast<double>(track.confidence) *
                            classification.dynamic_probability));

      history.push_front(sample);
      while (history.size() > configuration_.history_size) {
        history.pop_back();
      }
      classification.motion_history.assign(history.begin(), history.end());
      if (classification.is_dynamic) {
        auto dynamic_track = track;
        dynamic_track.confidence = classification.confidence;
        result.dynamic_tracks.push_back(std::move(dynamic_track));
      }
      result.classified_tracks.push_back(std::move(classification));
    }

    for (auto iterator = histories_.begin(); iterator != histories_.end();) {
      if (active_track_ids.count(iterator->first) == 0U) {
        iterator = histories_.erase(iterator);
      } else {
        ++iterator;
      }
    }

    statistics.dynamic_ratio =
        statistics.total_track_count == 0
            ? 0.0
            : static_cast<double>(statistics.confirmed_dynamic_count) /
                  static_cast<double>(statistics.total_track_count);
    statistics.average_velocity =
        tracks.empty() ? 0.0
                       : velocity_sum / static_cast<double>(tracks.size());
    statistics.classification_time_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start)
            .count();
    last_stamp_nanoseconds_ = stamp_nanoseconds;
    has_last_stamp_ = true;
    return result;
  }

private:
  DynamicClassificationConfiguration configuration_;
  std::unordered_map<std::string, ClassificationHistory> histories_;
  std::int64_t last_stamp_nanoseconds_{0};
  bool has_last_stamp_{false};
  bool configured_{false};
};

DynamicClassifier::DynamicClassifier() : impl_(std::make_unique<Impl>()) {}

DynamicClassifier::~DynamicClassifier() = default;

DynamicClassifier::DynamicClassifier(DynamicClassifier &&) noexcept = default;

DynamicClassifier &
DynamicClassifier::operator=(DynamicClassifier &&) noexcept = default;

void DynamicClassifier::configure(
    const DynamicClassificationConfiguration &configuration) {
  impl_->configure(configuration);
}

void DynamicClassifier::reset() { impl_->reset(); }

DynamicClassificationResult
DynamicClassifier::classify(const std::vector<TrackEstimate> &tracks,
                            std::int64_t stamp_nanoseconds) {
  return impl_->classify(tracks, stamp_nanoseconds);
}

} // namespace uav_usv_lv_dot_core
