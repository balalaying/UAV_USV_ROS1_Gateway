#include "uav_usv_lv_dot_ros2/detector_node.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <tf2/time.h>
#include <tf2_ros/transform_listener.h>

#include "uav_usv_lv_dot_core/types.hpp"
#include "uav_usv_lv_dot_ros2/cluster_marker_conversion.hpp"
#include "uav_usv_lv_dot_ros2/message_conversion.hpp"
#include "uav_usv_lv_dot_ros2/pointcloud_conversion.hpp"

namespace uav_usv_lv_dot_ros2 {
namespace {

diagnostic_msgs::msg::KeyValue diagnostic_value(const std::string &key,
                                                const std::string &value) {
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::string fixed(double value, int precision = 3) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

} // namespace

DetectorNode::DetectorNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("lv_dot_detector_node", options) {
  declare_parameter<std::string>("vehicle_id", "");
  declare_parameter<std::string>("output_frame", "map");
  declare_parameter<double>("tf_timeout_seconds", 0.1);
  declare_parameter<double>("diagnostics_period_seconds", 1.0);
  declare_parameter<double>("input_min_range", 0.5);
  declare_parameter<double>("input_max_range", 20.0);
  declare_parameter<double>("input_min_z", -1.75);
  declare_parameter<double>("input_max_z", 4.0);
  declare_parameter<double>("input_voxel_size", 0.04);
  declare_parameter<bool>("crop_self", true);
  declare_parameter<std::vector<double>>(
      "self_bounds", std::vector<double>{-4.3, 2.5, -1.8, 1.8, -2.4, 0.35});
  declare_parameter<double>("local_range_x", 10.0);
  declare_parameter<double>("local_range_y", 10.0);
  declare_parameter<double>("ground_height", 0.22);
  declare_parameter<double>("roof_height", 6.0);
  declare_parameter<std::int64_t>("downsample_threshold", 12000);
  declare_parameter<double>("adaptive_voxel_initial_size", 0.1);
  declare_parameter<double>("gaussian_downsample_sigma", 16.0);
  declare_parameter<std::int64_t>("random_seed", 1);
  declare_parameter<double>("lidar_dbscan_epsilon", 0.65);
  declare_parameter<std::int64_t>("lidar_dbscan_min_points", 3);
  declare_parameter<std::vector<double>>("maximum_object_size",
                                         std::vector<double>{30.0, 15.0, 12.0});
  declare_parameter<double>("tracking_max_match_range", 2.0);
  declare_parameter<double>("tracking_max_size_difference", 8.0);
  declare_parameter<std::vector<double>>(
      "tracking_feature_weights",
      std::vector<double>{3.0, 3.0, 0.1, 0.5, 0.5, 0.05, 0.0, 0.0, 0.0});
  declare_parameter<std::int64_t>("tracking_history_size", 100);
  declare_parameter<std::int64_t>("tracking_fix_size_history_threshold", 10);
  declare_parameter<double>("tracking_fix_size_dimension_threshold", 0.4);
  declare_parameter<std::int64_t>("tracking_kalman_averaging_frames", 3);
  declare_parameter<std::int64_t>("tracking_confirmation_hits", 3);
  declare_parameter<std::int64_t>("tracking_maximum_missed_frames", 5);
  declare_parameter<std::vector<double>>(
      "kalman_filter_parameters",
      std::vector<double>{0.25, 0.01, 0.05, 0.05, 0.04, 0.3, 0.6});
  declare_parameter<std::int64_t>("frame_skip", 2);
  declare_parameter<double>("dynamic_velocity_threshold", 0.05);
  declare_parameter<double>("dynamic_voting_threshold", 0.15);
  declare_parameter<std::int64_t>("frames_force_dynamic", 3);
  declare_parameter<std::int64_t>("frames_force_dynamic_check_range", 12);
  declare_parameter<std::int64_t>("dynamic_consistency_threshold", 2);
  declare_parameter<std::int64_t>("dynamic_history_size", 100);
}

DetectorNode::CallbackReturn
DetectorNode::on_configure(const rclcpp_lifecycle::State &) {
  vehicle_id_ = get_parameter("vehicle_id").as_string();
  output_frame_ = get_parameter("output_frame").as_string();
  tf_timeout_seconds_ = get_parameter("tf_timeout_seconds").as_double();
  diagnostics_period_seconds_ =
      get_parameter("diagnostics_period_seconds").as_double();

  if (output_frame_.empty() || tf_timeout_seconds_ <= 0.0 ||
      diagnostics_period_seconds_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "Invalid output frame or timing parameter");
    return CallbackReturn::FAILURE;
  }

  const auto self_bounds = get_parameter("self_bounds").as_double_array();
  const auto maximum_object_size =
      get_parameter("maximum_object_size").as_double_array();
  const auto tracking_feature_weights =
      get_parameter("tracking_feature_weights").as_double_array();
  const auto kalman_filter_parameters =
      get_parameter("kalman_filter_parameters").as_double_array();
  if (self_bounds.size() != 6 || maximum_object_size.size() != 3 ||
      tracking_feature_weights.size() != 9 ||
      kalman_filter_parameters.size() != 7) {
    RCLCPP_ERROR(
        get_logger(),
        "Expected self_bounds=6, maximum_object_size=3, "
        "tracking_feature_weights=9 and kalman_filter_parameters=7 values");
    return CallbackReturn::FAILURE;
  }
  uav_usv_lv_dot_core::CoreConfiguration core_configuration;
  core_configuration.input_min_range =
      get_parameter("input_min_range").as_double();
  core_configuration.input_max_range =
      get_parameter("input_max_range").as_double();
  core_configuration.input_min_z = get_parameter("input_min_z").as_double();
  core_configuration.input_max_z = get_parameter("input_max_z").as_double();
  core_configuration.input_voxel_size =
      get_parameter("input_voxel_size").as_double();
  core_configuration.crop_self = get_parameter("crop_self").as_bool();
  std::copy(self_bounds.begin(), self_bounds.end(),
            core_configuration.self_bounds.begin());
  core_configuration.local_range_x = get_parameter("local_range_x").as_double();
  core_configuration.local_range_y = get_parameter("local_range_y").as_double();
  core_configuration.ground_height = get_parameter("ground_height").as_double();
  core_configuration.roof_height = get_parameter("roof_height").as_double();
  core_configuration.downsample_threshold =
      static_cast<std::uint64_t>(std::max<std::int64_t>(
          1, get_parameter("downsample_threshold").as_int()));
  core_configuration.adaptive_voxel_initial_size =
      get_parameter("adaptive_voxel_initial_size").as_double();
  core_configuration.gaussian_downsample_sigma =
      get_parameter("gaussian_downsample_sigma").as_double();
  core_configuration.random_seed = static_cast<std::uint32_t>(
      std::max<std::int64_t>(0, get_parameter("random_seed").as_int()));
  core_configuration.dbscan_epsilon_squared =
      get_parameter("lidar_dbscan_epsilon").as_double();
  core_configuration.dbscan_min_points =
      static_cast<std::uint32_t>(std::max<std::int64_t>(
          1, get_parameter("lidar_dbscan_min_points").as_int()));
  std::copy(maximum_object_size.begin(), maximum_object_size.end(),
            core_configuration.maximum_object_size.begin());
  auto &tracking = core_configuration.tracking;
  tracking.max_match_range =
      get_parameter("tracking_max_match_range").as_double();
  tracking.max_size_difference =
      get_parameter("tracking_max_size_difference").as_double();
  std::copy(tracking_feature_weights.begin(), tracking_feature_weights.end(),
            tracking.feature_weights.begin());
  tracking.history_size = static_cast<std::uint32_t>(std::max<std::int64_t>(
      1, get_parameter("tracking_history_size").as_int()));
  tracking.fix_size_history_threshold =
      static_cast<std::uint32_t>(std::max<std::int64_t>(
          1, get_parameter("tracking_fix_size_history_threshold").as_int()));
  tracking.fix_size_dimension_threshold =
      get_parameter("tracking_fix_size_dimension_threshold").as_double();
  tracking.kalman_averaging_frames =
      static_cast<std::uint32_t>(std::max<std::int64_t>(
          1, get_parameter("tracking_kalman_averaging_frames").as_int()));
  tracking.confirmation_hits =
      static_cast<std::uint32_t>(std::max<std::int64_t>(
          1, get_parameter("tracking_confirmation_hits").as_int()));
  tracking.maximum_missed_frames =
      static_cast<std::uint32_t>(std::max<std::int64_t>(
          1, get_parameter("tracking_maximum_missed_frames").as_int()));
  tracking.kalman_noise.initial_covariance = kalman_filter_parameters[0];
  tracking.kalman_noise.process_position = kalman_filter_parameters[1];
  tracking.kalman_noise.process_velocity = kalman_filter_parameters[2];
  tracking.kalman_noise.process_acceleration = kalman_filter_parameters[3];
  tracking.kalman_noise.measurement_position = kalman_filter_parameters[4];
  tracking.kalman_noise.measurement_velocity = kalman_filter_parameters[5];
  tracking.kalman_noise.measurement_acceleration = kalman_filter_parameters[6];
  auto &dynamic = core_configuration.dynamic_classification;
  dynamic.frame_skip = static_cast<std::uint32_t>(
      std::max<std::int64_t>(1, get_parameter("frame_skip").as_int()));
  dynamic.velocity_threshold =
      get_parameter("dynamic_velocity_threshold").as_double();
  dynamic.voting_threshold =
      get_parameter("dynamic_voting_threshold").as_double();
  dynamic.force_dynamic_frames =
      static_cast<std::uint32_t>(std::max<std::int64_t>(
          1, get_parameter("frames_force_dynamic").as_int()));
  dynamic.force_dynamic_check_range =
      static_cast<std::uint32_t>(std::max<std::int64_t>(
          1, get_parameter("frames_force_dynamic_check_range").as_int()));
  dynamic.consistency_threshold =
      static_cast<std::uint32_t>(std::max<std::int64_t>(
          1, get_parameter("dynamic_consistency_threshold").as_int()));
  dynamic.history_size = static_cast<std::uint32_t>(std::max<std::int64_t>(
      1, get_parameter("dynamic_history_size").as_int()));
  try {
    detector_core_.configure(core_configuration);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "Invalid Phase 4 core parameters: %s",
                 error.what());
    return CallbackReturn::FAILURE;
  }
  reset_statistics();
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  observations_publisher_ =
      create_publisher<uav_usv_interfaces::msg::TrackedObjectArray>(
          "observations", rclcpp::QoS(10).reliable());
  tracks_publisher_ =
      create_publisher<uav_usv_interfaces::msg::TrackedObjectArray>(
          "tracks", rclcpp::QoS(10).reliable());
  dynamic_tracks_publisher_ =
      create_publisher<uav_usv_interfaces::msg::TrackedObjectArray>(
          "dynamic_tracks", rclcpp::QoS(10).reliable());
  diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          "diagnostics", rclcpp::QoS(10).reliable());
  lidar_bboxes_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
          "diagnostics/lidar_bboxes", rclcpp::QoS(5).best_effort());
  diagnostics_timer_ = create_wall_timer(
      std::chrono::duration<double>(diagnostics_period_seconds_),
      std::bind(&DetectorNode::publish_diagnostics, this));
  diagnostics_timer_->cancel();

  RCLCPP_INFO(get_logger(),
              "Configured Phase 4 LiDAR dynamic classifier for vehicle '%s', "
              "output frame '%s'",
              vehicle_id_.c_str(), output_frame_.c_str());
  return CallbackReturn::SUCCESS;
}

DetectorNode::CallbackReturn
DetectorNode::on_activate(const rclcpp_lifecycle::State &state) {
  const auto result = rclcpp_lifecycle::LifecycleNode::on_activate(state);
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }

  cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points", rclcpp::SensorDataQoS(),
      std::bind(&DetectorNode::cloud_callback, this, std::placeholders::_1));
  diagnostics_timer_->reset();
  RCLCPP_INFO(get_logger(),
              "Activated; waiting for PointCloud2 on relative topic 'points'");
  return CallbackReturn::SUCCESS;
}

DetectorNode::CallbackReturn
DetectorNode::on_deactivate(const rclcpp_lifecycle::State &state) {
  cloud_subscription_.reset();
  if (diagnostics_timer_) {
    diagnostics_timer_->cancel();
  }
  RCLCPP_INFO(get_logger(), "Deactivated");
  return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
}

DetectorNode::CallbackReturn
DetectorNode::on_cleanup(const rclcpp_lifecycle::State &) {
  cloud_subscription_.reset();
  diagnostics_timer_.reset();
  observations_publisher_.reset();
  tracks_publisher_.reset();
  dynamic_tracks_publisher_.reset();
  diagnostics_publisher_.reset();
  lidar_bboxes_publisher_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  detector_core_.reset();
  reset_statistics();
  RCLCPP_INFO(get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

DetectorNode::CallbackReturn
DetectorNode::on_shutdown(const rclcpp_lifecycle::State &) {
  cloud_subscription_.reset();
  diagnostics_timer_.reset();
  RCLCPP_INFO(get_logger(), "Shutdown complete");
  return CallbackReturn::SUCCESS;
}

void DetectorNode::cloud_callback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
  const auto processing_start = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    ++statistics_.input_count;
  }

  if (message->header.frame_id.empty()) {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    ++statistics_.malformed_cloud_count;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Dropping cloud with empty frame_id");
    return;
  }

  const rclcpp::Time stamp(message->header.stamp,
                           get_clock()->get_clock_type());
  const auto stamp_nanoseconds = stamp.nanoseconds();
  {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    if (statistics_.previous_stamp_nanoseconds != 0 &&
        stamp_nanoseconds <= statistics_.previous_stamp_nanoseconds) {
      ++statistics_.nonmonotonic_stamp_count;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Dropping non-monotonic cloud timestamp");
      return;
    }
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
        output_frame_, message->header.frame_id, message->header.stamp,
        tf2::durationFromSec(tf_timeout_seconds_));
  } catch (const tf2::TransformException &error) {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    ++statistics_.tf_failure_count;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Dropping cloud because timestamped TF failed: %s",
                         error.what());
    return;
  }
  {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    ++statistics_.tf_success_count;
  }

  uav_usv_lv_dot_core::PointCloudFrame frame;
  try {
    convert_point_cloud(*message, frame);
  } catch (const std::exception &error) {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    ++statistics_.malformed_cloud_count;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Dropping malformed PointCloud2: %s", error.what());
    return;
  }

  frame.context.stamp_nanoseconds = stamp_nanoseconds;
  frame.context.sensor_frame = message->header.frame_id;
  frame.context.output_frame = output_frame_;
  frame.context.sensor_to_output.translation = {
      transform.transform.translation.x, transform.transform.translation.y,
      transform.transform.translation.z};
  frame.context.sensor_to_output.rotation_xyzw = {
      transform.transform.rotation.x, transform.transform.rotation.y,
      transform.transform.rotation.z, transform.transform.rotation.w};

  {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    if (statistics_.previous_stamp_nanoseconds != 0) {
      statistics_.last_dt_seconds =
          static_cast<double>(stamp_nanoseconds -
                              statistics_.previous_stamp_nanoseconds) *
          1e-9;
      statistics_.sum_dt_seconds += statistics_.last_dt_seconds;
      ++statistics_.dt_sample_count;
    } else {
      statistics_.last_dt_seconds = 0.0;
    }
    frame.context.dt_seconds = statistics_.last_dt_seconds;
    statistics_.previous_stamp_nanoseconds = stamp_nanoseconds;
    ++statistics_.accepted_count;
    statistics_.last_input_point_count = frame.points.size();
    // Bag playback can advance /clock just after delivering a sensor message.
    // Latency is therefore clamped at zero instead of reporting a negative
    // value.
    statistics_.last_input_latency_ms =
        std::max(0.0, (now() - stamp).seconds() * 1000.0);
    statistics_.sum_input_latency_ms += statistics_.last_input_latency_ms;
  }

  auto result = detector_core_.process(frame);
  auto markers = to_lidar_bbox_markers(result);
  auto tracks = to_ros_message(result);
  auto dynamic_result = result;
  dynamic_result.tracks = result.dynamic_tracks;
  auto dynamic_tracks = to_ros_message(dynamic_result);
  auto observation_result = result;
  observation_result.tracks.clear();
  auto observations = to_ros_message(observation_result);
  if (lidar_bboxes_publisher_->is_activated()) {
    lidar_bboxes_publisher_->publish(std::move(markers));
  }
  if (observations_publisher_->is_activated()) {
    observations_publisher_->publish(std::move(observations));
  }
  if (tracks_publisher_->is_activated()) {
    tracks_publisher_->publish(std::move(tracks));
  }
  if (dynamic_tracks_publisher_->is_activated()) {
    dynamic_tracks_publisher_->publish(std::move(dynamic_tracks));
  }

  const auto processing_end = std::chrono::steady_clock::now();
  const double processing_ms = std::chrono::duration<double, std::milli>(
                                   processing_end - processing_start)
                                   .count();
  {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    statistics_.last_processing_time_ms = processing_ms;
    statistics_.sum_processing_time_ms += processing_ms;
    statistics_.last_finite_point_count =
        result.clustering_statistics.finite_point_count;
    statistics_.last_preprocessed_point_count =
        result.clustering_statistics.preprocessed_point_count;
    statistics_.last_clustered_point_count =
        result.clustering_statistics.clustered_point_count;
    statistics_.last_noise_point_count =
        result.clustering_statistics.noise_point_count;
    statistics_.last_cluster_count = result.lidar_clusters.size();
    statistics_.sum_cluster_count += result.lidar_clusters.size();
    statistics_.last_preprocessing_time_ms =
        result.clustering_statistics.preprocessing_time_ms;
    statistics_.sum_preprocessing_time_ms +=
        result.clustering_statistics.preprocessing_time_ms;
    statistics_.last_clustering_time_ms =
        result.clustering_statistics.clustering_time_ms;
    statistics_.sum_clustering_time_ms +=
        result.clustering_statistics.clustering_time_ms;
    statistics_.last_detection_count =
        result.tracking_statistics.detection_count;
    statistics_.last_matched_count = result.tracking_statistics.matched_count;
    statistics_.last_created_track_count =
        result.tracking_statistics.created_track_count;
    statistics_.last_removed_track_count =
        result.tracking_statistics.removed_track_count;
    statistics_.last_active_track_count =
        result.tracking_statistics.active_track_count;
    statistics_.last_confirmed_track_count =
        result.tracking_statistics.confirmed_track_count;
    statistics_.last_lost_track_count =
        result.tracking_statistics.lost_track_count;
    statistics_.total_detection_count +=
        result.tracking_statistics.detection_count;
    statistics_.total_matched_count += result.tracking_statistics.matched_count;
    statistics_.total_created_track_count +=
        result.tracking_statistics.created_track_count;
    statistics_.total_removed_track_count +=
        result.tracking_statistics.removed_track_count;
    statistics_.total_id_switch_count +=
        result.tracking_statistics.id_switch_count;
    statistics_.total_match_distance +=
        result.tracking_statistics.average_match_distance *
        static_cast<double>(result.tracking_statistics.matched_count);
    statistics_.last_match_success_rate =
        result.tracking_statistics.match_success_rate;
    statistics_.last_average_match_distance =
        result.tracking_statistics.average_match_distance;
    statistics_.last_average_velocity =
        result.tracking_statistics.average_velocity;
    statistics_.last_kalman_update_time_ms =
        result.tracking_statistics.kalman_update_time_ms;
    statistics_.sum_kalman_update_time_ms +=
        result.tracking_statistics.kalman_update_time_ms;
    statistics_.last_dynamic_total_track_count =
        result.dynamic_statistics.total_track_count;
    statistics_.last_dynamic_static_count =
        result.dynamic_statistics.static_track_count;
    statistics_.last_dynamic_candidate_count =
        result.dynamic_statistics.candidate_track_count;
    statistics_.last_dynamic_confirmed_count =
        result.dynamic_statistics.confirmed_dynamic_count;
    statistics_.last_dynamic_unclassified_count =
        result.dynamic_statistics.unclassified_track_count;
    statistics_.total_dynamic_confirmed_count +=
        result.dynamic_statistics.confirmed_dynamic_count;
    statistics_.last_dynamic_ratio = result.dynamic_statistics.dynamic_ratio;
    statistics_.last_dynamic_average_velocity =
        result.dynamic_statistics.average_velocity;
    statistics_.last_dynamic_classification_time_ms =
        result.dynamic_statistics.classification_time_ms;
    statistics_.sum_dynamic_classification_time_ms +=
        result.dynamic_statistics.classification_time_ms;
  }
}

void DetectorNode::publish_diagnostics() {
  if (!diagnostics_publisher_ || !diagnostics_publisher_->is_activated()) {
    return;
  }

  RuntimeStatistics statistics;
  {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    statistics = statistics_;
  }

  const auto tf_total =
      statistics.tf_success_count + statistics.tf_failure_count;
  const double tf_success_rate =
      tf_total == 0 ? 0.0
                    : static_cast<double>(statistics.tf_success_count) /
                          static_cast<double>(tf_total);
  const double average_dt =
      statistics.dt_sample_count == 0
          ? 0.0
          : statistics.sum_dt_seconds /
                static_cast<double>(statistics.dt_sample_count);
  const double input_rate = average_dt > 0.0 ? 1.0 / average_dt : 0.0;
  const double average_processing =
      statistics.accepted_count == 0
          ? 0.0
          : statistics.sum_processing_time_ms /
                static_cast<double>(statistics.accepted_count);
  const double average_latency =
      statistics.accepted_count == 0
          ? 0.0
          : statistics.sum_input_latency_ms /
                static_cast<double>(statistics.accepted_count);
  const double average_clusters =
      statistics.accepted_count == 0
          ? 0.0
          : static_cast<double>(statistics.sum_cluster_count) /
                static_cast<double>(statistics.accepted_count);
  const double average_preprocessing =
      statistics.accepted_count == 0
          ? 0.0
          : statistics.sum_preprocessing_time_ms /
                static_cast<double>(statistics.accepted_count);
  const double average_clustering =
      statistics.accepted_count == 0
          ? 0.0
          : statistics.sum_clustering_time_ms /
                static_cast<double>(statistics.accepted_count);
  const double average_kalman_update =
      statistics.accepted_count == 0
          ? 0.0
          : statistics.sum_kalman_update_time_ms /
                static_cast<double>(statistics.accepted_count);
  const double average_dynamic_classification =
      statistics.accepted_count == 0
          ? 0.0
          : statistics.sum_dynamic_classification_time_ms /
                static_cast<double>(statistics.accepted_count);
  const double cumulative_match_success =
      statistics.total_detection_count == 0
          ? 1.0
          : static_cast<double>(statistics.total_matched_count) /
                static_cast<double>(statistics.total_detection_count);
  const double cumulative_match_distance =
      statistics.total_matched_count == 0
          ? 0.0
          : statistics.total_match_distance /
                static_cast<double>(statistics.total_matched_count);

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_node_base_interface()->get_fully_qualified_name() +
                std::string(": phase4_dynamic_classification");
  status.hardware_id = vehicle_id_.empty() ? "unassigned_vehicle" : vehicle_id_;
  if (statistics.input_count == 0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "ACTIVE: waiting for PointCloud2";
  } else if (tf_success_rate < 0.99) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "ACTIVE: timestamped TF failures detected";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "ACTIVE: Phase 4 dynamic classification healthy";
  }

  status.values.push_back(
      diagnostic_value("node_state", get_current_state().label()));
  status.values.push_back(diagnostic_value("vehicle_id", vehicle_id_));
  status.values.push_back(diagnostic_value("output_frame", output_frame_));
  status.values.push_back(
      diagnostic_value("input_count", std::to_string(statistics.input_count)));
  status.values.push_back(diagnostic_value(
      "accepted_count", std::to_string(statistics.accepted_count)));
  status.values.push_back(diagnostic_value("input_rate_hz", fixed(input_rate)));
  status.values.push_back(
      diagnostic_value("tf_success_rate", fixed(tf_success_rate, 6)));
  status.values.push_back(diagnostic_value(
      "tf_success_count", std::to_string(statistics.tf_success_count)));
  status.values.push_back(diagnostic_value(
      "tf_failure_count", std::to_string(statistics.tf_failure_count)));
  status.values.push_back(
      diagnostic_value("malformed_cloud_count",
                       std::to_string(statistics.malformed_cloud_count)));
  status.values.push_back(
      diagnostic_value("nonmonotonic_stamp_count",
                       std::to_string(statistics.nonmonotonic_stamp_count)));
  status.values.push_back(diagnostic_value(
      "last_dt_seconds", fixed(statistics.last_dt_seconds, 6)));
  status.values.push_back(
      diagnostic_value("average_dt_seconds", fixed(average_dt, 6)));
  status.values.push_back(diagnostic_value(
      "last_processing_time_ms", fixed(statistics.last_processing_time_ms)));
  status.values.push_back(diagnostic_value("average_processing_time_ms",
                                           fixed(average_processing)));
  status.values.push_back(diagnostic_value(
      "last_input_latency_ms", fixed(statistics.last_input_latency_ms)));
  status.values.push_back(
      diagnostic_value("average_input_latency_ms", fixed(average_latency)));
  status.values.push_back(
      diagnostic_value("last_input_point_count",
                       std::to_string(statistics.last_input_point_count)));
  status.values.push_back(
      diagnostic_value("last_finite_point_count",
                       std::to_string(statistics.last_finite_point_count)));
  status.values.push_back(diagnostic_value(
      "last_preprocessed_point_count",
      std::to_string(statistics.last_preprocessed_point_count)));
  status.values.push_back(
      diagnostic_value("last_clustered_point_count",
                       std::to_string(statistics.last_clustered_point_count)));
  status.values.push_back(
      diagnostic_value("last_noise_point_count",
                       std::to_string(statistics.last_noise_point_count)));
  status.values.push_back(diagnostic_value(
      "last_cluster_count", std::to_string(statistics.last_cluster_count)));
  status.values.push_back(
      diagnostic_value("average_cluster_count", fixed(average_clusters)));
  status.values.push_back(
      diagnostic_value("last_preprocessing_time_ms",
                       fixed(statistics.last_preprocessing_time_ms)));
  status.values.push_back(diagnostic_value("average_preprocessing_time_ms",
                                           fixed(average_preprocessing)));
  status.values.push_back(diagnostic_value(
      "last_clustering_time_ms", fixed(statistics.last_clustering_time_ms)));
  status.values.push_back(diagnostic_value("average_clustering_time_ms",
                                           fixed(average_clustering)));
  status.values.push_back(
      diagnostic_value("tracking_detection_count",
                       std::to_string(statistics.last_detection_count)));
  status.values.push_back(diagnostic_value(
      "tracking_matched_count", std::to_string(statistics.last_matched_count)));
  status.values.push_back(
      diagnostic_value("tracking_created_count",
                       std::to_string(statistics.last_created_track_count)));
  status.values.push_back(
      diagnostic_value("tracking_removed_count",
                       std::to_string(statistics.last_removed_track_count)));
  status.values.push_back(
      diagnostic_value("tracking_active_count",
                       std::to_string(statistics.last_active_track_count)));
  status.values.push_back(
      diagnostic_value("tracking_confirmed_count",
                       std::to_string(statistics.last_confirmed_track_count)));
  status.values.push_back(diagnostic_value(
      "tracking_lost_count", std::to_string(statistics.last_lost_track_count)));
  status.values.push_back(
      diagnostic_value("tracking_id_switch_count",
                       std::to_string(statistics.total_id_switch_count)));
  status.values.push_back(
      diagnostic_value("tracking_detection_count_total",
                       std::to_string(statistics.total_detection_count)));
  status.values.push_back(
      diagnostic_value("tracking_matched_count_total",
                       std::to_string(statistics.total_matched_count)));
  status.values.push_back(
      diagnostic_value("tracking_created_count_total",
                       std::to_string(statistics.total_created_track_count)));
  status.values.push_back(
      diagnostic_value("tracking_removed_count_total",
                       std::to_string(statistics.total_removed_track_count)));
  status.values.push_back(diagnostic_value("tracking_match_success_rate_total",
                                           fixed(cumulative_match_success, 6)));
  status.values.push_back(
      diagnostic_value("tracking_average_match_distance_total_m",
                       fixed(cumulative_match_distance)));
  status.values.push_back(
      diagnostic_value("tracking_match_success_rate",
                       fixed(statistics.last_match_success_rate, 6)));
  status.values.push_back(
      diagnostic_value("tracking_average_match_distance_m",
                       fixed(statistics.last_average_match_distance)));
  status.values.push_back(
      diagnostic_value("tracking_average_velocity_mps",
                       fixed(statistics.last_average_velocity)));
  status.values.push_back(
      diagnostic_value("tracking_last_kalman_update_time_ms",
                       fixed(statistics.last_kalman_update_time_ms)));
  status.values.push_back(diagnostic_value(
      "tracking_average_kalman_update_time_ms", fixed(average_kalman_update)));
  status.values.push_back(diagnostic_value(
      "dynamic_total_tracks",
      std::to_string(statistics.last_dynamic_total_track_count)));
  status.values.push_back(diagnostic_value(
      "dynamic_candidates",
      std::to_string(statistics.last_dynamic_candidate_count)));
  status.values.push_back(diagnostic_value(
      "dynamic_confirmed",
      std::to_string(statistics.last_dynamic_confirmed_count)));
  status.values.push_back(
      diagnostic_value("dynamic_static_count",
                       std::to_string(statistics.last_dynamic_static_count)));
  status.values.push_back(diagnostic_value(
      "dynamic_unclassified_count",
      std::to_string(statistics.last_dynamic_unclassified_count)));
  status.values.push_back(diagnostic_value(
      "dynamic_ratio", fixed(statistics.last_dynamic_ratio, 6)));
  status.values.push_back(
      diagnostic_value("dynamic_average_velocity_mps",
                       fixed(statistics.last_dynamic_average_velocity)));
  status.values.push_back(
      diagnostic_value("dynamic_classification_latency_ms",
                       fixed(statistics.last_dynamic_classification_time_ms)));
  status.values.push_back(
      diagnostic_value("dynamic_average_classification_latency_ms",
                       fixed(average_dynamic_classification)));
  status.values.push_back(diagnostic_value(
      "dynamic_confirmed_count_total",
      std::to_string(statistics.total_dynamic_confirmed_count)));
  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(std::move(array));
}

void DetectorNode::reset_statistics() {
  std::lock_guard<std::mutex> lock(statistics_mutex_);
  statistics_ = RuntimeStatistics{};
}

} // namespace uav_usv_lv_dot_ros2
