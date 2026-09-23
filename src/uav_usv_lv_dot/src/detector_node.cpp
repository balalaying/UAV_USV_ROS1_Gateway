#include "uav_usv_lv_dot/detector_node.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/exceptions.h>

#include "uav_usv_lv_dot/cluster_marker_conversion.hpp"
#include "uav_usv_lv_dot/message_conversion.hpp"
#include "uav_usv_lv_dot/pointcloud_conversion.hpp"

namespace uav_usv_lv_dot {
namespace {

diagnostic_msgs::KeyValue value(const std::string &key,
                                const std::string &text) {
  diagnostic_msgs::KeyValue item;
  item.key = key;
  item.value = text;
  return item;
}

template <typename T>
T parameter(ros::NodeHandle &node, const std::string &name,
            const T &default_value) {
  T result = default_value;
  node.param(name, result, default_value);
  return result;
}

std::vector<double> vector_parameter(ros::NodeHandle &node,
                                     const std::string &name,
                                     const std::vector<double> &fallback) {
  std::vector<double> result;
  if (!node.getParam(name, result)) {
    result = fallback;
  }
  return result;
}

} // namespace

DetectorNode::DetectorNode() : private_nh_("~") {
  vehicle_id_ = parameter<std::string>(private_nh_, "vehicle_id", "");
  output_frame_ =
      parameter<std::string>(private_nh_, "output_frame", "map");
  tf_timeout_seconds_ =
      parameter<double>(private_nh_, "tf_timeout_seconds", 0.1);
  allow_latest_tf_fallback_ =
      parameter<bool>(private_nh_, "allow_latest_tf_fallback", true);
  diagnostics_period_seconds_ =
      parameter<double>(private_nh_, "diagnostics_period_seconds", 1.0);
  configure_core();

  tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));
  observations_publisher_ =
      private_nh_.advertise<uav_usv_interfaces::TrackedObjectArray>(
          "observations", 10);
  tracks_publisher_ =
      private_nh_.advertise<uav_usv_interfaces::TrackedObjectArray>(
          "tracks", 10);
  dynamic_tracks_publisher_ =
      private_nh_.advertise<uav_usv_interfaces::TrackedObjectArray>(
          "dynamic_tracks", 10);
  diagnostics_publisher_ =
      private_nh_.advertise<diagnostic_msgs::DiagnosticArray>(
          "diagnostics", 10);
  lidar_bboxes_publisher_ =
      private_nh_.advertise<visualization_msgs::MarkerArray>(
          "diagnostics/lidar_bboxes", 5);
  cloud_subscription_ = private_nh_.subscribe(
      "points", 2, &DetectorNode::cloud_callback, this);
  diagnostics_timer_ = private_nh_.createTimer(
      ros::Duration(diagnostics_period_seconds_),
      &DetectorNode::publish_diagnostics, this);
  ROS_INFO("LV-DOT ROS 1 detector ready for vehicle '%s', frame '%s'",
           vehicle_id_.c_str(), output_frame_.c_str());
}

void DetectorNode::configure_core() {
  uav_usv_lv_dot_core::CoreConfiguration config;
  config.input_min_range =
      parameter<double>(private_nh_, "input_min_range", 0.5);
  config.input_max_range =
      parameter<double>(private_nh_, "input_max_range", 20.0);
  config.input_min_z = parameter<double>(private_nh_, "input_min_z", -1.75);
  config.input_max_z = parameter<double>(private_nh_, "input_max_z", 4.0);
  config.input_voxel_size =
      parameter<double>(private_nh_, "input_voxel_size", 0.04);
  config.crop_self = parameter<bool>(private_nh_, "crop_self", true);
  const auto self_bounds = vector_parameter(
      private_nh_, "self_bounds", {-4.3, 2.5, -1.8, 1.8, -2.4, 0.35});
  const auto max_size = vector_parameter(
      private_nh_, "maximum_object_size", {30.0, 15.0, 12.0});
  const auto feature_weights = vector_parameter(
      private_nh_, "tracking_feature_weights",
      {3.0, 3.0, 0.1, 0.5, 0.5, 0.05, 0.0, 0.0, 0.0});
  const auto kalman = vector_parameter(
      private_nh_, "kalman_filter_parameters",
      {0.25, 0.01, 0.05, 0.05, 0.04, 0.3, 0.6});
  if (self_bounds.size() != 6 || max_size.size() != 3 ||
      feature_weights.size() != 9 || kalman.size() != 7) {
    throw std::runtime_error("invalid LV-DOT vector parameter length");
  }
  std::copy(self_bounds.begin(), self_bounds.end(), config.self_bounds.begin());
  config.local_range_x =
      parameter<double>(private_nh_, "local_range_x", 10.0);
  config.local_range_y =
      parameter<double>(private_nh_, "local_range_y", 10.0);
  config.ground_height =
      parameter<double>(private_nh_, "ground_height", 0.22);
  config.roof_height = parameter<double>(private_nh_, "roof_height", 6.0);
  config.downsample_threshold = static_cast<std::uint64_t>(std::max(
      1, parameter<int>(private_nh_, "downsample_threshold", 12000)));
  config.adaptive_voxel_initial_size =
      parameter<double>(private_nh_, "adaptive_voxel_initial_size", 0.1);
  config.gaussian_downsample_sigma =
      parameter<double>(private_nh_, "gaussian_downsample_sigma", 16.0);
  config.random_seed = static_cast<std::uint32_t>(std::max(
      0, parameter<int>(private_nh_, "random_seed", 1)));
  const double epsilon =
      parameter<double>(private_nh_, "lidar_dbscan_epsilon", 0.65);
  config.dbscan_epsilon_squared = epsilon;
  config.dbscan_min_points = static_cast<std::uint32_t>(std::max(
      1, parameter<int>(private_nh_, "lidar_dbscan_min_points", 3)));
  std::copy(max_size.begin(), max_size.end(),
            config.maximum_object_size.begin());

  auto &tracking = config.tracking;
  tracking.max_match_range =
      parameter<double>(private_nh_, "tracking_max_match_range", 2.0);
  tracking.max_size_difference =
      parameter<double>(private_nh_, "tracking_max_size_difference", 8.0);
  std::copy(feature_weights.begin(), feature_weights.end(),
            tracking.feature_weights.begin());
  tracking.history_size = parameter<int>(
      private_nh_, "tracking_history_size", 100);
  tracking.fix_size_history_threshold = parameter<int>(
      private_nh_, "tracking_fix_size_history_threshold", 10);
  tracking.fix_size_dimension_threshold = parameter<double>(
      private_nh_, "tracking_fix_size_dimension_threshold", 0.4);
  tracking.kalman_averaging_frames = parameter<int>(
      private_nh_, "tracking_kalman_averaging_frames", 3);
  tracking.confirmation_hits = parameter<int>(
      private_nh_, "tracking_confirmation_hits", 3);
  tracking.maximum_missed_frames = parameter<int>(
      private_nh_, "tracking_maximum_missed_frames", 5);
  tracking.kalman_noise.initial_covariance = kalman[0];
  tracking.kalman_noise.process_position = kalman[1];
  tracking.kalman_noise.process_velocity = kalman[2];
  tracking.kalman_noise.process_acceleration = kalman[3];
  tracking.kalman_noise.measurement_position = kalman[4];
  tracking.kalman_noise.measurement_velocity = kalman[5];
  tracking.kalman_noise.measurement_acceleration = kalman[6];

  auto &dynamic = config.dynamic_classification;
  dynamic.frame_skip =
      parameter<int>(private_nh_, "frame_skip", 2);
  dynamic.velocity_threshold = parameter<double>(
      private_nh_, "dynamic_velocity_threshold", 0.05);
  dynamic.voting_threshold = parameter<double>(
      private_nh_, "dynamic_voting_threshold", 0.15);
  dynamic.force_dynamic_frames = parameter<int>(
      private_nh_, "frames_force_dynamic", 3);
  dynamic.force_dynamic_check_range = parameter<int>(
      private_nh_, "frames_force_dynamic_check_range", 12);
  dynamic.consistency_threshold = parameter<int>(
      private_nh_, "dynamic_consistency_threshold", 2);
  dynamic.history_size = parameter<int>(
      private_nh_, "dynamic_history_size", 100);
  detector_core_.configure(config);
}

void DetectorNode::cloud_callback(
    const sensor_msgs::PointCloud2ConstPtr &message) {
  const auto started = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    ++statistics_.input_count;
  }
  if (message->header.frame_id.empty()) {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    ++statistics_.malformed_cloud_count;
    ROS_WARN_THROTTLE(5.0, "Dropping cloud with empty frame_id");
    return;
  }
  const std::int64_t stamp_ns = message->header.stamp.toNSec();
  {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    if (statistics_.previous_stamp_nanoseconds != 0 &&
        stamp_ns <= statistics_.previous_stamp_nanoseconds) {
      ++statistics_.nonmonotonic_stamp_count;
      ROS_WARN_THROTTLE(5.0, "Dropping non-monotonic cloud timestamp");
      return;
    }
  }

  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
        output_frame_, message->header.frame_id, message->header.stamp,
        ros::Duration(tf_timeout_seconds_));
  } catch (const tf2::TransformException &error) {
    if (!allow_latest_tf_fallback_) {
      std::lock_guard<std::mutex> lock(statistics_mutex_);
      ++statistics_.tf_failure_count;
      ROS_WARN_THROTTLE(5.0, "Dropping cloud: %s", error.what());
      return;
    }
    try {
      transform = tf_buffer_.lookupTransform(
          output_frame_, message->header.frame_id, ros::Time(0),
          ros::Duration(tf_timeout_seconds_));
      ROS_WARN_THROTTLE(5.0, "Using latest TF after timestamp lookup failed");
    } catch (const tf2::TransformException &latest_error) {
      std::lock_guard<std::mutex> lock(statistics_mutex_);
      ++statistics_.tf_failure_count;
      ROS_WARN_THROTTLE(5.0, "Dropping cloud: %s", latest_error.what());
      return;
    }
  }

  uav_usv_lv_dot_core::PointCloudFrame frame;
  try {
    convert_point_cloud(*message, frame);
  } catch (const std::exception &error) {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    ++statistics_.malformed_cloud_count;
    ROS_WARN_THROTTLE(5.0, "Dropping malformed PointCloud2: %s",
                      error.what());
    return;
  }
  frame.context.stamp_nanoseconds = stamp_ns;
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
    frame.context.dt_seconds =
        statistics_.previous_stamp_nanoseconds == 0
            ? 0.0
            : static_cast<double>(
                  stamp_ns - statistics_.previous_stamp_nanoseconds) *
                  1.0e-9;
    statistics_.previous_stamp_nanoseconds = stamp_ns;
    ++statistics_.accepted_count;
    ++statistics_.tf_success_count;
    statistics_.last_input_point_count = frame.points.size();
  }

  auto result = detector_core_.process(frame);
  lidar_bboxes_publisher_.publish(to_lidar_bbox_markers(result));
  tracks_publisher_.publish(to_ros_message(result));
  auto dynamic_result = result;
  dynamic_result.tracks = result.dynamic_tracks;
  dynamic_tracks_publisher_.publish(to_ros_message(dynamic_result));
  auto observations = result;
  observations.tracks.clear();
  observations_publisher_.publish(to_ros_message(observations));

  const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    statistics_.last_processing_time_ms = elapsed_ms;
    statistics_.last_detection_count = result.tracks.size();
    statistics_.last_dynamic_count = result.dynamic_tracks.size();
  }
}

void DetectorNode::publish_diagnostics(const ros::TimerEvent &) {
  RuntimeStatistics statistics;
  {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    statistics = statistics_;
  }
  diagnostic_msgs::DiagnosticArray array;
  array.header.stamp = ros::Time::now();
  diagnostic_msgs::DiagnosticStatus status;
  status.name = ros::this_node::getName() + ": lv_dot_ros1";
  status.hardware_id = vehicle_id_.empty() ? "unassigned_vehicle" : vehicle_id_;
  status.level = statistics.input_count == 0
                     ? diagnostic_msgs::DiagnosticStatus::WARN
                     : diagnostic_msgs::DiagnosticStatus::OK;
  status.message = statistics.input_count == 0
                       ? "waiting for PointCloud2"
                       : "ROS 1 detector healthy";
  status.values.push_back(value("input_count",
                                std::to_string(statistics.input_count)));
  status.values.push_back(value("accepted_count",
                                std::to_string(statistics.accepted_count)));
  status.values.push_back(value("tf_success_count",
                                std::to_string(statistics.tf_success_count)));
  status.values.push_back(value("tf_failure_count",
                                std::to_string(statistics.tf_failure_count)));
  status.values.push_back(value("last_input_point_count",
                                std::to_string(statistics.last_input_point_count)));
  status.values.push_back(value("last_detection_count",
                                std::to_string(statistics.last_detection_count)));
  status.values.push_back(value("last_dynamic_count",
                                std::to_string(statistics.last_dynamic_count)));
  status.values.push_back(value("last_processing_time_ms",
                                std::to_string(statistics.last_processing_time_ms)));
  array.status.push_back(status);
  diagnostics_publisher_.publish(array);
}

} // namespace uav_usv_lv_dot
