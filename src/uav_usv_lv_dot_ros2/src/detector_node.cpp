#include "uav_usv_lv_dot_ros2/detector_node.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <tf2/time.h>
#include <tf2_ros/transform_listener.h>

#include "uav_usv_lv_dot_core/types.hpp"
#include "uav_usv_lv_dot_ros2/message_conversion.hpp"
#include "uav_usv_lv_dot_ros2/pointcloud_conversion.hpp"

namespace uav_usv_lv_dot_ros2
{
namespace
{

diagnostic_msgs::msg::KeyValue diagnostic_value(
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::string fixed(double value, int precision = 3)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

}  // namespace

DetectorNode::DetectorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lv_dot_detector_node", options)
{
  declare_parameter<std::string>("vehicle_id", "");
  declare_parameter<std::string>("output_frame", "map");
  declare_parameter<double>("tf_timeout_seconds", 0.1);
  declare_parameter<double>("diagnostics_period_seconds", 1.0);
}

DetectorNode::CallbackReturn DetectorNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  vehicle_id_ = get_parameter("vehicle_id").as_string();
  output_frame_ = get_parameter("output_frame").as_string();
  tf_timeout_seconds_ = get_parameter("tf_timeout_seconds").as_double();
  diagnostics_period_seconds_ = get_parameter("diagnostics_period_seconds").as_double();

  if (output_frame_.empty() || tf_timeout_seconds_ <= 0.0 || diagnostics_period_seconds_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "Invalid output frame or timing parameter");
    return CallbackReturn::FAILURE;
  }

  detector_core_.configure(uav_usv_lv_dot_core::CoreConfiguration{});
  reset_statistics();
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  observations_publisher_ = create_publisher<
    uav_usv_interfaces::msg::TrackedObjectArray>("observations", rclcpp::QoS(10).reliable());
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "diagnostics", rclcpp::QoS(10).reliable());
  diagnostics_timer_ = create_wall_timer(
    std::chrono::duration<double>(diagnostics_period_seconds_),
    std::bind(&DetectorNode::publish_diagnostics, this));
  diagnostics_timer_->cancel();

  RCLCPP_INFO(
    get_logger(), "Configured Phase 1 detector for vehicle '%s', output frame '%s'",
    vehicle_id_.c_str(), output_frame_.c_str());
  return CallbackReturn::SUCCESS;
}

DetectorNode::CallbackReturn DetectorNode::on_activate(const rclcpp_lifecycle::State & state)
{
  const auto result = rclcpp_lifecycle::LifecycleNode::on_activate(state);
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }

  cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "points", rclcpp::SensorDataQoS(),
    std::bind(&DetectorNode::cloud_callback, this, std::placeholders::_1));
  diagnostics_timer_->reset();
  RCLCPP_INFO(get_logger(), "Activated; waiting for PointCloud2 on relative topic 'points'");
  return CallbackReturn::SUCCESS;
}

DetectorNode::CallbackReturn DetectorNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  cloud_subscription_.reset();
  if (diagnostics_timer_) {
    diagnostics_timer_->cancel();
  }
  RCLCPP_INFO(get_logger(), "Deactivated");
  return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
}

DetectorNode::CallbackReturn DetectorNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  cloud_subscription_.reset();
  diagnostics_timer_.reset();
  observations_publisher_.reset();
  diagnostics_publisher_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  detector_core_.reset();
  reset_statistics();
  RCLCPP_INFO(get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

DetectorNode::CallbackReturn DetectorNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  cloud_subscription_.reset();
  diagnostics_timer_.reset();
  RCLCPP_INFO(get_logger(), "Shutdown complete");
  return CallbackReturn::SUCCESS;
}

void DetectorNode::cloud_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  const auto processing_start = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    ++statistics_.input_count;
  }

  if (message->header.frame_id.empty()) {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    ++statistics_.malformed_cloud_count;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping cloud with empty frame_id");
    return;
  }

  const rclcpp::Time stamp(message->header.stamp, get_clock()->get_clock_type());
  const auto stamp_nanoseconds = stamp.nanoseconds();
  {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    if (
      statistics_.previous_stamp_nanoseconds != 0 &&
      stamp_nanoseconds <= statistics_.previous_stamp_nanoseconds)
    {
      ++statistics_.nonmonotonic_stamp_count;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dropping non-monotonic cloud timestamp");
      return;
    }
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      output_frame_, message->header.frame_id, message->header.stamp,
      tf2::durationFromSec(tf_timeout_seconds_));
  } catch (const tf2::TransformException & error) {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    ++statistics_.tf_failure_count;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Dropping cloud because timestamped TF failed: %s",
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
  } catch (const std::exception & error) {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    ++statistics_.malformed_cloud_count;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Dropping malformed PointCloud2: %s", error.what());
    return;
  }

  frame.context.stamp_nanoseconds = stamp_nanoseconds;
  frame.context.sensor_frame = message->header.frame_id;
  frame.context.output_frame = output_frame_;
  frame.context.sensor_to_output.translation = {
    transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z};
  frame.context.sensor_to_output.rotation_xyzw = {
    transform.transform.rotation.x,
    transform.transform.rotation.y,
    transform.transform.rotation.z,
    transform.transform.rotation.w};

  {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    if (statistics_.previous_stamp_nanoseconds != 0) {
      statistics_.last_dt_seconds =
        static_cast<double>(stamp_nanoseconds - statistics_.previous_stamp_nanoseconds) * 1e-9;
      statistics_.sum_dt_seconds += statistics_.last_dt_seconds;
      ++statistics_.dt_sample_count;
    } else {
      statistics_.last_dt_seconds = 0.0;
    }
    frame.context.dt_seconds = statistics_.last_dt_seconds;
    statistics_.previous_stamp_nanoseconds = stamp_nanoseconds;
    ++statistics_.accepted_count;
    statistics_.last_point_count = frame.points.size();
    // Bag playback can advance /clock just after delivering a sensor message.
    // Latency is therefore clamped at zero instead of reporting a negative value.
    statistics_.last_input_latency_ms =
      std::max(0.0, (now() - stamp).seconds() * 1000.0);
    statistics_.sum_input_latency_ms += statistics_.last_input_latency_ms;
  }

  auto result = detector_core_.process(frame);
  auto observations = to_ros_message(result);
  if (observations_publisher_->is_activated()) {
    observations_publisher_->publish(std::move(observations));
  }

  const auto processing_end = std::chrono::steady_clock::now();
  const double processing_ms =
    std::chrono::duration<double, std::milli>(processing_end - processing_start).count();
  {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    statistics_.last_processing_time_ms = processing_ms;
    statistics_.sum_processing_time_ms += processing_ms;
  }
}

void DetectorNode::publish_diagnostics()
{
  if (!diagnostics_publisher_ || !diagnostics_publisher_->is_activated()) {
    return;
  }

  RuntimeStatistics statistics;
  {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    statistics = statistics_;
  }

  const auto tf_total = statistics.tf_success_count + statistics.tf_failure_count;
  const double tf_success_rate = tf_total == 0 ? 0.0 :
    static_cast<double>(statistics.tf_success_count) / static_cast<double>(tf_total);
  const double average_dt = statistics.dt_sample_count == 0 ? 0.0 :
    statistics.sum_dt_seconds / static_cast<double>(statistics.dt_sample_count);
  const double input_rate = average_dt > 0.0 ? 1.0 / average_dt : 0.0;
  const double average_processing = statistics.accepted_count == 0 ? 0.0 :
    statistics.sum_processing_time_ms / static_cast<double>(statistics.accepted_count);
  const double average_latency = statistics.accepted_count == 0 ? 0.0 :
    statistics.sum_input_latency_ms / static_cast<double>(statistics.accepted_count);

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name =
    get_node_base_interface()->get_fully_qualified_name() + std::string(": phase1_input");
  status.hardware_id = vehicle_id_.empty() ? "unassigned_vehicle" : vehicle_id_;
  if (statistics.input_count == 0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "ACTIVE: waiting for PointCloud2";
  } else if (tf_success_rate < 0.99) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "ACTIVE: timestamped TF failures detected";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "ACTIVE: Phase 1 input pipeline healthy";
  }

  status.values.push_back(diagnostic_value("node_state", get_current_state().label()));
  status.values.push_back(diagnostic_value("vehicle_id", vehicle_id_));
  status.values.push_back(diagnostic_value("output_frame", output_frame_));
  status.values.push_back(diagnostic_value("input_count", std::to_string(statistics.input_count)));
  status.values.push_back(
    diagnostic_value("accepted_count", std::to_string(statistics.accepted_count)));
  status.values.push_back(diagnostic_value("input_rate_hz", fixed(input_rate)));
  status.values.push_back(diagnostic_value("tf_success_rate", fixed(tf_success_rate, 6)));
  status.values.push_back(
    diagnostic_value("tf_success_count", std::to_string(statistics.tf_success_count)));
  status.values.push_back(
    diagnostic_value("tf_failure_count", std::to_string(statistics.tf_failure_count)));
  status.values.push_back(
    diagnostic_value("malformed_cloud_count", std::to_string(statistics.malformed_cloud_count)));
  status.values.push_back(
    diagnostic_value(
      "nonmonotonic_stamp_count", std::to_string(statistics.nonmonotonic_stamp_count)));
  status.values.push_back(diagnostic_value("last_dt_seconds", fixed(statistics.last_dt_seconds, 6)));
  status.values.push_back(diagnostic_value("average_dt_seconds", fixed(average_dt, 6)));
  status.values.push_back(
    diagnostic_value("last_processing_time_ms", fixed(statistics.last_processing_time_ms)));
  status.values.push_back(
    diagnostic_value("average_processing_time_ms", fixed(average_processing)));
  status.values.push_back(
    diagnostic_value("last_input_latency_ms", fixed(statistics.last_input_latency_ms)));
  status.values.push_back(
    diagnostic_value("average_input_latency_ms", fixed(average_latency)));
  status.values.push_back(
    diagnostic_value("last_point_count", std::to_string(statistics.last_point_count)));
  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(std::move(array));
}

void DetectorNode::reset_statistics()
{
  std::lock_guard<std::mutex> lock(statistics_mutex_);
  statistics_ = RuntimeStatistics{};
}

}  // namespace uav_usv_lv_dot_ros2
