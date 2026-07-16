#ifndef UAV_USV_LV_DOT_ROS2__DETECTOR_NODE_HPP_
#define UAV_USV_LV_DOT_ROS2__DETECTOR_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <uav_usv_interfaces/msg/tracked_object_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "uav_usv_lv_dot_core/detector_core.hpp"

namespace uav_usv_lv_dot_ros2 {

class DetectorNode : public rclcpp_lifecycle::LifecycleNode {
public:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit DetectorNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

private:
  struct RuntimeStatistics {
    std::uint64_t input_count{0};
    std::uint64_t accepted_count{0};
    std::uint64_t tf_success_count{0};
    std::uint64_t tf_failure_count{0};
    std::uint64_t malformed_cloud_count{0};
    std::uint64_t nonmonotonic_stamp_count{0};
    std::uint64_t dt_sample_count{0};
    std::int64_t previous_stamp_nanoseconds{0};
    double last_dt_seconds{0.0};
    double sum_dt_seconds{0.0};
    double last_processing_time_ms{0.0};
    double sum_processing_time_ms{0.0};
    double last_input_latency_ms{0.0};
    double sum_input_latency_ms{0.0};
    std::uint64_t last_input_point_count{0};
    std::uint64_t last_finite_point_count{0};
    std::uint64_t last_preprocessed_point_count{0};
    std::uint64_t last_clustered_point_count{0};
    std::uint64_t last_noise_point_count{0};
    std::uint64_t last_cluster_count{0};
    std::uint64_t sum_cluster_count{0};
    double last_preprocessing_time_ms{0.0};
    double sum_preprocessing_time_ms{0.0};
    double last_clustering_time_ms{0.0};
    double sum_clustering_time_ms{0.0};
    std::uint64_t last_detection_count{0};
    std::uint64_t last_matched_count{0};
    std::uint64_t last_created_track_count{0};
    std::uint64_t last_removed_track_count{0};
    std::uint64_t last_active_track_count{0};
    std::uint64_t last_confirmed_track_count{0};
    std::uint64_t last_lost_track_count{0};
    std::uint64_t total_detection_count{0};
    std::uint64_t total_matched_count{0};
    std::uint64_t total_created_track_count{0};
    std::uint64_t total_removed_track_count{0};
    std::uint64_t total_id_switch_count{0};
    double total_match_distance{0.0};
    double last_match_success_rate{0.0};
    double last_average_match_distance{0.0};
    double last_average_velocity{0.0};
    double last_kalman_update_time_ms{0.0};
    double sum_kalman_update_time_ms{0.0};
    std::uint64_t last_dynamic_total_track_count{0};
    std::uint64_t last_dynamic_static_count{0};
    std::uint64_t last_dynamic_candidate_count{0};
    std::uint64_t last_dynamic_confirmed_count{0};
    std::uint64_t last_dynamic_unclassified_count{0};
    std::uint64_t total_dynamic_confirmed_count{0};
    double last_dynamic_ratio{0.0};
    double last_dynamic_average_velocity{0.0};
    double last_dynamic_classification_time_ms{0.0};
    double sum_dynamic_classification_time_ms{0.0};
  };

  void cloud_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  void publish_diagnostics();
  void reset_statistics();

  std::string vehicle_id_;
  std::string output_frame_;
  double tf_timeout_seconds_{0.1};
  double diagnostics_period_seconds_{1.0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      cloud_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<
      uav_usv_interfaces::msg::TrackedObjectArray>::SharedPtr
      observations_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<
      uav_usv_interfaces::msg::TrackedObjectArray>::SharedPtr tracks_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<
      uav_usv_interfaces::msg::TrackedObjectArray>::SharedPtr
      dynamic_tracks_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<
      diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<
      visualization_msgs::msg::MarkerArray>::SharedPtr lidar_bboxes_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  uav_usv_lv_dot_core::DetectorCore detector_core_;
  RuntimeStatistics statistics_;
  std::mutex statistics_mutex_;
};

} // namespace uav_usv_lv_dot_ros2

#endif // UAV_USV_LV_DOT_ROS2__DETECTOR_NODE_HPP_
