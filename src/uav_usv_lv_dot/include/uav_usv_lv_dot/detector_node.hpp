#ifndef UAV_USV_LV_DOT_ROS2__DETECTOR_NODE_HPP_
#define UAV_USV_LV_DOT_ROS2__DETECTOR_NODE_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <uav_usv_interfaces/TrackedObjectArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "uav_usv_lv_dot_core/detector_core.hpp"

namespace uav_usv_lv_dot {

class DetectorNode {
public:
  DetectorNode();

private:
  struct RuntimeStatistics {
    std::uint64_t input_count{0};
    std::uint64_t accepted_count{0};
    std::uint64_t tf_success_count{0};
    std::uint64_t tf_failure_count{0};
    std::uint64_t malformed_cloud_count{0};
    std::uint64_t nonmonotonic_stamp_count{0};
    std::int64_t previous_stamp_nanoseconds{0};
    double last_processing_time_ms{0.0};
    std::uint64_t last_input_point_count{0};
    std::uint64_t last_detection_count{0};
    std::uint64_t last_dynamic_count{0};
  };

  void configure_core();
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &message);
  void publish_diagnostics(const ros::TimerEvent &event);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::string vehicle_id_;
  std::string output_frame_;
  double tf_timeout_seconds_{0.1};
  bool allow_latest_tf_fallback_{true};
  double diagnostics_period_seconds_{1.0};

  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  ros::Subscriber cloud_subscription_;
  ros::Publisher observations_publisher_;
  ros::Publisher tracks_publisher_;
  ros::Publisher dynamic_tracks_publisher_;
  ros::Publisher diagnostics_publisher_;
  ros::Publisher lidar_bboxes_publisher_;
  ros::Timer diagnostics_timer_;

  uav_usv_lv_dot_core::DetectorCore detector_core_;
  RuntimeStatistics statistics_;
  std::mutex statistics_mutex_;
};

} // namespace uav_usv_lv_dot

#endif // UAV_USV_LV_DOT_ROS2__DETECTOR_NODE_HPP_
