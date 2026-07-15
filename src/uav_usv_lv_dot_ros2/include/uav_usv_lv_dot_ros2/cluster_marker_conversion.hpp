#ifndef UAV_USV_LV_DOT_ROS2__CLUSTER_MARKER_CONVERSION_HPP_
#define UAV_USV_LV_DOT_ROS2__CLUSTER_MARKER_CONVERSION_HPP_

#include <visualization_msgs/msg/marker_array.hpp>

#include "uav_usv_lv_dot_core/types.hpp"

namespace uav_usv_lv_dot_ros2 {

visualization_msgs::msg::MarkerArray
to_lidar_bbox_markers(const uav_usv_lv_dot_core::DetectionResult &result);

} // namespace uav_usv_lv_dot_ros2

#endif // UAV_USV_LV_DOT_ROS2__CLUSTER_MARKER_CONVERSION_HPP_
