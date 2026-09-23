#ifndef UAV_USV_LV_DOT_ROS2__CLUSTER_MARKER_CONVERSION_HPP_
#define UAV_USV_LV_DOT_ROS2__CLUSTER_MARKER_CONVERSION_HPP_

#include <visualization_msgs/MarkerArray.h>

#include "uav_usv_lv_dot_core/types.hpp"

namespace uav_usv_lv_dot {

visualization_msgs::MarkerArray
to_lidar_bbox_markers(const uav_usv_lv_dot_core::DetectionResult &result);

} // namespace uav_usv_lv_dot

#endif // UAV_USV_LV_DOT_ROS2__CLUSTER_MARKER_CONVERSION_HPP_
