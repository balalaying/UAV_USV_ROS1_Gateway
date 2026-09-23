#ifndef UAV_USV_LV_DOT_ROS2__POINTCLOUD_CONVERSION_HPP_
#define UAV_USV_LV_DOT_ROS2__POINTCLOUD_CONVERSION_HPP_

#include <sensor_msgs/PointCloud2.h>

#include "uav_usv_lv_dot_core/types.hpp"

namespace uav_usv_lv_dot
{

void convert_point_cloud(
  const sensor_msgs::PointCloud2 & message,
  uav_usv_lv_dot_core::PointCloudFrame & frame);

}  // namespace uav_usv_lv_dot

#endif  // UAV_USV_LV_DOT_ROS2__POINTCLOUD_CONVERSION_HPP_
