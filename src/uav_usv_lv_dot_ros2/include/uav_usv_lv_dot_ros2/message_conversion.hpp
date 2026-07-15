#ifndef UAV_USV_LV_DOT_ROS2__MESSAGE_CONVERSION_HPP_
#define UAV_USV_LV_DOT_ROS2__MESSAGE_CONVERSION_HPP_

#include <uav_usv_interfaces/msg/tracked_object_array.hpp>

#include "uav_usv_lv_dot_core/types.hpp"

namespace uav_usv_lv_dot_ros2
{

uav_usv_interfaces::msg::TrackedObjectArray to_ros_message(
  const uav_usv_lv_dot_core::DetectionResult & result);

}  // namespace uav_usv_lv_dot_ros2

#endif  // UAV_USV_LV_DOT_ROS2__MESSAGE_CONVERSION_HPP_
