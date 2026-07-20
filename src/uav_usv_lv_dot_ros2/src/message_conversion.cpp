#include "uav_usv_lv_dot_ros2/message_conversion.hpp"

#include <algorithm>
#include <cstdint>
#include <utility>

#include <builtin_interfaces/msg/time.hpp>

namespace uav_usv_lv_dot_ros2
{
namespace
{

builtin_interfaces::msg::Time to_time(std::int64_t nanoseconds)
{
  builtin_interfaces::msg::Time stamp;
  if (nanoseconds <= 0) {
    return stamp;
  }
  stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}

}  // namespace

uav_usv_interfaces::msg::TrackedObjectArray to_ros_message(
  const uav_usv_lv_dot_core::DetectionResult & result)
{
  uav_usv_interfaces::msg::TrackedObjectArray message;
  message.header.stamp = to_time(result.stamp_nanoseconds);
  message.header.frame_id = result.output_frame;
  message.objects.reserve(result.tracks.size());

  for (const auto & track : result.tracks) {
    uav_usv_interfaces::msg::TrackedObject object;
    std::copy(track.uuid.begin(), track.uuid.end(), object.uuid.uuid.begin());
    object.track_id = track.track_id;
    object.first_seen = to_time(track.first_seen_nanoseconds);
    object.last_update = to_time(track.last_update_nanoseconds);
    object.source_mask = static_cast<std::uint8_t>(track.source);
    object.classification = static_cast<std::uint8_t>(track.classification);
    object.class_name =
      object.classification == 1U ? "vessel" : "unknown";
    object.class_confidence =
      object.classification == 0U ? 0.0F : track.confidence;
    object.sensor_source = "lidar";
    object.pose.pose.position.x = track.position[0];
    object.pose.pose.position.y = track.position[1];
    object.pose.pose.position.z = track.position[2];
    object.pose.pose.orientation.x = track.orientation_xyzw[0];
    object.pose.pose.orientation.y = track.orientation_xyzw[1];
    object.pose.pose.orientation.z = track.orientation_xyzw[2];
    object.pose.pose.orientation.w = track.orientation_xyzw[3];
    std::copy(
      track.pose_covariance.begin(), track.pose_covariance.end(),
      object.pose.covariance.begin());
    object.twist.twist.linear.x = track.linear_velocity[0];
    object.twist.twist.linear.y = track.linear_velocity[1];
    object.twist.twist.linear.z = track.linear_velocity[2];
    object.twist.twist.angular.x = track.angular_velocity[0];
    object.twist.twist.angular.y = track.angular_velocity[1];
    object.twist.twist.angular.z = track.angular_velocity[2];
    std::copy(
      track.twist_covariance.begin(), track.twist_covariance.end(),
      object.twist.covariance.begin());
    object.dimensions.x = track.dimensions[0];
    object.dimensions.y = track.dimensions[1];
    object.dimensions.z = track.dimensions[2];
    object.confidence = track.confidence;
    object.mmsi = track.mmsi;
    message.objects.push_back(std::move(object));
  }
  return message;
}

}  // namespace uav_usv_lv_dot_ros2
