#include "uav_usv_lv_dot_ros2/cluster_marker_conversion.hpp"

#include <array>
#include <cstdint>
#include <string>

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace uav_usv_lv_dot_ros2 {
namespace {

builtin_interfaces::msg::Time to_stamp(std::int64_t nanoseconds) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}

geometry_msgs::msg::Point point(double x, double y, double z) {
  geometry_msgs::msg::Point result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}

} // namespace

visualization_msgs::msg::MarkerArray
to_lidar_bbox_markers(const uav_usv_lv_dot_core::DetectionResult &result) {
  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = result.output_frame;
  clear.header.stamp = to_stamp(result.stamp_nanoseconds);
  clear.ns = "lv_dot_ros2/lidar_bboxes";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear);

  constexpr std::array<std::array<std::size_t, 2>, 12> edges{{{{0, 1}},
                                                              {{1, 2}},
                                                              {{2, 3}},
                                                              {{3, 0}},
                                                              {{4, 5}},
                                                              {{5, 6}},
                                                              {{6, 7}},
                                                              {{7, 4}},
                                                              {{0, 4}},
                                                              {{1, 5}},
                                                              {{2, 6}},
                                                              {{3, 7}}}};
  for (const auto &cluster : result.lidar_clusters) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = result.output_frame;
    marker.header.stamp = to_stamp(result.stamp_nanoseconds);
    marker.ns = "lv_dot_ros2/lidar_bboxes";
    marker.id = cluster.cluster_id;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = cluster.center[0];
    marker.pose.position.y = cluster.center[1];
    marker.pose.position.z = cluster.center[2];
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.06;
    marker.color.r = 0.1F;
    marker.color.g = 0.85F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 200000000U;
    marker.text =
        "cluster_id=" + std::to_string(cluster.cluster_id) +
        ";points=" + std::to_string(cluster.point_count) + ";preprocess_ms=" +
        std::to_string(result.clustering_statistics.preprocessing_time_ms) +
        ";cluster_ms=" +
        std::to_string(result.clustering_statistics.clustering_time_ms);

    const double half_x = 0.5 * cluster.dimensions[0];
    const double half_y = 0.5 * cluster.dimensions[1];
    const double half_z = 0.5 * cluster.dimensions[2];
    const std::array<geometry_msgs::msg::Point, 8> corners{
        {point(-half_x, -half_y, -half_z), point(-half_x, half_y, -half_z),
         point(half_x, half_y, -half_z), point(half_x, -half_y, -half_z),
         point(-half_x, -half_y, half_z), point(-half_x, half_y, half_z),
         point(half_x, half_y, half_z), point(half_x, -half_y, half_z)}};
    marker.points.reserve(edges.size() * 2U);
    for (const auto &edge : edges) {
      marker.points.push_back(corners[edge[0]]);
      marker.points.push_back(corners[edge[1]]);
    }
    array.markers.push_back(std::move(marker));
  }
  return array;
}

} // namespace uav_usv_lv_dot_ros2
