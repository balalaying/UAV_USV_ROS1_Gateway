// Bounding-box behavior is derived from LV-DOT lidarDetector.cpp at commit
// 449bf2c960a26b067b235d82f6e0aac65fc05a6b (MIT License).
#include "uav_usv_lv_dot_core/lidar_clusterer.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

#include "uav_usv_lv_dot_core/dbscan.hpp"

namespace uav_usv_lv_dot_core {

LidarClusterer::LidarClusterer(LidarClustererConfiguration configuration)
    : configuration_(std::move(configuration)) {
  if (configuration_.dbscan_epsilon_squared <= 0.0 ||
      configuration_.dbscan_min_points == 0) {
    throw std::invalid_argument("Invalid LiDAR clusterer configuration");
  }
}

LidarClustererResult
LidarClusterer::cluster(const std::vector<PointXYZI> &points,
                        std::int64_t stamp_nanoseconds) const {
  LidarClustererResult result;
  if (points.empty()) {
    return result;
  }

  Dbscan dbscan(configuration_.dbscan_min_points,
                configuration_.dbscan_epsilon_squared);
  const auto classified = dbscan.run(points);
  std::int32_t cluster_count = 0;
  for (const auto &point : classified) {
    cluster_count = std::max(cluster_count, point.cluster_id);
    if (point.cluster_id == kNoise || point.cluster_id == kUnclassified) {
      ++result.noise_point_count;
    }
  }

  struct Accumulator {
    std::array<double, 3> sum{{0.0, 0.0, 0.0}};
    std::array<double, 3> minimum{{std::numeric_limits<double>::max(),
                                   std::numeric_limits<double>::max(),
                                   std::numeric_limits<double>::max()}};
    std::array<double, 3> maximum{{std::numeric_limits<double>::lowest(),
                                   std::numeric_limits<double>::lowest(),
                                   std::numeric_limits<double>::lowest()}};
    std::uint64_t count{0};
  };
  std::vector<Accumulator> accumulators(
      static_cast<std::size_t>(cluster_count));
  for (const auto &classified_point : classified) {
    if (classified_point.cluster_id <= 0) {
      continue;
    }
    auto &accumulator =
        accumulators[static_cast<std::size_t>(classified_point.cluster_id - 1)];
    const std::array<double, 3> coordinate{{classified_point.point.x,
                                            classified_point.point.y,
                                            classified_point.point.z}};
    for (std::size_t axis = 0; axis < coordinate.size(); ++axis) {
      accumulator.sum[axis] += coordinate[axis];
      accumulator.minimum[axis] =
          std::min(accumulator.minimum[axis], coordinate[axis]);
      accumulator.maximum[axis] =
          std::max(accumulator.maximum[axis], coordinate[axis]);
    }
    ++accumulator.count;
  }

  for (std::size_t index = 0; index < accumulators.size(); ++index) {
    const auto &accumulator = accumulators[index];
    if (accumulator.count == 0) {
      continue;
    }
    LidarCluster cluster;
    cluster.cluster_id = static_cast<std::int32_t>(index + 1);
    cluster.point_count = accumulator.count;
    cluster.stamp_nanoseconds = stamp_nanoseconds;
    bool oversized = false;
    for (std::size_t axis = 0; axis < cluster.center.size(); ++axis) {
      cluster.center[axis] =
          accumulator.sum[axis] / static_cast<double>(accumulator.count);
      cluster.dimensions[axis] =
          accumulator.maximum[axis] - accumulator.minimum[axis];
      oversized = oversized || cluster.dimensions[axis] >
                                   configuration_.maximum_object_size[axis];
    }
    if (!oversized) {
      result.clustered_point_count += cluster.point_count;
      result.clusters.push_back(std::move(cluster));
    } else {
      result.noise_point_count += cluster.point_count;
    }
  }
  return result;
}

} // namespace uav_usv_lv_dot_core
