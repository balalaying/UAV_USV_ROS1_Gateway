// Algorithm behavior is derived from LV-DOT dbscan.cpp at commit
// 449bf2c960a26b067b235d82f6e0aac65fc05a6b (MIT License).
#include "uav_usv_lv_dot_core/dbscan.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace uav_usv_lv_dot_core {

Dbscan::Dbscan(std::uint32_t minimum_points, double epsilon_squared)
    : minimum_points_(minimum_points), epsilon_squared_(epsilon_squared) {
  if (minimum_points_ == 0 || epsilon_squared_ <= 0.0) {
    throw std::invalid_argument(
        "DBSCAN minimum points and epsilon must be positive");
  }
}

std::vector<DbscanPoint>
Dbscan::run(const std::vector<PointXYZI> &input) const {
  std::vector<DbscanPoint> points;
  points.reserve(input.size());
  for (const auto &point : input) {
    points.push_back(DbscanPoint{point, kUnclassified});
  }

  std::int32_t cluster_id = 1;
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (points[index].cluster_id == kUnclassified &&
        expand_cluster(points, index, cluster_id)) {
      ++cluster_id;
    }
  }
  return points;
}

std::vector<std::size_t>
Dbscan::neighbors(const std::vector<DbscanPoint> &points,
                  const DbscanPoint &query) const {
  std::vector<std::size_t> result;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const double dx =
        static_cast<double>(query.point.x) - points[index].point.x;
    const double dy =
        static_cast<double>(query.point.y) - points[index].point.y;
    const double dz =
        static_cast<double>(query.point.z) - points[index].point.z;
    if (dx * dx + dy * dy + dz * dz <= epsilon_squared_) {
      result.push_back(index);
    }
  }
  return result;
}

bool Dbscan::expand_cluster(std::vector<DbscanPoint> &points,
                            std::size_t point_index,
                            std::int32_t cluster_id) const {
  auto seeds = neighbors(points, points.at(point_index));
  if (seeds.size() < minimum_points_) {
    points[point_index].cluster_id = kNoise;
    return false;
  }

  for (const auto seed : seeds) {
    points[seed].cluster_id = cluster_id;
  }
  const auto core = std::find(seeds.begin(), seeds.end(), point_index);
  if (core != seeds.end()) {
    seeds.erase(core);
  }

  for (std::size_t seed_index = 0; seed_index < seeds.size(); ++seed_index) {
    const auto adjacent = neighbors(points, points.at(seeds[seed_index]));
    if (adjacent.size() < minimum_points_) {
      continue;
    }
    for (const auto neighbor_index : adjacent) {
      auto &neighbor = points[neighbor_index];
      if (neighbor.cluster_id != kUnclassified &&
          neighbor.cluster_id != kNoise) {
        continue;
      }
      if (neighbor.cluster_id == kUnclassified) {
        seeds.push_back(neighbor_index);
      }
      neighbor.cluster_id = cluster_id;
    }
  }
  return true;
}

} // namespace uav_usv_lv_dot_core
