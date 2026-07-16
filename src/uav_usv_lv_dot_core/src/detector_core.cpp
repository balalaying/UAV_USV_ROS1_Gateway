#include "uav_usv_lv_dot_core/detector_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "uav_usv_lv_dot_core/lidar_clusterer.hpp"

namespace uav_usv_lv_dot_core {
namespace {

struct VoxelKey {
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  bool operator==(const VoxelKey &other) const noexcept {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelHash {
  std::size_t operator()(const VoxelKey &key) const noexcept {
    std::size_t seed = std::hash<std::int64_t>{}(key.x);
    seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) +
            (seed >> 2U);
    seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) +
            (seed >> 2U);
    return seed;
  }
};

bool finite(const PointXYZI &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

std::vector<PointXYZI> voxel_filter(const std::vector<PointXYZI> &points,
                                    double leaf_size) {
  if (leaf_size <= 0.0 || points.size() < 2) {
    return points;
  }
  std::unordered_set<VoxelKey, VoxelHash> occupied;
  occupied.reserve(points.size());
  std::vector<PointXYZI> result;
  result.reserve(points.size());
  for (const auto &point : points) {
    const VoxelKey key{
        static_cast<std::int64_t>(std::floor(point.x / leaf_size)),
        static_cast<std::int64_t>(std::floor(point.y / leaf_size)),
        static_cast<std::int64_t>(std::floor(point.z / leaf_size))};
    if (occupied.insert(key).second) {
      result.push_back(point);
    }
  }
  return result;
}

PointXYZI transform_point(const PointXYZI &point,
                          const RigidTransform &transform) {
  double qx = transform.rotation_xyzw[0];
  double qy = transform.rotation_xyzw[1];
  double qz = transform.rotation_xyzw[2];
  double qw = transform.rotation_xyzw[3];
  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm <= std::numeric_limits<double>::epsilon()) {
    throw std::invalid_argument(
        "Point-cloud transform contains a zero quaternion");
  }
  qx /= norm;
  qy /= norm;
  qz /= norm;
  qw /= norm;

  const double xx = qx * qx;
  const double yy = qy * qy;
  const double zz = qz * qz;
  const double xy = qx * qy;
  const double xz = qx * qz;
  const double yz = qy * qz;
  const double wx = qw * qx;
  const double wy = qw * qy;
  const double wz = qw * qz;
  PointXYZI output = point;
  output.x = static_cast<float>(
      (1.0 - 2.0 * (yy + zz)) * point.x + 2.0 * (xy - wz) * point.y +
      2.0 * (xz + wy) * point.z + transform.translation[0]);
  output.y = static_cast<float>(
      2.0 * (xy + wz) * point.x + (1.0 - 2.0 * (xx + zz)) * point.y +
      2.0 * (yz - wx) * point.z + transform.translation[1]);
  output.z = static_cast<float>(
      2.0 * (xz - wy) * point.x + 2.0 * (yz + wx) * point.y +
      (1.0 - 2.0 * (xx + yy)) * point.z + transform.translation[2]);
  return output;
}

void validate_configuration(const CoreConfiguration &configuration) {
  if (configuration.input_min_range < 0.0 ||
      configuration.input_max_range < configuration.input_min_range ||
      configuration.input_max_z < configuration.input_min_z ||
      configuration.input_voxel_size < 0.0 ||
      configuration.local_range_x <= 0.0 ||
      configuration.local_range_y <= 0.0 ||
      configuration.roof_height < configuration.ground_height ||
      configuration.downsample_threshold == 0 ||
      configuration.adaptive_voxel_initial_size <= 0.0 ||
      configuration.gaussian_downsample_sigma <= 0.0 ||
      configuration.dbscan_epsilon_squared <= 0.0 ||
      configuration.dbscan_min_points == 0 ||
      std::any_of(configuration.maximum_object_size.begin(),
                  configuration.maximum_object_size.end(),
                  [](double value) { return value <= 0.0; })) {
    throw std::invalid_argument("Invalid LV-DOT core configuration");
  }
}

} // namespace

void DetectorCore::configure(const CoreConfiguration &configuration) {
  validate_configuration(configuration);
  tracker_.configure(configuration.tracking);
  dynamic_classifier_.configure(configuration.dynamic_classification);
  configuration_ = configuration;
  configured_ = true;
  processed_frames_ = 0;
}

void DetectorCore::reset() {
  tracker_.reset();
  dynamic_classifier_.reset();
  configured_ = false;
  processed_frames_ = 0;
}

std::vector<PointXYZI>
DetectorCore::preprocess(const PointCloudFrame &frame) const {
  std::vector<PointXYZI> input_filtered;
  input_filtered.reserve(frame.points.size());
  const double minimum_range_squared =
      configuration_.input_min_range * configuration_.input_min_range;
  const double maximum_range_squared =
      configuration_.input_max_range * configuration_.input_max_range;
  for (const auto &point : frame.points) {
    if (!finite(point)) {
      continue;
    }
    const double range_squared = static_cast<double>(point.x) * point.x +
                                 static_cast<double>(point.y) * point.y +
                                 static_cast<double>(point.z) * point.z;
    if (range_squared < minimum_range_squared ||
        range_squared > maximum_range_squared ||
        point.z < configuration_.input_min_z ||
        point.z > configuration_.input_max_z) {
      continue;
    }
    if (configuration_.crop_self) {
      const auto &bounds = configuration_.self_bounds;
      const bool inside = point.x >= bounds[0] && point.x <= bounds[1] &&
                          point.y >= bounds[2] && point.y <= bounds[3] &&
                          point.z >= bounds[4] && point.z <= bounds[5];
      if (inside) {
        continue;
      }
    }
    input_filtered.push_back(point);
  }
  input_filtered =
      voxel_filter(input_filtered, configuration_.input_voxel_size);

  std::mt19937 random(configuration_.random_seed +
                      static_cast<std::uint32_t>(processed_frames_));
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  std::vector<PointXYZI> transformed;
  transformed.reserve(input_filtered.size());
  const double sigma_squared = configuration_.gaussian_downsample_sigma *
                               configuration_.gaussian_downsample_sigma;
  for (const auto &point : input_filtered) {
    if (point.x < -configuration_.local_range_x ||
        point.x > configuration_.local_range_x ||
        point.y < -configuration_.local_range_y ||
        point.y > configuration_.local_range_y) {
      continue;
    }
    const double radial_squared = static_cast<double>(point.x) * point.x +
                                  static_cast<double>(point.y) * point.y;
    const double keep_probability =
        std::exp(-radial_squared / (2.0 * sigma_squared));
    if (uniform(random) >= keep_probability) {
      continue;
    }
    const auto output = transform_point(point, frame.context.sensor_to_output);
    if (output.z >= configuration_.ground_height &&
        output.z <= configuration_.roof_height) {
      transformed.push_back(output);
    }
  }

  if (transformed.size() > configuration_.downsample_threshold) {
    double leaf_size = configuration_.adaptive_voxel_initial_size;
    std::vector<PointXYZI> downsampled = transformed;
    while (downsampled.size() > configuration_.downsample_threshold) {
      leaf_size *= 1.1;
      downsampled = voxel_filter(transformed, leaf_size);
    }
    return downsampled;
  }
  return transformed;
}

DetectionResult DetectorCore::process(const PointCloudFrame &frame) {
  if (!configured_) {
    throw std::logic_error(
        "DetectorCore must be configured before processing frames");
  }

  DetectionResult result;
  result.stamp_nanoseconds = frame.context.stamp_nanoseconds;
  result.output_frame = frame.context.output_frame;
  result.clustering_statistics.input_point_count = frame.points.size();
  result.clustering_statistics.finite_point_count = static_cast<std::uint64_t>(
      std::count_if(frame.points.begin(), frame.points.end(), finite));

  const auto preprocessing_start = std::chrono::steady_clock::now();
  const auto preprocessed = preprocess(frame);
  const auto clustering_start = std::chrono::steady_clock::now();
  result.clustering_statistics.preprocessed_point_count = preprocessed.size();
  result.clustering_statistics.preprocessing_time_ms =
      std::chrono::duration<double, std::milli>(clustering_start -
                                                preprocessing_start)
          .count();

  LidarClusterer clusterer(LidarClustererConfiguration{
      configuration_.dbscan_epsilon_squared, configuration_.dbscan_min_points,
      configuration_.maximum_object_size});
  auto cluster_result =
      clusterer.cluster(preprocessed, frame.context.stamp_nanoseconds);
  const auto clustering_end = std::chrono::steady_clock::now();
  result.lidar_clusters = std::move(cluster_result.clusters);
  result.clustering_statistics.clustered_point_count =
      cluster_result.clustered_point_count;
  result.clustering_statistics.noise_point_count =
      cluster_result.noise_point_count;
  result.clustering_statistics.clustering_time_ms =
      std::chrono::duration<double, std::milli>(clustering_end -
                                                clustering_start)
          .count();

  auto tracking_result =
      tracker_.update(result.lidar_clusters, frame.context.stamp_nanoseconds,
                      frame.context.sensor_to_output.translation);
  result.tracks = std::move(tracking_result.tracks);
  result.tracking_statistics = tracking_result.statistics;
  auto dynamic_result = dynamic_classifier_.classify(
      result.tracks, frame.context.stamp_nanoseconds);
  result.classified_tracks = std::move(dynamic_result.classified_tracks);
  result.dynamic_tracks = std::move(dynamic_result.dynamic_tracks);
  result.dynamic_statistics = dynamic_result.statistics;

  ++processed_frames_;
  return result;
}

bool DetectorCore::configured() const noexcept { return configured_; }

std::uint64_t DetectorCore::processed_frames() const noexcept {
  return processed_frames_;
}

} // namespace uav_usv_lv_dot_core
