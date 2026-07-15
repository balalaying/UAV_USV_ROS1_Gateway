#ifndef UAV_USV_LV_DOT_CORE__LIDAR_CLUSTERER_HPP_
#define UAV_USV_LV_DOT_CORE__LIDAR_CLUSTERER_HPP_

#include <array>
#include <cstdint>
#include <vector>

#include "uav_usv_lv_dot_core/types.hpp"

namespace uav_usv_lv_dot_core {

struct LidarClustererConfiguration {
  double dbscan_epsilon_squared{0.65};
  std::uint32_t dbscan_min_points{3};
  std::array<double, 3> maximum_object_size{{30.0, 15.0, 12.0}};
};

struct LidarClustererResult {
  std::vector<LidarCluster> clusters;
  std::uint64_t clustered_point_count{0};
  std::uint64_t noise_point_count{0};
};

class LidarClusterer {
public:
  explicit LidarClusterer(LidarClustererConfiguration configuration);

  LidarClustererResult cluster(const std::vector<PointXYZI> &points,
                               std::int64_t stamp_nanoseconds) const;

private:
  LidarClustererConfiguration configuration_;
};

} // namespace uav_usv_lv_dot_core

#endif // UAV_USV_LV_DOT_CORE__LIDAR_CLUSTERER_HPP_
