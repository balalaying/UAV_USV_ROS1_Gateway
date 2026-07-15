#ifndef UAV_USV_LV_DOT_CORE__DBSCAN_HPP_
#define UAV_USV_LV_DOT_CORE__DBSCAN_HPP_

#include <cstdint>
#include <vector>

#include "uav_usv_lv_dot_core/types.hpp"

namespace uav_usv_lv_dot_core {

constexpr std::int32_t kUnclassified = -1;
constexpr std::int32_t kNoise = -2;

struct DbscanPoint {
  PointXYZI point;
  std::int32_t cluster_id{kUnclassified};
};

class Dbscan {
public:
  Dbscan(std::uint32_t minimum_points, double epsilon_squared);

  std::vector<DbscanPoint> run(const std::vector<PointXYZI> &points) const;

private:
  std::vector<std::size_t> neighbors(const std::vector<DbscanPoint> &points,
                                     const DbscanPoint &query) const;
  bool expand_cluster(std::vector<DbscanPoint> &points, std::size_t point_index,
                      std::int32_t cluster_id) const;

  std::uint32_t minimum_points_;
  double epsilon_squared_;
};

} // namespace uav_usv_lv_dot_core

#endif // UAV_USV_LV_DOT_CORE__DBSCAN_HPP_
