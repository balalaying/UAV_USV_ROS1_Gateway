#ifndef UAV_USV_LV_DOT_CORE__KALMAN_FILTER_HPP_
#define UAV_USV_LV_DOT_CORE__KALMAN_FILTER_HPP_

#include <array>

namespace uav_usv_lv_dot_core {

struct KalmanNoise {
  double initial_covariance{0.25};
  double process_position{0.01};
  double process_velocity{0.05};
  double process_acceleration{0.05};
  double measurement_position{0.04};
  double measurement_velocity{0.3};
  double measurement_acceleration{0.6};
};

class ConstantAccelerationKalman {
public:
  void initialize(double position, const KalmanNoise &noise);
  void predict(double dt_seconds);
  void update(double position, double velocity, double acceleration);

  bool initialized() const noexcept;
  double position() const noexcept;
  double velocity() const noexcept;
  double acceleration() const noexcept;
  double covariance(std::size_t row, std::size_t column) const;

private:
  std::array<double, 3> state_{{0.0, 0.0, 0.0}};
  std::array<double, 9> covariance_{};
  KalmanNoise noise_;
  bool initialized_{false};
};

} // namespace uav_usv_lv_dot_core

#endif // UAV_USV_LV_DOT_CORE__KALMAN_FILTER_HPP_
