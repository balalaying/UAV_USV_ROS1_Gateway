#include "uav_usv_lv_dot_core/kalman_filter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace uav_usv_lv_dot_core {
namespace {

using Matrix3 = std::array<double, 9>;

double &at(Matrix3 &matrix, std::size_t row, std::size_t column) {
  return matrix[row * 3U + column];
}

double at(const Matrix3 &matrix, std::size_t row, std::size_t column) {
  return matrix[row * 3U + column];
}

Matrix3 multiply(const Matrix3 &left, const Matrix3 &right) {
  Matrix3 result{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      for (std::size_t inner = 0; inner < 3; ++inner) {
        at(result, row, column) +=
            at(left, row, inner) * at(right, inner, column);
      }
    }
  }
  return result;
}

Matrix3 transpose(const Matrix3 &matrix) {
  Matrix3 result{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      at(result, row, column) = at(matrix, column, row);
    }
  }
  return result;
}

Matrix3 inverse(const Matrix3 &matrix) {
  const double determinant =
      at(matrix, 0, 0) *
          (at(matrix, 1, 1) * at(matrix, 2, 2) -
           at(matrix, 1, 2) * at(matrix, 2, 1)) -
      at(matrix, 0, 1) *
          (at(matrix, 1, 0) * at(matrix, 2, 2) -
           at(matrix, 1, 2) * at(matrix, 2, 0)) +
      at(matrix, 0, 2) *
          (at(matrix, 1, 0) * at(matrix, 2, 1) -
           at(matrix, 1, 1) * at(matrix, 2, 0));
  if (!std::isfinite(determinant) || std::abs(determinant) < 1.0e-12) {
    throw std::runtime_error("Kalman innovation matrix is singular");
  }
  Matrix3 result{};
  at(result, 0, 0) =
      (at(matrix, 1, 1) * at(matrix, 2, 2) -
       at(matrix, 1, 2) * at(matrix, 2, 1)) /
      determinant;
  at(result, 0, 1) =
      (at(matrix, 0, 2) * at(matrix, 2, 1) -
       at(matrix, 0, 1) * at(matrix, 2, 2)) /
      determinant;
  at(result, 0, 2) =
      (at(matrix, 0, 1) * at(matrix, 1, 2) -
       at(matrix, 0, 2) * at(matrix, 1, 1)) /
      determinant;
  at(result, 1, 0) =
      (at(matrix, 1, 2) * at(matrix, 2, 0) -
       at(matrix, 1, 0) * at(matrix, 2, 2)) /
      determinant;
  at(result, 1, 1) =
      (at(matrix, 0, 0) * at(matrix, 2, 2) -
       at(matrix, 0, 2) * at(matrix, 2, 0)) /
      determinant;
  at(result, 1, 2) =
      (at(matrix, 0, 2) * at(matrix, 1, 0) -
       at(matrix, 0, 0) * at(matrix, 1, 2)) /
      determinant;
  at(result, 2, 0) =
      (at(matrix, 1, 0) * at(matrix, 2, 1) -
       at(matrix, 1, 1) * at(matrix, 2, 0)) /
      determinant;
  at(result, 2, 1) =
      (at(matrix, 0, 1) * at(matrix, 2, 0) -
       at(matrix, 0, 0) * at(matrix, 2, 1)) /
      determinant;
  at(result, 2, 2) =
      (at(matrix, 0, 0) * at(matrix, 1, 1) -
       at(matrix, 0, 1) * at(matrix, 1, 0)) /
      determinant;
  return result;
}

} // namespace

void ConstantAccelerationKalman::initialize(double position,
                                             const KalmanNoise &noise) {
  noise_ = noise;
  state_ = {position, 0.0, 0.0};
  covariance_ = {};
  at(covariance_, 0, 0) = noise.initial_covariance;
  at(covariance_, 1, 1) = noise.initial_covariance;
  at(covariance_, 2, 2) = noise.initial_covariance;
  initialized_ = true;
}

void ConstantAccelerationKalman::predict(double dt_seconds) {
  if (!initialized_ || !std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
    return;
  }
  const double dt_squared = dt_seconds * dt_seconds;
  const Matrix3 transition{{1.0, dt_seconds, 0.5 * dt_squared,
                            0.0, 1.0, dt_seconds,
                            0.0, 0.0, 1.0}};
  const auto previous = state_;
  for (std::size_t row = 0; row < 3; ++row) {
    state_[row] = 0.0;
    for (std::size_t column = 0; column < 3; ++column) {
      state_[row] += at(transition, row, column) * previous[column];
    }
  }
  covariance_ = multiply(multiply(transition, covariance_),
                         transpose(transition));
  at(covariance_, 0, 0) += noise_.process_position;
  at(covariance_, 1, 1) += noise_.process_velocity;
  at(covariance_, 2, 2) += noise_.process_acceleration;
}

void ConstantAccelerationKalman::update(double position, double velocity,
                                         double acceleration) {
  if (!initialized_) {
    return;
  }
  Matrix3 innovation = covariance_;
  at(innovation, 0, 0) += noise_.measurement_position;
  at(innovation, 1, 1) += noise_.measurement_velocity;
  at(innovation, 2, 2) += noise_.measurement_acceleration;
  const Matrix3 gain = multiply(covariance_, inverse(innovation));
  const std::array<double, 3> residual{{position - state_[0],
                                        velocity - state_[1],
                                        acceleration - state_[2]}};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      state_[row] += at(gain, row, column) * residual[column];
    }
  }
  Matrix3 identity_minus_gain{{1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0}};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      at(identity_minus_gain, row, column) -= at(gain, row, column);
    }
  }
  covariance_ = multiply(identity_minus_gain, covariance_);
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = row + 1; column < 3; ++column) {
      const double symmetric =
          0.5 * (at(covariance_, row, column) +
                 at(covariance_, column, row));
      at(covariance_, row, column) = symmetric;
      at(covariance_, column, row) = symmetric;
    }
    at(covariance_, row, row) =
        std::max(0.0, at(covariance_, row, row));
  }
}

bool ConstantAccelerationKalman::initialized() const noexcept {
  return initialized_;
}

double ConstantAccelerationKalman::position() const noexcept {
  return state_[0];
}

double ConstantAccelerationKalman::velocity() const noexcept {
  return state_[1];
}

double ConstantAccelerationKalman::acceleration() const noexcept {
  return state_[2];
}

double ConstantAccelerationKalman::covariance(std::size_t row,
                                               std::size_t column) const {
  if (row >= 3 || column >= 3) {
    throw std::out_of_range("Kalman covariance index out of range");
  }
  return at(covariance_, row, column);
}

} // namespace uav_usv_lv_dot_core
