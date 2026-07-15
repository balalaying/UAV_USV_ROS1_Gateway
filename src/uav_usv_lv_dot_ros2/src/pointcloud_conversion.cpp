#include "uav_usv_lv_dot_ros2/pointcloud_conversion.hpp"

#include <cstring>
#include <stdexcept>
#include <string>

#include <sensor_msgs/msg/point_field.hpp>

namespace uav_usv_lv_dot_ros2
{
namespace
{

const sensor_msgs::msg::PointField * find_field(
  const sensor_msgs::msg::PointCloud2 & message,
  const std::string & name)
{
  for (const auto & field : message.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

float read_float_field(
  const std::uint8_t * point,
  const sensor_msgs::msg::PointField & field)
{
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
    float value = 0.0F;
    std::memcpy(&value, point + field.offset, sizeof(value));
    return value;
  }
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT64) {
    double value = 0.0;
    std::memcpy(&value, point + field.offset, sizeof(value));
    return static_cast<float>(value);
  }
  throw std::runtime_error("PointCloud2 coordinate fields must be FLOAT32 or FLOAT64");
}

std::size_t field_size(const sensor_msgs::msg::PointField & field)
{
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
    return sizeof(float);
  }
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT64) {
    return sizeof(double);
  }
  throw std::runtime_error("PointCloud2 coordinate fields must be FLOAT32 or FLOAT64");
}

void validate_field(
  const sensor_msgs::msg::PointField & field,
  std::uint32_t point_step)
{
  if (field.count != 1 || field.offset + field_size(field) > point_step) {
    throw std::runtime_error("PointCloud2 field layout exceeds point_step");
  }
}

}  // namespace

void convert_point_cloud(
  const sensor_msgs::msg::PointCloud2 & message,
  uav_usv_lv_dot_core::PointCloudFrame & frame)
{
  if (message.is_bigendian) {
    throw std::runtime_error("Big-endian PointCloud2 is not supported in Phase 1");
  }

  const auto * x_field = find_field(message, "x");
  const auto * y_field = find_field(message, "y");
  const auto * z_field = find_field(message, "z");
  const auto * intensity_field = find_field(message, "intensity");
  if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
    throw std::runtime_error("PointCloud2 must contain x, y and z fields");
  }
  validate_field(*x_field, message.point_step);
  validate_field(*y_field, message.point_step);
  validate_field(*z_field, message.point_step);
  if (intensity_field != nullptr) {
    validate_field(*intensity_field, message.point_step);
  }

  const auto point_count = static_cast<std::size_t>(message.width) * message.height;
  const auto expected_size = static_cast<std::size_t>(message.row_step) * message.height;
  const auto minimum_row_step =
    static_cast<std::size_t>(message.point_step) * message.width;
  if (
    message.point_step == 0 || message.row_step < minimum_row_step ||
    message.data.size() < expected_size)
  {
    throw std::runtime_error("PointCloud2 data layout is inconsistent");
  }

  frame.points.clear();
  frame.points.reserve(point_count);
  frame.is_dense = message.is_dense;
  for (std::uint32_t row = 0; row < message.height; ++row) {
    const auto * row_data = message.data.data() + static_cast<std::size_t>(row) * message.row_step;
    for (std::uint32_t column = 0; column < message.width; ++column) {
      const auto * point = row_data + static_cast<std::size_t>(column) * message.point_step;
      uav_usv_lv_dot_core::PointXYZI converted;
      converted.x = read_float_field(point, *x_field);
      converted.y = read_float_field(point, *y_field);
      converted.z = read_float_field(point, *z_field);
      if (intensity_field != nullptr) {
        converted.intensity = read_float_field(point, *intensity_field);
        converted.has_intensity = true;
      }
      frame.points.push_back(converted);
    }
  }
}

}  // namespace uav_usv_lv_dot_ros2
