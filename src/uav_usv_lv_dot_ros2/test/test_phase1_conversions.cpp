#include <gtest/gtest.h>

#include <stdexcept>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "uav_usv_lv_dot_core/types.hpp"
#include "uav_usv_lv_dot_ros2/message_conversion.hpp"
#include "uav_usv_lv_dot_ros2/pointcloud_conversion.hpp"

namespace core = uav_usv_lv_dot_core;
namespace wrapper = uav_usv_lv_dot_ros2;

TEST(PointCloudConversion, ConvertsXyzAndIntensity)
{
  sensor_msgs::msg::PointCloud2 message;
  sensor_msgs::PointCloud2Modifier modifier(message);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(2);

  sensor_msgs::PointCloud2Iterator<float> x(message, "x");
  sensor_msgs::PointCloud2Iterator<float> y(message, "y");
  sensor_msgs::PointCloud2Iterator<float> z(message, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(message, "intensity");
  for (std::size_t index = 0; index < 2; ++index, ++x, ++y, ++z, ++intensity) {
    *x = static_cast<float>(index + 1);
    *y = static_cast<float>(index + 2);
    *z = static_cast<float>(index + 3);
    *intensity = static_cast<float>(index + 4);
  }

  core::PointCloudFrame frame;
  wrapper::convert_point_cloud(message, frame);
  ASSERT_EQ(frame.points.size(), 2U);
  EXPECT_FLOAT_EQ(frame.points[0].x, 1.0F);
  EXPECT_FLOAT_EQ(frame.points[1].z, 4.0F);
  EXPECT_TRUE(frame.points[0].has_intensity);
  EXPECT_FLOAT_EQ(frame.points[1].intensity, 5.0F);
}

TEST(PointCloudConversion, RejectsACloudWithoutZ)
{
  sensor_msgs::msg::PointCloud2 message;
  sensor_msgs::PointCloud2Modifier modifier(message);
  modifier.setPointCloud2Fields(
    2,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(1);

  core::PointCloudFrame frame;
  EXPECT_THROW(wrapper::convert_point_cloud(message, frame), std::runtime_error);
}

TEST(MessageConversion, PreservesCompleteTrackFields)
{
  core::DetectionResult result;
  result.stamp_nanoseconds = 2123456789LL;
  result.output_frame = "map";
  core::TrackEstimate track;
  track.track_id = "track_1";
  track.source = core::ObservationSource::kLidar;
  track.classification = core::ObjectClass::kVessel;
  track.position = {1.0, 2.0, 3.0};
  track.linear_velocity = {4.0, 5.0, 6.0};
  track.dimensions = {7.0, 8.0, 9.0};
  track.confidence = 0.75F;
  result.tracks.push_back(track);

  const auto message = wrapper::to_ros_message(result);
  EXPECT_EQ(message.header.frame_id, "map");
  EXPECT_EQ(message.header.stamp.sec, 2);
  ASSERT_EQ(message.objects.size(), 1U);
  EXPECT_EQ(message.objects[0].track_id, "track_1");
  EXPECT_EQ(message.objects[0].source_mask, 1U);
  EXPECT_EQ(message.objects[0].classification, 1U);
  EXPECT_DOUBLE_EQ(message.objects[0].pose.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(message.objects[0].twist.twist.linear.z, 6.0);
  EXPECT_FLOAT_EQ(message.objects[0].confidence, 0.75F);
}
