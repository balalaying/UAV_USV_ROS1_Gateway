#include <gtest/gtest.h>

#include <stdexcept>

#include "uav_usv_lv_dot_core/detector_core.hpp"

namespace core = uav_usv_lv_dot_core;

TEST(DetectorCore, RequiresConfiguration)
{
  core::DetectorCore detector;
  core::PointCloudFrame frame;
  EXPECT_THROW(detector.process(frame), std::logic_error);
}

TEST(DetectorCore, PreservesFrameContractAndPublishesNoPhaseOneTracks)
{
  core::DetectorCore detector;
  detector.configure(core::CoreConfiguration{});

  core::PointCloudFrame frame;
  frame.context.stamp_nanoseconds = 123456789;
  frame.context.dt_seconds = 0.1;
  frame.context.sensor_frame = "sensor";
  frame.context.output_frame = "map";
  frame.points.push_back(core::PointXYZI{1.0F, 2.0F, 3.0F, 4.0F, true});

  const auto result = detector.process(frame);
  EXPECT_EQ(result.stamp_nanoseconds, frame.context.stamp_nanoseconds);
  EXPECT_EQ(result.output_frame, "map");
  EXPECT_TRUE(result.tracks.empty());
  EXPECT_EQ(detector.processed_frames(), 1U);
}

TEST(DetectorCore, ResetClearsState)
{
  core::DetectorCore detector;
  detector.configure(core::CoreConfiguration{});
  detector.process(core::PointCloudFrame{});
  detector.reset();

  EXPECT_FALSE(detector.configured());
  EXPECT_EQ(detector.processed_frames(), 0U);
}
