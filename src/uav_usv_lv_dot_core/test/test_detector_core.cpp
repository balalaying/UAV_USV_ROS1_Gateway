#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "uav_usv_lv_dot_core/dbscan.hpp"
#include "uav_usv_lv_dot_core/detector_core.hpp"
#include "uav_usv_lv_dot_core/lidar_clusterer.hpp"

namespace core = uav_usv_lv_dot_core;

TEST(DetectorCore, RequiresConfiguration) {
  core::DetectorCore detector;
  core::PointCloudFrame frame;
  EXPECT_THROW(detector.process(frame), std::logic_error);
}

TEST(DetectorCore, PreservesFrameContractAndPublishesNoPhaseOneTracks) {
  core::DetectorCore detector;
  core::CoreConfiguration configuration;
  configuration.input_min_range = 0.0;
  configuration.input_max_range = 100.0;
  configuration.input_min_z = -10.0;
  configuration.input_max_z = 10.0;
  configuration.input_voxel_size = 0.0;
  configuration.crop_self = false;
  configuration.local_range_x = 100.0;
  configuration.local_range_y = 100.0;
  configuration.ground_height = -10.0;
  configuration.roof_height = 10.0;
  configuration.gaussian_downsample_sigma = 1.0e9;
  detector.configure(configuration);

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
  EXPECT_TRUE(result.lidar_clusters.empty());
  EXPECT_EQ(result.clustering_statistics.input_point_count, 1U);
  EXPECT_EQ(detector.processed_frames(), 1U);
}

TEST(DetectorCore, ResetClearsState) {
  core::DetectorCore detector;
  detector.configure(core::CoreConfiguration{});
  detector.process(core::PointCloudFrame{});
  detector.reset();

  EXPECT_FALSE(detector.configured());
  EXPECT_EQ(detector.processed_frames(), 0U);
}

TEST(Dbscan, PreservesUpstreamSquaredEpsilonSemantics) {
  const std::vector<core::PointXYZI> points{{0.0F, 0.0F, 0.0F, 0.0F, false},
                                            {0.7F, 0.0F, 0.0F, 0.0F, false},
                                            {1.4F, 0.0F, 0.0F, 0.0F, false},
                                            {5.0F, 0.0F, 0.0F, 0.0F, false}};
  core::Dbscan dbscan(2, 0.65);
  const auto classified = dbscan.run(points);

  ASSERT_EQ(classified.size(), points.size());
  EXPECT_EQ(classified[0].cluster_id, 1);
  EXPECT_EQ(classified[1].cluster_id, 1);
  EXPECT_EQ(classified[2].cluster_id, 1);
  EXPECT_EQ(classified[3].cluster_id, core::kNoise);
}

TEST(LidarClusterer, ProducesCentroidAxisAlignedSizeAndPointCounts) {
  core::LidarClustererConfiguration configuration;
  configuration.dbscan_min_points = 3;
  configuration.dbscan_epsilon_squared = 0.65;
  core::LidarClusterer clusterer(configuration);
  const std::vector<core::PointXYZI> points{{1.0F, 2.0F, 3.0F, 0.0F, false},
                                            {1.2F, 2.0F, 3.2F, 0.0F, false},
                                            {0.8F, 2.4F, 2.8F, 0.0F, false},
                                            {8.0F, 8.0F, 3.0F, 0.0F, false}};

  const auto result = clusterer.cluster(points, 42);
  ASSERT_EQ(result.clusters.size(), 1U);
  EXPECT_EQ(result.clusters[0].cluster_id, 1);
  EXPECT_EQ(result.clusters[0].point_count, 3U);
  EXPECT_EQ(result.clusters[0].stamp_nanoseconds, 42);
  EXPECT_NEAR(result.clusters[0].center[0], 1.0, 1.0e-6);
  EXPECT_NEAR(result.clusters[0].center[1], 2.133333333, 1.0e-6);
  EXPECT_NEAR(result.clusters[0].center[2], 3.0, 1.0e-6);
  EXPECT_NEAR(result.clusters[0].dimensions[0], 0.4, 1.0e-6);
  EXPECT_NEAR(result.clusters[0].dimensions[1], 0.4, 1.0e-6);
  EXPECT_NEAR(result.clusters[0].dimensions[2], 0.4, 1.0e-6);
  EXPECT_EQ(result.clustered_point_count, 3U);
  EXPECT_EQ(result.noise_point_count, 1U);
}

TEST(DetectorCore, FiltersTransformsAndClustersWithoutProducingTracks) {
  core::CoreConfiguration configuration;
  configuration.input_min_range = 0.0;
  configuration.input_max_range = 20.0;
  configuration.input_min_z = -2.0;
  configuration.input_max_z = 4.0;
  configuration.input_voxel_size = 0.0;
  configuration.crop_self = false;
  configuration.local_range_x = 10.0;
  configuration.local_range_y = 10.0;
  configuration.ground_height = 0.2;
  configuration.roof_height = 6.0;
  configuration.gaussian_downsample_sigma = 1.0e9;
  configuration.dbscan_min_points = 3;

  core::PointCloudFrame frame;
  frame.context.stamp_nanoseconds = 100;
  frame.context.output_frame = "map";
  frame.context.sensor_to_output.translation = {10.0, -3.0, 1.0};
  frame.points = {
      {1.0F, 0.0F, 0.0F, 0.0F, false},
      {1.1F, 0.1F, 0.0F, 0.0F, false},
      {0.9F, -0.1F, 0.1F, 0.0F, false},
      {1.0F, 0.0F, -1.0F, 0.0F, false},
      {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 0.0F, false}};

  core::DetectorCore detector;
  detector.configure(configuration);
  const auto result = detector.process(frame);
  ASSERT_EQ(result.lidar_clusters.size(), 1U);
  EXPECT_TRUE(result.tracks.empty());
  EXPECT_NEAR(result.lidar_clusters[0].center[0], 11.0, 1.0e-5);
  EXPECT_NEAR(result.lidar_clusters[0].center[1], -3.0, 1.0e-5);
  EXPECT_NEAR(result.lidar_clusters[0].center[2], 1.033333333, 1.0e-5);
  EXPECT_EQ(result.clustering_statistics.input_point_count, 5U);
  EXPECT_EQ(result.clustering_statistics.finite_point_count, 4U);
  EXPECT_EQ(result.clustering_statistics.preprocessed_point_count, 3U);
}
