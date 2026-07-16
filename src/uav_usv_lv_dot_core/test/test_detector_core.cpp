#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "uav_usv_lv_dot_core/dbscan.hpp"
#include "uav_usv_lv_dot_core/detector_core.hpp"
#include "uav_usv_lv_dot_core/dynamic_classifier.hpp"
#include "uav_usv_lv_dot_core/lidar_clusterer.hpp"
#include "uav_usv_lv_dot_core/multi_object_tracker.hpp"

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

TEST(DetectorCore, FiltersTransformsClustersAndStartsTracks) {
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
  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks[0].lifecycle, core::TrackLifecycle::kNew);
  EXPECT_EQ(result.tracks[0].track_id, "lv_dot_ros2_track_000001");
  EXPECT_NEAR(result.lidar_clusters[0].center[0], 11.0, 1.0e-5);
  EXPECT_NEAR(result.lidar_clusters[0].center[1], -3.0, 1.0e-5);
  EXPECT_NEAR(result.lidar_clusters[0].center[2], 1.033333333, 1.0e-5);
  EXPECT_EQ(result.clustering_statistics.input_point_count, 5U);
  EXPECT_EQ(result.clustering_statistics.finite_point_count, 4U);
  EXPECT_EQ(result.clustering_statistics.preprocessed_point_count, 3U);
}

TEST(MultiObjectTracker, KeepsStableIdAndUsesLifecycleGracePeriod) {
  core::TrackingConfiguration configuration;
  configuration.confirmation_hits = 3;
  configuration.maximum_missed_frames = 2;
  core::MultiObjectTracker tracker;
  tracker.configure(configuration);

  core::LidarCluster detection;
  detection.center = {0.0, 0.0, 1.0};
  detection.dimensions = {2.0, 1.0, 1.0};
  detection.point_count = 20;
  auto first = tracker.update({detection}, 1000000000LL, {0.0, 0.0, 0.0});
  ASSERT_EQ(first.tracks.size(), 1U);
  const auto identifier = first.tracks[0].track_id;
  EXPECT_EQ(first.tracks[0].lifecycle, core::TrackLifecycle::kNew);

  detection.center[0] = 0.5;
  auto second = tracker.update({detection}, 1500000000LL, {0.0, 0.0, 0.0});
  ASSERT_EQ(second.tracks.size(), 1U);
  EXPECT_EQ(second.tracks[0].track_id, identifier);
  EXPECT_EQ(second.tracks[0].lifecycle, core::TrackLifecycle::kNew);

  detection.center[0] = 1.5;
  auto third = tracker.update({detection}, 2500000000LL, {0.0, 0.0, 0.0});
  ASSERT_EQ(third.tracks.size(), 1U);
  EXPECT_EQ(third.tracks[0].track_id, identifier);
  EXPECT_EQ(third.tracks[0].lifecycle, core::TrackLifecycle::kConfirmed);
  EXPECT_GT(third.tracks[0].linear_velocity[0], 0.0);
  EXPECT_EQ(third.statistics.confirmed_track_count, 1U);

  auto lost = tracker.update({}, 3000000000LL, {0.0, 0.0, 0.0});
  ASSERT_EQ(lost.tracks.size(), 1U);
  EXPECT_EQ(lost.tracks[0].track_id, identifier);
  EXPECT_EQ(lost.tracks[0].lifecycle, core::TrackLifecycle::kLost);
  EXPECT_EQ(lost.tracks[0].missed_count, 1U);

  tracker.update({}, 3500000000LL, {0.0, 0.0, 0.0});
  const auto removed = tracker.update({}, 4000000000LL, {0.0, 0.0, 0.0});
  EXPECT_TRUE(removed.tracks.empty());
  EXPECT_EQ(removed.statistics.removed_track_count, 1U);
}

TEST(MultiObjectTracker, PreventsTwoDetectionsFromOwningOneTrack) {
  core::TrackingConfiguration configuration;
  core::MultiObjectTracker tracker;
  tracker.configure(configuration);
  core::LidarCluster seed;
  seed.center = {0.0, 0.0, 1.0};
  seed.dimensions = {1.0, 1.0, 1.0};
  tracker.update({seed}, 1000000000LL, {0.0, 0.0, 0.0});

  auto first = seed;
  auto second = seed;
  first.center[0] = 0.1;
  second.center[0] = 0.2;
  const auto result =
      tracker.update({first, second}, 1100000000LL, {0.0, 0.0, 0.0});
  ASSERT_EQ(result.tracks.size(), 2U);
  EXPECT_EQ(result.statistics.matched_count, 1U);
  EXPECT_EQ(result.statistics.created_track_count, 1U);
  EXPECT_NE(result.tracks[0].track_id, result.tracks[1].track_id);
}

TEST(MultiObjectTracker, VelocityObservationUsesActualTimestampDifference) {
  core::TrackingConfiguration configuration;
  configuration.confirmation_hits = 2;
  core::LidarCluster detection;
  detection.center = {0.0, 0.0, 1.0};
  detection.dimensions = {1.0, 1.0, 1.0};

  core::MultiObjectTracker slow_tracker;
  slow_tracker.configure(configuration);
  slow_tracker.update({detection}, 1000000000LL, {0.0, 0.0, 0.0});
  detection.center[0] = 1.0;
  const auto slow =
      slow_tracker.update({detection}, 2000000000LL, {0.0, 0.0, 0.0});

  detection.center[0] = 0.0;
  core::MultiObjectTracker fast_tracker;
  fast_tracker.configure(configuration);
  fast_tracker.update({detection}, 1000000000LL, {0.0, 0.0, 0.0});
  detection.center[0] = 1.0;
  const auto fast =
      fast_tracker.update({detection}, 1500000000LL, {0.0, 0.0, 0.0});

  ASSERT_EQ(slow.tracks.size(), 1U);
  ASSERT_EQ(fast.tracks.size(), 1U);
  EXPECT_GT(fast.tracks[0].linear_velocity[0],
            slow.tracks[0].linear_velocity[0] * 1.2);
}

TEST(DynamicClassifier, ConfirmsContinuousMotionAfterTunedHistoryWindow) {
  core::DynamicClassifier classifier;
  classifier.configure(core::DynamicClassificationConfiguration{});
  core::TrackEstimate track;
  track.track_id = "moving_track";
  track.lifecycle = core::TrackLifecycle::kConfirmed;
  track.confidence = 1.0F;
  track.linear_velocity = {1.0, 0.0, 0.0};

  core::DynamicClassificationResult result;
  for (std::int64_t frame = 0; frame < 13; ++frame) {
    track.position[0] = static_cast<double>(frame) * 0.1;
    result = classifier.classify({track}, 1000000000LL + frame * 100000000LL);
    if (frame < 12) {
      EXPECT_TRUE(result.dynamic_tracks.empty());
    }
  }

  ASSERT_EQ(result.classified_tracks.size(), 1U);
  EXPECT_EQ(result.classified_tracks[0].motion_state,
            core::DynamicMotionState::kConfirmedDynamic);
  EXPECT_TRUE(result.classified_tracks[0].is_dynamic);
  EXPECT_GT(result.classified_tracks[0].dynamic_probability, 0.5);
  EXPECT_EQ(result.classified_tracks[0].motion_history.size(), 13U);
  ASSERT_EQ(result.dynamic_tracks.size(), 1U);
  EXPECT_EQ(result.dynamic_tracks[0].track_id, track.track_id);
}

TEST(DynamicClassifier, KeepsAStationaryTrackStatic) {
  core::DynamicClassifier classifier;
  classifier.configure(core::DynamicClassificationConfiguration{});
  core::TrackEstimate track;
  track.track_id = "static_track";
  track.lifecycle = core::TrackLifecycle::kConfirmed;
  track.confidence = 1.0F;

  for (std::int64_t frame = 0; frame < 20; ++frame) {
    const auto result =
        classifier.classify({track}, 1000000000LL + frame * 100000000LL);
    ASSERT_EQ(result.classified_tracks.size(), 1U);
    EXPECT_EQ(result.classified_tracks[0].motion_state,
              core::DynamicMotionState::kStatic);
    EXPECT_FALSE(result.classified_tracks[0].is_dynamic);
    EXPECT_TRUE(result.dynamic_tracks.empty());
    EXPECT_EQ(result.statistics.static_track_count, 1U);
  }
}

TEST(DynamicClassifier, ForceDynamicRetainsARecentlyConfirmedTrack) {
  core::DynamicClassifier classifier;
  classifier.configure(core::DynamicClassificationConfiguration{});
  core::TrackEstimate track;
  track.track_id = "retained_track";
  track.lifecycle = core::TrackLifecycle::kConfirmed;
  track.confidence = 1.0F;
  track.linear_velocity = {1.0, 0.0, 0.0};

  for (std::int64_t frame = 0; frame < 16; ++frame) {
    track.position[0] = static_cast<double>(frame) * 0.1;
    classifier.classify({track}, 1000000000LL + frame * 100000000LL);
  }
  track.linear_velocity = {0.0, 0.0, 0.0};
  const auto retained = classifier.classify({track}, 2600000000LL);
  ASSERT_EQ(retained.classified_tracks.size(), 1U);
  EXPECT_TRUE(retained.classified_tracks[0].is_dynamic);
  EXPECT_EQ(retained.classified_tracks[0].motion_state,
            core::DynamicMotionState::kConfirmedDynamic);
}

TEST(DynamicClassifier, DoesNotPublishPredictedLostTracksAsDynamic) {
  core::DynamicClassifier classifier;
  classifier.configure(core::DynamicClassificationConfiguration{});
  core::TrackEstimate track;
  track.track_id = "lost_track";
  track.lifecycle = core::TrackLifecycle::kConfirmed;
  track.confidence = 1.0F;
  track.linear_velocity = {1.0, 0.0, 0.0};
  for (std::int64_t frame = 0; frame < 13; ++frame) {
    track.position[0] = static_cast<double>(frame) * 0.1;
    classifier.classify({track}, 1000000000LL + frame * 100000000LL);
  }

  track.lifecycle = core::TrackLifecycle::kLost;
  const auto lost = classifier.classify({track}, 2300000000LL);
  EXPECT_TRUE(lost.dynamic_tracks.empty());
  EXPECT_EQ(lost.statistics.unclassified_track_count, 1U);
}
