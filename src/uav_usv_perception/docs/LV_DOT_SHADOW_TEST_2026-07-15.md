# LV-DOT Shadow Mode Test - 2026-07-15

## Scope

This test validates the isolated ROS 1 backend and ROS 2 perception contract.
It does not switch the capture mission away from Gazebo ground truth and does
not claim that cross-vehicle camera/LiDAR fusion is complete.

## Environment

- Host: ROS 2 Humble
- Backend: ROS 1 Noetic container
- LV-DOT commit: `449bf2c960a26b067b235d82f6e0aac65fc05a6b`
- Container image: `uav-usv/lv-dot-noetic:449bf2c`
- Input relay: TCP 19090, latest message per sensor
- Output relay: TCP 19091, dynamic boxes and velocity markers only

## Commands

```bash
colcon build --packages-select uav_usv_perception uav_usv_mission \
  --symlink-install
tools/lv_dot/build_isolated.sh
ros2 launch uav_usv_perception lv_dot_shadow.launch.py \
  start_lv_dot_pose_adapter:=false
tools/lv_dot/run_isolated.sh
```

The pose adapter was disabled only for the synthetic input test because the
test publisher supplied a synchronized `PoseStamped` directly.

## Results

### Native input processing

Synthetic map pose, irregular moving point cloud, and RGB image were sent
through the bounded ingress. Observed relay rates were:

| Input | Requested rate | Relayed result |
| --- | ---: | ---: |
| PointCloud2 | 10 Hz | approximately 9.5-10 Hz |
| PoseStamped | 10 Hz | approximately 9.5-10 Hz |
| Image | 10 Hz | approximately 9.5-10 Hz |
| CameraInfo | interface check | relayed with original header and intrinsics |

Inside the Noetic container:

| Native topic | Result |
| --- | --- |
| `/onboard_detector/raw_lidar_point_cloud` | 10.00 Hz |
| `/onboard_detector/lidar_bboxes` | 10.00 Hz, non-empty cluster box |
| `/lv_dot/onboard_detector/velocity_visualizaton` | published |
| `/lv_dot/onboard_detector/dynamic_bboxes` | empty for this synthetic motion |

This proves that the native C++ detector consumes the ROS 2 sensor inputs and
runs its LiDAR clustering path. It does not yet prove successful native
dynamic classification. In this synthetic test the upstream Kalman velocity
remained zero, so the dynamic voting stage rejected the moving cluster.

### Output contract

A known dynamic `MarkerArray` was injected at the native output boundary with
the detector stopped. This isolates and verifies:

```text
ROS 1 MarkerArray -> TCP egress -> ROS 2 MarkerArray
-> lv_dot_adapter -> TrackedObjectArray -> shadow evaluator
```

Measured Shadow metrics:

- detection rate: `1.0`;
- track stability: `1.0`;
- ID switches: `0`;
- velocity error: approximately `7e-7 m/s`;
- adapter source mask: LiDAR;
- output frame: `map`.

### Fusion and source mux

The perception layer was started with its default `ground_truth` source.
Runtime switching produced:

| Source | `/fleet/perception/targets` | Status |
| --- | --- | --- |
| `sensor` | `lv_dot_test` | online, one track |
| `ground_truth` | `target_vessel` | online, one track |

The test restored `ground_truth` before shutdown. No automatic fallback or
mission-source replacement was added.

### Regression checks

- `python3 -m py_compile`: passed for all new Python and launch files;
- shell syntax: passed for both container helper scripts;
- `colcon test --packages-select uav_usv_perception`: 11 tests, 0 errors;
- Qt offscreen startup: six seconds without exception;
- earlier minimal dynamic-capture regression remained successful with
  `perception_source=ground_truth`.

## Known Issues

1. Native LV-DOT dynamic classification is not yet validated with the real
   Gazebo target and Mid-360 recording. The synthetic cluster reached LiDAR
   clustering but not `dynamic_bboxes`.
2. LV-DOT assumes a rigidly co-located camera and LiDAR. The UAV camera and
   USV Mid-360 are on different vehicles, so the current native backend is
   LiDAR-only; the UAV image is an auxiliary input, not fused evidence.
3. Upstream local LiDAR processing is limited to a small region around the
   sensor. Points outside that region must be clipped by the existing
   Mid-360 preprocessor.
4. Confidence, persistent ID, and maritime class are synthesized by the ROS 2
   adapter because upstream publishes visualization markers rather than a
   structured tracking message.

## Next Validation

Record synchronized Gazebo data from `uav_01`, `usv_01`, and
`target_vessel`, then tune only the isolated detector parameters against that
recording. Keep the capture mux on `ground_truth` until native
`dynamic_bboxes` is non-empty, stable, and within the agreed error bounds.
