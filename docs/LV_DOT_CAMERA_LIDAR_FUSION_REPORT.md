# LV-DOT Camera-LiDAR Fusion Report

## 1. Scope and safety boundary

This stage adds a passive USV camera and Mid-360 association path. It does not
publish commands and does not replace the capture perception source.

- `perception_source` remains `ground_truth`.
- `capture_manager`, `FleetCommand`, PX4, Nav2 and the vehicle agents are not
  consumers of the new fusion output.
- The new path is a Shadow Mode candidate perception source only.

## 2. Data flow

```text
USV front camera image_raw + camera_info
        |
        v
usv_camera_detection_node
        |
        +--> Detection2DArray
        +--> annotated debug image
        |
        v
camera_lidar_association_node <--- LV-DOT LiDAR 3D bboxes + tracks
        |
        +--> TrackedObjectArray observations
        +--> yellow LiDAR-only markers
        +--> blue camera-only markers
        +--> green camera+LiDAR markers
        +--> JSON diagnostics
```

The camera supplies semantic class and confidence. Mid-360/LV-DOT supplies
metric 3D position, dimensions and tracked velocity. A successful association
therefore keeps the LiDAR geometry while adding the camera classification.

## 3. ROS 2 interfaces

| Direction | Topic | Type | Frame |
|---|---|---|---|
| Input | `/fleet/uplink/usv_01/camera/image_raw` | `sensor_msgs/Image` | `usv_01/camera_link` |
| Input | `/fleet/uplink/usv_01/camera/camera_info` | `sensor_msgs/CameraInfo` | `usv_01/camera_link` |
| Input | `/perception/lv_dot_ros2/diagnostics/lidar_bboxes` | `visualization_msgs/MarkerArray` | `map` |
| Input | `/perception/lv_dot_ros2/tracks` | `uav_usv_interfaces/TrackedObjectArray` | `map` |
| Output | `/perception/usv_01/camera/detections` | `vision_msgs/Detection2DArray` | camera frame |
| Output | `/perception/usv_01/camera/detections/image` | `sensor_msgs/Image` | camera frame |
| Output | `/perception/usv_01/camera_lidar/observations` | `uav_usv_interfaces/TrackedObjectArray` | `map` |
| Output | `/perception/usv_01/camera_lidar/lidar_only_bboxes` | `visualization_msgs/MarkerArray` | `map` |
| Output | `/perception/usv_01/camera_lidar/camera_only_bboxes` | `visualization_msgs/MarkerArray` | `map` |
| Output | `/perception/usv_01/camera_lidar/fused_bboxes` | `visualization_msgs/MarkerArray` | `map` |
| Output | `/perception/usv_01/camera_lidar/status` | `std_msgs/String` | n/a |

TF used for each LiDAR frame timestamp:

```text
map -> usv_01/base_link -> usv_01/camera_link
                         -> usv_01/mid360_link
```

The camera mounting transform is `(3.24, 0.0, 1.55)` metres in `base_link`.
The sensor and visual were moved together above the bow so that the hull does
not block the image. No mass, inertia, collision or control property changed.

## 4. Detection and association

The simulation camera detector produces `vision_msgs/Detection2DArray`. It
uses the target's coloured hull/navigation-light appearance only to validate
the ROS 2 fusion interface; it is not presented as a production vision model.

For every LiDAR bbox frame the association node:

1. Selects the nearest camera detection frame inside `sync_slop_seconds`.
2. Looks up the historical `camera_link <- map` transform using the LiDAR
   timestamp. It does not substitute the latest TF.
3. Projects all eight 3D bbox corners through `CameraInfo.K`.
4. Scores 2D/3D pairs using rectangle IoU and a configurable pixel gate.
5. Performs greedy one-to-one matching, preventing two clusters from claiming
   the same camera detection.
6. Reuses the nearest LV-DOT track ID when available.

The association process uses a two-thread ROS 2 executor. One thread can
receive time-stamped TF while the other performs projection, avoiding the
single-threaded dead time where a callback waits for a transform that its own
executor has not yet serviced.

Unmatched observations remain visible instead of being discarded:

- green: `camera+lidar`, source mask Camera | LiDAR | Fused;
- yellow: `lidar`, metric 3D observation without camera semantics;
- blue: `camera`, monocular range estimate with deliberately large covariance.

`TrackedObject` was compatibly extended with `class_name`,
`class_confidence`, and `sensor_source`. Existing enum and bitmask fields remain
unchanged, so existing consumers continue to work.

## 5. Qt display

The existing Perception Monitor subscribes to all three marker layers. Its
layer switches are display-only and never change an algorithm or publish a
task command. The side status panel reports the current LiDAR-only,
camera-only and fused counts.

## 6. Launch and tuning parameters

Full live Shadow Mode demo:

```bash
source /opt/ros/humble/setup.bash
source ~/UAV_USV/install/setup.bash
ros2 launch uav_usv_bringup fleet_dynamic_capture_live_perception.launch.py
```

Standalone association nodes, when the camera and LV-DOT topics already
exist:

```bash
ros2 launch uav_usv_perception camera_lidar_fusion.launch.py \
  use_sim_time:=false
```

Important parameters:

| Parameter | Default | Effect |
|---|---:|---|
| `sync_slop_seconds` | 0.20 s | Maximum camera/LiDAR timestamp difference |
| `camera_max_rate_hz` | 20 Hz | Camera detection processing cap |
| `camera_min_pixels` | 2 | Minimum simulation colour component size |
| `camera_maximum_center_y_ratio` | 0.88 | Reject low image/water artifacts |
| `minimum_association_score` | 0.08 | Minimum IoU/proximity score |
| `association_pixel_gate` | 32 px | Projection centre tolerance |
| `minimum_lidar_xy_extent` | 0.20 m | Reject tiny LiDAR noise clusters |

## 7. Verification

Build and test commands:

```bash
colcon build --packages-select \
  uav_usv_interfaces uav_usv_gazebo uav_usv_lv_dot_core \
  uav_usv_lv_dot_ros2 uav_usv_perception uav_usv_mission \
  uav_usv_bringup --symlink-install
colcon test --packages-select uav_usv_perception uav_usv_mission
colcon test-result --verbose
```

Result on 2026-07-18: 67 tests, 0 errors, 0 failures, 0 skipped.

One live overlap sample from the complete fleet world:

| Metric | Measured value |
|---|---:|
| association frames | 261 |
| accumulated matches | 174 |
| camera/LiDAR timestamp error | 9.9 ms |
| projected LiDAR boxes | 2 |
| candidate pairs | 4 |
| association processing time | 3.35 ms |
| TF failures | 0 |
| current fused / LiDAR-only / camera-only | 1 / 1 / 7 |

The fused observation contained:

```text
track_id: lv_dot_ros2_track_001105
source_mask: 11
class_name: vessel
sensor_source: camera+lidar
class_confidence: 0.691
```

The association status explicitly reports `control_connected=false`, and
`/fleet/perception/targets` continues to contain the ground-truth source.

## 8. Known limitations

- The camera detector is a deterministic simulation adapter, not a learned
  maritime detector. A real detector can replace it without changing the
  association output contract.
- Camera-only distance uses known vessel width and therefore has high
  covariance. Only a fused result should be treated as metric 3D perception.
- A green box exists only while the object is visible to both sensors. When it
  leaves the camera FOV, correct behaviour is a yellow LiDAR-only box.
- This stage does not calibrate lens distortion. The current Gazebo camera has
  an ideal pinhole model.
- Fusion remains Shadow Mode and is intentionally disconnected from control.
