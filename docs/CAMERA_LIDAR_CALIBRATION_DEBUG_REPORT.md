# USV Camera-LiDAR Geometry Calibration Debug Report

## 1. Scope and safety boundary

This change only repairs the USV perception and Qt debug-display path. It does
not modify PX4, Nav2, FleetCommand, capture_manager, behavior logic, or the
default perception source. The system remains in Shadow Mode and
`perception_source=ground_truth`.

The calibration path is:

```text
USV-01 camera detection (image timestamp)
              |
              v
historical camera pose in map
              |
Mid-360 cloud (cloud timestamp) -> historical LiDAR pose in map
              |                              |
              +---------- map ---------------+
                             |
                             v
               image ROI / DBSCAN cluster
                             |
                             v
              measured Camera+LiDAR 3D bbox
```

## 2. Root causes

### 2.1 USV-02 had points but no LiDAR box

The screenshot point cluster is USV-02. The ROS 2 LV-DOT detector was still
using the offline rosbag comparison window:

```text
local_range_x = 10 m
local_range_y = 10 m
input_max_range = 20 m
```

USV-02 starts about 12 m from USV-01. Its returns therefore reached the Qt raw
point-cloud layer, but were removed before DBSCAN. No cluster existed upstream,
so Qt correctly had no box to draw.

The live fleet launch now overrides only the real-time window:

```text
input_max_range = 70 m
local_range_x = 50 m
local_range_y = 20 m
```

The standalone LV-DOT launch keeps the tuned offline defaults, preserving the
three rosbag comparison baselines.

No synthetic or manually positioned box was added. Qt only renders ADD markers
published by the ROS2 DBSCAN detector.

### 2.2 Camera and LiDAR were transformed at one timestamp

The former path treated camera and LiDAR samples as if they were captured at
the same pose. That creates a visible offset whenever the USV moves. The repair
uses exact historical transforms for both samples:

```text
LiDAR point at cloud stamp:  mid360_link -> map
Camera ROI at image stamp:   map -> camera_link
```

No latest-TF fallback is used. A missing historical transform drops that frame
and increments `tf_failures`.

### 2.3 Final box was shifted by shape completion

The old occlusion-completion step moved the final center away from the measured
surface. It was removed from this calibration path. The yellow box is now fitted
only to the accepted green ROI points. Shape completion can be added later as a
separate estimator, but must not alter geometric calibration evidence.

### 2.4 Distant camera identity plates were erased

The target's colored identity plate can occupy only 2-4 pixels at long range.
A 3x3 morphology kernel removed it. The camera detector now uses a 2x2 kernel
and de-duplicates overlapping affiliation candidates. It still detects from the
real image; it does not read target truth.

## 3. Verified calibration values

Live CameraInfo:

| Field | Value |
|---|---:|
| Resolution | 240 x 135 |
| fx / fy | 207.8935 / 207.8935 |
| cx / cy | 119.5 / 67.0 |
| Frame | `usv_01/camera_link` |

Live rigid mount transform:

```text
usv_01/base_link -> usv_01/camera_link
translation = [3.2400, 0.0000, 1.5500] m

usv_01/base_link -> usv_01/mid360_link
translation = [0.9075, 0.0000, 1.5625] m

usv_01/camera_link <- usv_01/mid360_link
translation = [-2.3325, 0.0000, 0.0125] m
rotation    = identity
```

Projection uses the Gazebo camera convention X-forward, Y-left, Z-up:

```text
u = cx - fx * Y / X
v = cy - fy * Z / X
```

The inverse projection used for the red camera frustum is the exact algebraic
inverse of these equations and has a unit round-trip test.

## 4. Qt calibration layers

The merged Perception Monitor keeps the existing visual style and adds three
independent, passive layers:

| Layer | Topic | Color | Meaning |
|---|---|---|---|
| Camera projection | `/perception/usv_01/vision_guided/camera_projection` | Red | Camera detection rectangle projected at measured LiDAR depth |
| LiDAR ROI cloud | `/perception/usv_01/vision_guided/roi_cloud` | Green | Real Mid-360 points selected by that image ROI |
| Final fused bbox | `/perception/usv_01/vision_guided/roi_bboxes` | Yellow | Box fitted to the accepted green points |

The normal LiDAR-only box remains sourced from:

```text
/perception/lv_dot_ros2/diagnostics/lidar_bboxes
```

USV-02 is lateral to USV-01 and outside the forward camera's useful field of
view, so it should receive a LiDAR-only box. This is correct behavior, not a
fusion failure. The fixed enemy vessel is used to inspect red/green/yellow
Camera-LiDAR overlap.

## 5. Live test result (2026-07-18)

Command:

```bash
source /opt/ros/humble/setup.bash
cd <your_UAV_USV_workspace>
source install/setup.bash
ros2 launch uav_usv_bringup camera_lidar_calibration_debug.launch.py
```

Measured results:

| Item | Result |
|---|---:|
| Mid-360 / LV-DOT cloud | 17.9 Hz, about 2750 points/frame |
| DBSCAN clusters | 1-3 depending on visible hull surfaces |
| LiDAR bbox output | 16-19 Hz |
| Camera projection markers | 17-18 Hz |
| ROI cloud | 18 Hz |
| Camera+LiDAR final bbox | 16-18 Hz |
| Historical cross-time TF frames | 1756 in sampled run |
| Camera-LiDAR accepted matches | 1290 in sampled run |
| Average synchronization difference | 68 ms in sampled run |
| Qt canvas | 25 FPS, about 1.7-2.0 ms/render |

USV-02 pose from its odometry was:

```text
(-15.929, 13.953, 0.553) m
```

The live DBSCAN marker centers included:

```text
(-16.444, 13.584, 1.333) m
(-13.855, 13.931, 1.122) m
```

The first center is approximately 0.6 m from the USV-02 base pose. Multiple
clusters can occur because the large vessel hull and superstructure are
spatially separated returns. The display option that keeps only the principal
target box can hide secondary hull clusters without changing detection.

## 6. Launch parameters

The live fleet launch exposes:

```text
lv_dot_input_max_range
lv_dot_local_range_x
lv_dot_local_range_y
target_speed
target_nominal_turn_rate
enable_sudden_turn
```

`camera_lidar_calibration_debug.launch.py` fixes the target by setting speed and
turn rate to zero and disabling the sudden-turn event. The normal fleet launch
retains its dynamic target defaults.

## 7. Verification

Completed checks:

```text
colcon build: 6 packages passed
colcon test:  86 tests, 0 errors, 0 failures
```

Runtime verification confirmed:

- raw Mid-360 cloud is real-time, not rosbag playback;
- USV-02 creates an upstream DBSCAN box automatically;
- CameraInfo and Camera-LiDAR mount TF are valid;
- historical cross-time TF lookup is active;
- red projection, green ROI points, and yellow measured box publish continuously;
- Qt does not manufacture boxes when upstream topics contain only DELETE markers;
- the source mux remains on ground truth and the control chain is untouched.

## 8. Known limitations

- A colored identity plate is not the same physical surface as the complete
  vessel hull. At long range, the plate center and mean LiDAR return can differ
  by several pixels even with correct calibration.
- Large scaled vessel visuals can produce separate hull/superstructure DBSCAN
  clusters. This should later be handled by semantic cluster merging, not by
  drawing a synthetic box.
- Exact historical TF deliberately drops occasional startup frames while the TF
  cache is filling.
