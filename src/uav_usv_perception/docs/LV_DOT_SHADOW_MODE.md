# LV-DOT Shadow Mode

This stage keeps `/fleet/perception/targets` on `ground_truth`. LV-DOT runs as
an isolated observer and cannot command a vehicle or replace the capture input
unless an operator explicitly changes `perception_source_mux`.

## Data flow

```text
ROS 2 Humble host
  /perception/usv_01/points_filtered  PointCloud2
  /perception/lv_dot/usv_01/pose      PoseStamped
  /fleet/uplink/uav_01/camera/image_raw
  /fleet/uplink/uav_01/camera/camera_info
                |
                | bounded latest-message TCP ingress (port 19090)
                v
ROS 1 Noetic container
  /lv_dot/input/usv_01/points_filtered
  /lv_dot/input/usv_01/pose
  /lv_dot/input/uav_01/camera/image_raw
  /lv_dot/input/uav_01/camera/camera_info
  pinned LV-DOT detector (449bf2c)
                |
                | MarkerArray dynamic boxes, velocity labels and diagnostics
                | whitelisted latest-message TCP egress (port 19091)
                v
ROS 2 Humble host
  lv_dot_adapter
    -> /perception/lv_dot/observations  TrackedObjectArray
  lv_dot_shadow_evaluator
    -> /perception/lv_dot/shadow_metrics  std_msgs/String JSON
  perception_fusion_node
    -> /perception/fused/tracks
  perception_source_mux
    -> /fleet/perception/targets
```

The two narrow TCP relays avoid mixing the ROS 1 and ROS 2 discovery graphs.
Ingress transports only `PointCloud2`, `PoseStamped`, `Image`, and
`CameraInfo`; egress transports only dynamic-box, velocity, and read-only
pipeline diagnostic `MarkerArray` data. Neither relay
exposes commands, services, custom fleet messages, or control topics.
Container-side inputs use the private `/lv_dot/input/...` prefix, so an input
cannot feed back into its host sensor topic.

The upstream package is not copied into this repository. The image build
clones the audited commit into `/opt/lv_dot_ws` and builds it with catkin.

## Important sensor boundary

LV-DOT assumes that its camera, LiDAR, and pose belong to one rigid body. In
this project the Mid-360 is on `usv_01`, while the auxiliary camera is on
`uav_01`. Their raw measurements must not be combined using one static
body-to-sensor transform.

The first isolated backend therefore runs the native **USV LiDAR path**. The
UAV image topic is relayed and monitored as a future auxiliary source, but it
does not set the camera or fused bits in `TrackedObject.source_mask`. A future
UAV detector must produce its own map-frame observations; fleet fusion can
then associate those observations with the USV LV-DOT tracks.

The standard UAV `CameraInfo` is preserved at the isolation boundary. Native
LV-DOT still reads its color intrinsics from `detector_param.yaml`, so the two
sets of intrinsics must be checked before enabling a camera backend.

## Build the isolated backend

Docker is intentionally outside the ROS 2 workspace. With a normal system
Docker installation:

```bash
cd /path/to/UAV_USV
tools/lv_dot/build_isolated.sh
```

The image is pinned as:

```text
uav-usv/lv-dot-noetic:449bf2c
```

The build installs dependencies missing from the upstream package manifest,
including `tf2_geometry_msgs` and `gazebo_msgs`; it does not patch LV-DOT.

## Run

Start the simulation and sensor adapters first. Keep the mux on ground truth:

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture.launch.py \
  perception_source:=ground_truth enable_mid360:=true
```

In a second terminal start the ROS 2 Shadow nodes:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_usv_perception lv_dot_shadow.launch.py
```

In a third terminal start the isolated ROS 1 detector:

```bash
tools/lv_dot/run_isolated.sh
```

For the repeatable one-UAV, one-USV tuning scene use:

```bash
ros2 launch uav_usv_perception lv_dot_tuning.launch.py \
  target_profile:=constant start_rviz:=true
tools/lv_dot/run_isolated.sh
tools/lv_dot/record_tuning_bag.sh bags/lv_dot_constant
```

`target_profile` accepts `constant`, `turn`, and `acceleration`. The tuning
launch always leaves `perception_source` on `ground_truth`.

Set the same non-default ingress and egress ports on both sides when needed:

```bash
ros2 launch uav_usv_perception lv_dot_shadow.launch.py \
  lv_dot_ingress_port:=19190 lv_dot_egress_port:=19191
LV_DOT_INGRESS_PORT=19190 LV_DOT_EGRESS_PORT=19191 \
  tools/lv_dot/run_isolated.sh
```

## Verify

```bash
ros2 topic hz /perception/lv_dot/observations
ros2 topic echo /perception/lv_dot/observations --once
ros2 topic echo /perception/lv_dot/shadow_metrics
ros2 param get /perception_source_mux perception_source
```

Expected default:

```text
perception_source = ground_truth
```

For a short operator-controlled comparison only:

```bash
ros2 param set /perception_source_mux perception_source sensor
ros2 param set /perception_source_mux perception_source ground_truth
```

The first command must never be used as an automatic fallback. If LV-DOT is
offline, `sensor` mode can legitimately publish no target.

## Output semantics

Upstream exposes visualization markers rather than a structured track topic.
`lv_dot_adapter` therefore supplies the missing contract as follows:

| Field | Source |
| --- | --- |
| `track_id` | nearest-neighbour association maintained by the adapter |
| `position` and dimensions | upstream dynamic box marker |
| `velocity` | upstream velocity marker, or finite difference fallback |
| `confidence` | configurable adapter default, currently 0.70 |
| `source_mask` | LiDAR in the current backend |
| `classification` | unknown; no maritime classifier is active |
| covariance | configurable conservative diagonal values |

These synthesized fields are suitable for Shadow evaluation, not yet a claim
of production perception quality.

## Shadow metrics

`/perception/lv_dot/shadow_metrics` contains:

- ground-truth and LV-DOT online state;
- nearest-track position and velocity error;
- rolling detection rate;
- ID-switch-based track stability;
- message latency;
- observation and sample counts.
- LiDAR, filtered, and tracked box counts for pipeline diagnosis;
- non-empty observation frequency.

The Qt `Perception Monitor` page subscribes only to this lightweight JSON. It
does not decode images or point clouds in the GUI thread.

## Current limitations

- Native LV-DOT has a hard-coded local LiDAR region of about 10 x 10 x 5 m.
- The upstream maritime class is not implemented; output class is unknown.
- The current backend has no USV RGB-D camera, so native depth fusion is off.
- UAV camera support requires an independent detector and map-frame projection.
- Marker-derived IDs and confidence are adapter estimates.
- The upstream ROS 1 package has no lifecycle or health interface.
