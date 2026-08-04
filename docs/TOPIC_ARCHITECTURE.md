# Topic Architecture

> Audit baseline, 2026-08-01. This document records the current intended
> interfaces. It does not rename, remap, publish, or remove any topic.

## Coordinate Rule

`map` is the fleet-level coordinate frame. All fleet state, fused target and
Base Station displays use map coordinates. Sensor-native messages retain their
own `camera_link` or `mid360_link` frame and are transformed by the appropriate
adapter/fusion layer, not by a display client.

## Interface Layers

| Layer | Principal topics | Owner | Notes |
|---|---|---|---|
| Vehicle state and command | `/fleet/state`, `/fleet/command`, `/fleet/command_ack`, `/fleet/control_lease` | vehicle agents / mission | Stable FleetCommand chain; task nodes do not directly command Gazebo or PX4 low-level topics. |
| Raw uplink sensors | `/fleet/uplink/<vehicle_id>/mid360/points`, `/fleet/uplink/<vehicle_id>/camera/image_raw`, `/fleet/uplink/<vehicle_id>/camera/camera_info` | Gazebo bridges/adapters | Vehicle-scoped source data. |
| Sensor health | `/fleet/sensor_status` | sensor adapters | Health, rate, latency and online state only. |
| Processed per-USV point cloud | `/perception/<usv_id>/mid360/points_filtered` | mid360 preprocessor | Standard LV-DOT and visualization input. |
| LV-DOT diagnostics | `/perception/lv_dot_ros2/diagnostics/*`, `/perception/lv_dot_ros2/tracks`, `/perception/lv_dot_ros2/dynamic_tracks` | native LV-DOT ROS 2 | Diagnostic tracks remain Shadow inputs; no direct control use. |
| Standard observations | `/perception/lv_dot/observations`, `/perception/<vehicle_id>/observations` | observation adapters | Sensor-agnostic `TrackedObjectArray` form. |
| Fleet fusion | `/perception/fused/tracks`, `/fleet/perception/usv_tracks`, `/fleet/perception/fused_targets` | perception fusion | Map-frame aggregate products. |
| Source selection | `/fleet/perception/targets` | perception source mux | Stable task-layer target input; default source is `ground_truth`. |
| Mission state | `/capture/state`, `/capture/roles`, `/capture/target_status`, `/capture/markers` | capture/visualization nodes | Preserve existing state and role interfaces. |
| World model | `/fleet/world_model`, `/fleet/world_model/summary` | fleet world model | Canonical fleet situation output. |
| Base station | `/base_station/state`, `/base_station/events` | Base Station Service | Read-only client boundary and base reference. |
| Web gateway | WebSocket payloads sourced from fleet/base-station caches | fleet gateway | Web clients do not subscribe to ROS topics directly. |

## Namespace Convention

- Vehicle IDs are lower-case ROS names: `uav_01` through `uav_03`, `usv_01`
  through `usv_03`, `friendly_ship`, and `enemy_ship`.
- Display labels may be localized, but topic and frame IDs remain stable ASCII.
- A sensor topic belongs beneath its vehicle identity; fleet-wide products belong
  beneath `/fleet` or `/perception`.
- New functionality must publish a standard observation before it participates
  in fusion. It must not create a parallel task-control topic.

## TF Inventory Rule

The intended chain is:

```text
map
  -> <vehicle_id>/base_link
       -> <vehicle_id>/mid360_link
       -> <vehicle_id>/camera_link
       -> <vehicle_id>/depth_camera_link   (when present)
```

`base_station` is a reference entity managed by the Base Station Service. It
does not replace `map` or introduce a second fleet world frame.

## Duplicates Requiring Later Review

This audit found historical `/maritime/*` documentation and several demo-only
sensor/mission paths. They are not renamed here. In a later migration, every
candidate alias must be classified as one of: canonical, compatibility alias,
or deprecated, with launch and bag regression evidence before removal.
