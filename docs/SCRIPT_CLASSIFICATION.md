# Script Classification

> Phase 1 inventory only. Source files are not moved or changed.

## Runtime

- Vehicle control: `uav_usv_uav_control/scripts/uav_*_fleet_agent.py`,
  `uav_usv_usv_control/scripts/usv_fleet_agent.py`.
- Mission/world state: `capture_manager.py`, `target_tracker.py`,
  `capture_target_motion.py`, `capture_visualizer.py`, `fleet_world_model_node.py`,
  `fleet_behavior_manager.py`, `fleet_simulated_agent.py`.
- Base station and gateway: `base_station_service_node.py`, `gateway_node.py`,
  `websocket_server.py`, `http_server.py`, `remote_relay.py`, `remote_uplink.py`.
- Sensor/perception runtime: `mid360_preprocessor.py`, `gz_pointcloud_bridge.py`,
  `uav_camera_adapter.py`, `uav_camera_tf.py`, `usv_camera_detection_node.py`,
  `camera_lidar_association_node.py`, `perception_fusion_node.py`, and
  `perception_source_mux.py`.

## Adapter

- `ground_truth_adapter.py`, `lv_dot_adapter.py`, `lv_dot_observation_adapter.py`,
  `lv_dot_ingress_relay.py`, `lv_dot_egress_relay.py`, `lv_dot_pose_adapter.py`,
  `tf_topic_relay.py`, `sensor_stream_adapter.py`, and `uav_visual_observation_node.py`.

## Fusion and Tracking

- `perception_fusion_node.py`, `camera_lidar_association_node.py`,
  `vision_guided_lidar_roi_node.py`, `track_association.py`, and
  `vision_guided_core.py`.

## Visualization

- `fleet_base_station_gui.py`, `fleet_base_station.py`,
  `lv_dot_debug_visualization_node.py`, `qt_pointcloud_projection_node.py`,
  `lv_dot_debug_visualization.py`, `perception_topdown.py`, and
  `uav_visual_shell_spawner.py`.

## Validation

- `scripts/evaluation/*`, test modules under package `test/` directories,
  `mid360_demo_motion.py`, `lv_dot_target_motion.py`, and validation launches
  listed in [LAUNCH_INDEX.md](LAUNCH_INDEX.md).

## Tools

- `uav_usv_gazebo/tools/{prepare_fleet_mid360.py,build_coastline_assets.py,
  prepare_coastline.sh,run_gz_world.sh,sync_to_px4.sh}`.
- `uav_usv_sim/tools/{prepare_large_x500.py,build_coastline_assets.py,
  prepare_coastline.sh,run_gz_world.sh,sync_to_px4.sh}`.

## Compatibility and Deprecated Candidates

- Compatibility: original `uav_usv_sim/scripts/{boat_nav2_interface.py,
  rviz_goal_boat_control.py,keyboard_boat_control.py,maritime_tf_publisher.py}`.
- Deprecated candidates: `cooperative_lighthouse_mission.py`,
  `uav_buoy_visual_mission.py`, `defense_demo.py`, and legacy demo-specific
  scripts. These remain runnable evidence and are not approved for deletion.

## Future Move Rule

Before moving a script to `archive/`, preserve its installed executable name,
update its CMake/setup entry, add a compatibility launcher if it is public, and
run its owning scenario plus the 332 smoke tests.
