# Launch Index

> Phase 1 classification. This is metadata only: no launch implementation,
> argument, topic, TF, or executable has been changed.

## Safety Rule

The sole complete 332 entry is:

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture_live_perception.launch.py
```

Any launch marked **No** in the `Co-run with main` column must not be started at
the same time as the primary entry because it can duplicate Gazebo, PX4, DDS,
bridges, Fusion, the World Model, or Qt. A launch marked **Yes** is an
independent client/service or a deliberately standalone diagnostic; use its
arguments to avoid duplicate producers.

Legend: `GZ` Gazebo, `PX4` PX4/DDS, `P` perception, `Qt` Qt client, `WM` Fleet
World Model, `BS` Base Station Service.

## Primary and Deployment

| Path | Class | Purpose | GZ | PX4 | P | Qt | WM | BS | Co-run with main | Canonical successor |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|
| `uav_usv_bringup/launch/fleet_dynamic_capture_live_perception.launch.py` | primary | Complete 332 live-perception system | Y | Y | Y | Y | Y | Y | Yes, self | self |
| `uav_usv_bringup/launch/fleet_dynamic_capture.launch.py` | deployment | Core fleet capture composition | Y | Y | partial | optional | Y | optional | No | live perception main |
| `uav_usv_bringup/launch/simulation_332_scenario.launch.py` | simulation | 332 world/model inspection | Y | N | N | N | N | N | No | main or this for world-only work |
| `uav_usv_bringup/launch/dynamic_capture_console.launch.py` | base_station | Qt console/client composition | N | N | N | Y | N | optional | Conditional | main with `enable_console:=false`, or standalone client |
| `uav_usv_base_station/launch/base_station_service.launch.py` | base_station | Read-only Base Station Service | N | N | N | N | N | Y | Conditional | main with service disabled |

## Perception and Validation

| Path | Class | Purpose | GZ | PX4 | P | Qt | WM | BS | Co-run with main | Canonical successor |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|
| `uav_usv_lv_dot_ros2/launch/lv_dot_ros2.launch.py` | perception | Native LV-DOT lifecycle node | N | N | Y | N | N | N | Conditional | main-managed LV-DOT |
| `uav_usv_perception/launch/perception_layer.launch.py` | perception | Standard adapters, fusion and source mux | N | N | Y | N | N | N | Conditional | main-managed perception |
| `uav_usv_perception/launch/camera_lidar_fusion.launch.py` | perception | Camera-LiDAR fusion composition | N | N | Y | N | N | N | Conditional | main-managed fusion |
| `uav_usv_bringup/launch/mid360_sensor_demo.launch.py` | validation | Isolated Mid-360 sensor demo | Y | N | Y | optional | N | N | No | main or sensor validation |
| `uav_usv_bringup/launch/camera_lidar_calibration_debug.launch.py` | validation | Camera/LiDAR geometric calibration debugging | usually Y | N | Y | optional | N | N | No | main with selective nodes disabled |
| `uav_usv_perception/launch/lv_dot_shadow.launch.py` | validation | ROS1/ROS2 LV-DOT Shadow validation | N | N | Y | N | N | N | No | main-managed Shadow chain |
| `uav_usv_perception/launch/lv_dot_tuning.launch.py` | validation | LV-DOT tuning and bag/runtime diagnostics | optional | N | Y | optional | N | N | No | LV-DOT validation workflow |
| `uav_usv_perception/launch/lv_dot_fusion_validation.launch.py` | validation | LV-DOT/Fusion comparison | N | N | Y | N | N | N | No | perception validation workflow |
| `uav_usv_perception/launch/multisensor_fusion_validation.launch.py` | validation | Multi-sensor Shadow validation | N | N | Y | N | N | N | No | perception validation workflow |

## Gateway and Remote Clients

| Path | Class | Purpose | GZ | PX4 | P | Qt | WM | BS | Co-run with main | Canonical successor |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|
| `uav_usv_fleet_gateway/launch/remote_sensor_dashboard.launch.py` | base_station | Gateway plus remote sensor web dashboard | optional | N | adapter | N | reads | reads | Conditional | Base Station/gateway deployment |
| `uav_usv_fleet_gateway/launch/remote_summary_gateway.launch.py` | base_station | Read-only fleet summary gateway | N | N | N | N | reads | reads | Yes, when port/topic unique | remote dashboard gateway |
| `uav_usv_fleet_gateway/launch/mobile_fleet_demo.launch.py` | compatibility | Legacy mobile fleet web demo | N | N | N | N | reads | optional | Conditional | remote sensor dashboard |

## Historical Capture, Defense and Qt Entrances

| Path | Class | Purpose | GZ | PX4 | P | Qt | WM | BS | Co-run with main | Canonical successor |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|
| `uav_usv_bringup/launch/minimal_dynamic_capture.launch.py` | compatibility | Single-UAV/USV minimal capture | Y | Y | limited | optional | limited | N | No | primary main |
| `uav_usv_bringup/launch/dual_uav_dynamic_capture.launch.py` | compatibility | Two-UAV capture framework test | Y | Y | limited | optional | limited | N | No | primary main |
| `uav_usv_bringup/launch/cooperative_capture.launch.py` | deprecated | Earlier cooperative capture demo | Y | Y | legacy | optional | N | N | No | primary main |
| `uav_usv_bringup/launch/cooperative_response.launch.py` | deprecated | Earlier response mission demo | Y | Y | legacy | optional | N | N | No | primary main |
| `uav_usv_bringup/launch/defense.launch.py` | deprecated | Defense wrapper | Y | optional | legacy | optional | legacy | N | No | retained historical scenario |
| `uav_usv_bringup/launch/defense_sim.launch.py` | deprecated | Standalone defense Gazebo demo | Y | N | legacy | optional | legacy | N | No | retained historical scenario |
| `uav_usv_bringup/launch/All_Qt.launch.py` | compatibility | Earlier multi-vehicle Qt demo | optional | optional | legacy | Y | N | N | No | dynamic console/main |
| `uav_usv_bringup/launch/Qt_cooperation.launch.py` | compatibility | Earlier Qt cooperation demo | optional | optional | legacy | Y | N | N | No | dynamic console/main |
| `uav_usv_bringup/launch/unified_tasks_qt.launch.py` | deprecated | Historical unified task Qt launcher | Y | optional | legacy | Y | N | N | No | primary main |

## Original `uav_usv_sim` Compatibility Entrances

| Path | Class | Purpose | GZ | PX4 | P | Qt | WM | BS | Co-run with main | Canonical successor |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|
| `uav_usv_sim/launch/uav_usv_px4_sim.launch.py` | compatibility | Original PX4 + Gazebo UAV/USV sim | Y | Y | basic | N | N | N | No | primary main |
| `uav_usv_sim/launch/uav_usv_cooperation_demo.launch.py` | compatibility | Original UAV-USV cooperation demo | Y | Y | basic | optional | N | N | No | primary main |
| `uav_usv_sim/launch/boat_nav2_navigation.launch.py` | compatibility | Original USV Nav2/MPPI navigation | optional | N | lidar | RViz | N | N | No | primary main or standalone Nav2 test |
| `uav_usv_sim/launch/rviz_goal_boat_tracking.launch.py` | compatibility | RViz goal-based boat tracking | optional | N | lidar | RViz | N | N | No | standalone Nav2 test |
| `uav_usv_sim/launch/colregs_test_scenario.launch.py` | validation | COLREGs/AIS scenario test | Y | N | AIS/lidar | RViz | N | N | No | future `uav_usv_colregs` workflow |
| `uav_usv_sim/launch/cooperative_lighthouse_mission.launch.py` | deprecated | Lighthouse mission | Y | Y | basic | N | N | N | No | primary main |
| `uav_usv_sim/launch/uav_buoy_cooperative_navigation.launch.py` | deprecated | Buoy cooperation/navigation demo | Y | optional | basic | RViz | N | N | No | primary main |
| `uav_usv_sim/launch/uav_buoy_patrol.launch.py` | deprecated | Buoy patrol demo | optional | optional | basic | N | N | N | No | primary main |
| `uav_usv_sim/launch/uav_usv_world_keyboard.launch.py` | compatibility | Manual keyboard world-control demo | Y | optional | N | N | N | N | No | standalone manual test |

## Metadata Application Plan

The primary launch now carries an inline `REFRACTOR_METADATA` header. All other
launches are classified by this index in Phase 1 to avoid broad nonfunctional
churn. In Phase 2, add the same header to remaining launch files in small,
package-scoped commits while preserving their first executable statement and
syntax.
