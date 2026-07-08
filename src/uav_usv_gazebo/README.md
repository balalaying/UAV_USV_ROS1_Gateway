# uav_usv_gazebo

Ownership: simulation environment team.

This package owns the runnable maritime world, environmental models, weather,
coastline wrapper, and Gazebo system plugins. Control and mission nodes remain
in `uav_usv_sim`.

## Contents

- `worlds/default.sdf`: complete ocean environment with coastline, weather,
  fog, wind field, obstacles, moving vessels, and offshore facilities.
- `worlds/vrx_sydney_regatta_custom.sdf`: isolated VRX-style Sydney Regatta
  environment owned by this package, with the custom platform, reefs, harbor,
  lighthouses, buoys, and animated full-ocean wave surface.
- `plugins/BoatWaveFollower.cc`: wave-following motion for boats and floating objects.
- `plugins/DroneDeckFollower.cc`: parked-UAV deck attachment system.
- `config/sydney_coast.model.*`: local wrapper for the Sydney Regatta coastline.
- `models/simple_boat`: sensor-equipped USV and UAV landing deck.
- `models/waves`: animated Gerstner-wave surface.
- `models/medium_buoy` and `models/green_channel_buoy`: swaying channel marks.
- `models/target_vessel`: automatically moving maritime traffic vessel.

- `models/shore_platform`: collidable shoreline UAV helipad with an access pier.
- `models/rock_outcrop`: collidable marine rock cluster for obstacle courses.
- `models/green_channel_buoy`: illuminated starboard channel mark.
- `models/aquaculture_cage`: floating net pen with submerged net walls.
- `models/floating_barrel`: weathered oil-drum obstacle.
- `models/life_raft`: abandoned inflatable emergency raft.
- `models/driftwood`: floating logs and broken planks.
- `models/marina_pier`: illuminated T-head timber pier.
- `models/offshore_wind_turbine`: rotating offshore wind turbine with warning lights.
- `models/harbor_breakwater`: illuminated U-shaped harbor and concrete quay.
- `models/harbor_tug`: moving rescue tug with particle wake.
- `models/fishing_boat`: moving fishing vessel with outriggers and particle wake.
- `models/person_overboard`: floating casualty for rescue-perception tests.

## Run

```bash
source /opt/ros/jazzy/setup.bash
source /home/ssy/UAV_USV/install/setup.bash
ros2 run uav_usv_gazebo run_gz_world.sh
```

Run the isolated VRX-style world without changing the default world:

```bash
ros2 run uav_usv_gazebo run_gz_world.sh vrx_sydney_regatta_custom
```

PX4 asset synchronization is also owned by this package:

```bash
export PX4_DIR=/path/to/PX4-Autopilot
ros2 run uav_usv_gazebo sync_to_px4.sh
```

## 第一阶段任务

本包负责海面世界、动态目标船和标准测试场景。
