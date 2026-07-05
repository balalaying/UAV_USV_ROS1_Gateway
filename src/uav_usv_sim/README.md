# UAV_USV

Integrated UAV-USV simulation package.

For the common maritime messages, AIS simulator, dynamic target vessel, TF
layout, and COLREGs test scenes, see `docs/多源海事感知第一阶段.md`.

## Contents

Gazebo worlds, models, coastline configuration, and simulation plugins now
live in the dedicated `uav_usv_gazebo` package. This package keeps control,
mission, navigation, RViz, and compatibility launch files.

- `scripts/keyboard_boat_control.py`: keyboard teleop for `/model/simple_boat/cmd_vel`.
- `scripts/cooperative_lighthouse_mission.py`: MAVLink UAV + Gazebo USV cooperative lighthouse mission.
- `launch/colregs_test_scenario.launch.py`: head-on, crossing, and overtaking AIS test scenes.

## Build Outside /home

```bash
export UAV_USV_WS=/your/path/UAV_USV
export UAV_USV_INSTALL=/var/tmp/UAV_USV_install
export UAV_USV_BUILD=/var/tmp/UAV_USV_build
export UAV_USV_LOG=/var/tmp/UAV_USV_log

cd $UAV_USV_WS
source /opt/ros/humble/setup.bash
colcon --log-base $UAV_USV_LOG build \
  --build-base $UAV_USV_BUILD \
  --install-base $UAV_USV_INSTALL \
  --symlink-install
```

## Run Keyboard Control

```bash
source /opt/ros/humble/setup.bash
source $UAV_USV_INSTALL/setup.bash
ros2 run uav_usv_sim keyboard_boat_control
```

## Run Standalone Gazebo World

```bash
source $UAV_USV_INSTALL/setup.bash
ros2 run uav_usv_gazebo run_gz_world.sh
```

The first world launch downloads the official Sydney Regatta coastline from
Gazebo Fuel. Its roughly 137 MB cache is stored under `/var/tmp`, not `/home`:

```bash
export GZ_FUEL_CACHE_PATH=/var/tmp/UAV_USV_gz_fuel
export UAV_USV_ASSET_ROOT=/var/tmp/UAV_USV_assets
```

After the first successful download, the coastline can be loaded from the local
cache. See `docs/第三方资源说明.md` for source and license attribution.

The default world also contains a mild variable wind field, light linear sea
fog, animated waves and wind turbines, swaying channel marks, aquaculture
cages, floating debris, and two automatically moving traffic vessels.

## Run Gazebo World With Keyboard Control

```bash
source $UAV_USV_INSTALL/setup.bash
ros2 launch uav_usv_sim uav_usv_world_keyboard.launch.py
```

The launch starts Gazebo and opens keyboard boat control in a separate terminal.
Use `start_keyboard:=false` to start the world only.

## Run PX4 UAV-USV Cooperation Demo

Start PX4 + Gazebo with the UAV on the USV deck:

```bash
source /opt/ros/humble/setup.bash
source $UAV_USV_INSTALL/setup.bash
export PX4_DIR=/your/path/PX4-Autopilot
ros2 launch uav_usv_sim uav_usv_px4_sim.launch.py px4_dir:=$PX4_DIR
```

To start the UAV released on the shoreline helipad instead of attached to the
USV deck, use:

```bash
ros2 launch uav_usv_sim uav_usv_px4_sim.launch.py \
  px4_dir:=$PX4_DIR \
  spawn_on_shore_platform:=true
```

The default shoreline platform origin is `(165, 18)`. Its helipad center is
approximately `(159.666, 15.256)` and the landing surface is at `z=1.56 m`. Override
`shore_platform_pose` if the platform is moved.

In another terminal, start the cooperative mission:

```bash
source /opt/ros/humble/setup.bash
source $UAV_USV_INSTALL/setup.bash
ros2 launch uav_usv_sim cooperative_lighthouse_mission.launch.py
```

The mission uses MAVLink to command PX4 and Gazebo Transport to command the
boat. It takes off, lets the boat start moving toward the lighthouse at
`(35, 18)`, sends the UAV after a configurable delay, then reads the real-time
boat landing pad pose, descends onto it, and re-locks the UAV to the deck
follower constraint.

You can also start both launch files together:

```bash
ros2 launch uav_usv_sim uav_usv_cooperation_demo.launch.py \
  px4_dir:=$PX4_DIR \
  drone_depart_delay:=10.0 \
  takeoff_climb_rate:=0.8 \
  drone_cruise_speed:=2.0 \
  drone_deck_approach_speed:=1.2 \
  deck_land_altitude:=0.1 \
  deck_descent_rate:=0.35
```

## Sync Assets Back To PX4

```bash
source $UAV_USV_INSTALL/setup.bash
export PX4_DIR=/your/path/PX4-Autopilot
ros2 run uav_usv_gazebo sync_to_px4.sh
```

After syncing, PX4 can be started with:

```bash
cd $PX4_DIR
make px4_sitl gz_x500
```
