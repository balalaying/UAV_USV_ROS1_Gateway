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

This workspace currently targets ROS 2 Humble. If your team uses another ROS 2
distribution, make sure the Gazebo dependency versions in `CMakeLists.txt`
match your local installation.

```bash
cd ~/UAV_USV
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_usv_gazebo
source install/setup.bash
ros2 run uav_usv_gazebo run_gz_world.sh
```

Run the isolated VRX-style world without changing the default world:

```bash
ros2 run uav_usv_gazebo run_gz_world.sh vrx_sydney_regatta_custom
```

## VRX 水面实验副本

`worlds/heterogeneous_332_vrx_water.sdf` 是当前主世界
`worlds/heterogeneous_332.sdf` 的独立副本。副本使用 VRX Gazebo 8 的
Pierson–Moskowitz 波场、`WaveVisual` Gerstner 波面材质，以及 Gazebo 原生
Buoyancy / Hydrodynamics 系统；原主世界没有被修改。
VRX 实现来自 [Open Source Robotics Foundation 的 VRX 项目](https://github.com/osrf/vrx)，
与本工作区的 Gazebo Sim 8 ABI 对齐。

准备好 PX4 后，可直接启动完整的 3 UAV / 3 USV 预览：

```bash
source install/setup.bash
export PX4_DIR=/home/dji/PX4-Autopilot  # 按本机路径调整
ros2 run uav_usv_gazebo run_vrx_water_preview.sh
```

可用环境变量调整预览：`UAV_USV_X500_SCALE`（默认 `12`）、
`UAV_USV_CAMERA_WIDTH`（默认 `320`）、`UAV_USV_CAMERA_HEIGHT`（默认 `180`）、
`UAV_USV_CAMERA_RATE`（默认 `20`）和 `GZ_SIM_ARGS`（默认 `-r`）。

副本保留原有船舶控制和波浪跟随器，同时增加水动力阻尼，便于先观察视觉
和运行稳定性；若要做严格的物理水面实验，下一步可关闭旧的
`BoatWaveFollower` 位姿覆盖并重新标定船体浮力参数。

FFT 海面试验副本：

`worlds/heterogeneous_332_fft_water.sdf` 使用外部
`asv_wave_sim` 的 FFT / tiled ocean visual plugin。该方案支持当前的
Gazebo Harmonic (`gz-sim8`)，但项目许可证为 GPL-3.0，因此本仓库不复制
其源码或二进制；预览脚本从 `/tmp`（或环境变量指定的位置）加载它们。
当前工作区已在 `/tmp/asv_wave_sim` 和 `/tmp/asv_wave_install` 准备好试验版本；
如果换机器，需要先按 [asv_wave_sim 的 Harmonic 构建说明](https://github.com/srmainwaring/asv_wave_sim)
编译并安装该外部插件。
其构建依赖至少包括 `libfftw3-dev` 和 `libcgal-dev`。
预览脚本将 `DYNAMIC_TEXTURE` 切换为 `DYNAMIC_GEOMETRY` 并生成 5×5 瓦片，
这是当前 Ogre 2.3 环境下更稳定的 FFT 渲染路径。

```bash
export ASV_WAVE_ROOT=/tmp/asv_wave_sim
export ASV_WAVE_INSTALL=/tmp/asv_wave_install
source install/setup.bash
ros2 run uav_usv_gazebo run_fft_water_preview.sh
```

PX4 asset synchronization is also owned by this package:

```bash
export PX4_DIR=/path/to/PX4-Autopilot
ros2 run uav_usv_gazebo sync_to_px4.sh
```

The first run may download the Sydney Regatta coastline into
`/var/tmp/UAV_USV_gz_fuel` and generate local assets under
`/var/tmp/UAV_USV_assets`.

## 第一阶段任务

本包负责海面世界、动态目标船和标准测试场景。
