# uav_usv_bringup

Ownership: system integration team.

This package is the stable user entry point. During migration its launch files include the original `uav_usv_sim` launch files, so existing behavior remains unchanged.

## Frozen 332 Main Entry

`fleet_dynamic_capture_live_perception.launch.py` is the **only complete 332
integrated entry point**. It composes the 332 Gazebo world, 3 UAV, 3 USV, PX4,
Mid360, Camera, LV-DOT, Camera-LiDAR Fusion, Fleet Perception Fusion, Fleet
World Model, Base Station Service, and Qt/client chain without changing their
individual interfaces.

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture_live_perception.launch.py
```

演示电脑资源紧张时，推荐分成两个终端错峰启动。第一步先让 Gazebo、
PX4、Nav2 和任务核心稳定，第二步再挂接 USV_01 的 Mid-360 感知与 Qt：

```bash
# 终端 1
ros2 launch uav_usv_bringup sim.launch.py

# 终端 2（Gazebo 画面出现后再执行）
ros2 launch uav_usv_bringup Qt_base.launch.py
```

分段入口默认只处理 `USV_01` 的 Mid-360，不改变 332 载具、
摄像头画面、PX4/Nav2 或围捕控制链。需要恢复三船雷达时，
在核心入口显式传入
`mid360_vehicle_ids:=usv_01,usv_02,usv_03`。

Do not start it together with historical launches that also start Gazebo, PX4,
bridges, Fusion, or Qt. The complete classification, safe co-run decision, and
successor for every launch are in [docs/LAUNCH_INDEX.md](../../docs/LAUNCH_INDEX.md).
This Phase 1 marker changes no launch arguments or runtime behavior.

基础仿真、Nav2 和 COLREGs 测试仍由 `uav_usv_sim` 提供，避免在
`uav_usv_bringup` 中维护重复 launch：

- `ros2 launch uav_usv_sim uav_usv_px4_sim.launch.py`
- `ros2 launch uav_usv_sim boat_nav2_navigation.launch.py`
- `ros2 launch uav_usv_sim colregs_test_scenario.launch.py`

## 基站集中控制演示

命名约定：

- `Qt_cooperation.launch.py`：启动单主控链路的 Qt 协同基站，默认只显示船01和无人机01。
- `All_Qt.launch.py`：启动三组船机 Qt 基站，额外显示船02/03和无人机02/03。
- `Qt_base_station.rviz`：Qt 基站配套 RViz 可视化配置。

终端 1 启动 Gazebo、PX4、无人机和无人船：

```bash
ros2 launch uav_usv_sim uav_usv_px4_sim.launch.py
```

终端 2 启动 Nav2、载具代理、基站和 Qt 控制台：

```bash
ros2 launch uav_usv_bringup Qt_cooperation.launch.py
```

三组船机 Qt 基站入口：

```bash
ros2 launch uav_usv_bringup All_Qt.launch.py
```

基站自动取得 UAV、USV 的控制租约，命令无人机起飞，并命令无人机和
无人船前往共同目标。Qt 中的融合画面不是直接读取 Gazebo，而是订阅基站
接收并重新发布的数据。

主要基站数据接口：

| Topic | 内容 |
|---|---|
| `/fleet/base/camera_mosaic` | 船首相机与无人机下视相机融合画面 |
| `/fleet/base/usv_scan` | 基站接收到的船载 LaserScan |
| `/fleet/sensor_status` | 每个传感器的频率、延迟、消息数、字节数和健康状态 |
| `/fleet/state` | UAV、USV 在线状态、位姿和当前任务 |
| `/fleet/base/markers` | 载具、目标和传感器状态可视化 |
| `/fleet/command_ack` | 命令接受、执行、成功或失败反馈 |

不执行自动任务、只观察数据：

```bash
ros2 launch uav_usv_bringup Qt_cooperation.launch.py \
  auto_demo:=false
```

Qt 是默认基站界面。需要同时打开 RViz：

```bash
ros2 launch uav_usv_bringup Qt_cooperation.launch.py \
  start_rviz:=true
```

只运行后端、不打开任何图形界面：

```bash
ros2 launch uav_usv_bringup Qt_cooperation.launch.py \
  start_gui:=false start_rviz:=false
```
