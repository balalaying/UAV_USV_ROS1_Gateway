# uav_usv_bringup

Ownership: system integration team.

This package is the stable user entry point. During migration its launch files include the original `uav_usv_sim` launch files, so existing behavior remains unchanged.

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
