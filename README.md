# UAV_USV ROS1

当前仓库是 Ubuntu 20.04 / ROS1 Noetic / Gazebo Harmonic / ArduPilot SITL
环境，不再包含 PX4 或 ROS2 运行链路。

## 主仿真

```bash
cd ~/uav_usv_ros1_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch uav_usv_bringup heterogeneous_332_qt_ros1.launch \
  start_ardupilot:=true \
  start_algorithms:=false
```

护航、守卫、围捕任务：

```bash
roslaunch uav_usv_bringup gbsflacs_332_sim.launch \
  start_gui:=true \
  enable_mid360:=false
```

关闭整套仿真：

```bash
cd ~/uav_usv_ros1_ws/src/UAV_USV
bash src/uav_usv_bringup/scripts/stop_332_stack.sh
```

## 六载具端口

| 载具 | 固件 | MAVLink TCP | JSON-FDM |
|---|---|---:|---:|
| uav_01 | ArduCopter | 5760 | 9002 |
| uav_02 | ArduCopter | 5770 | 9012 |
| uav_03 | ArduCopter | 5780 | 9022 |
| usv_01 | ArduRover | 5790 | 9032 |
| usv_02 | ArduRover | 5800 | 9042 |
| usv_03 | ArduRover | 5810 | 9052 |

所有模型必须保持 `<lock_step>0</lock_step>`。FleetCommand 使用：

- `/fleet/state`
- `/fleet/command`
- `/fleet/control_lease`
- `/fleet/command_ack`

## 构建与检查

```bash
cd ~/uav_usv_ros1_ws
catkin_make
source devel/setup.bash
rospack profile
roslaunch --files uav_usv_bringup heterogeneous_332_qt_ros1.launch
```

详细运行说明见
[`src/uav_usv_bringup/ROS1_332_QT_GUIDE.md`](src/uav_usv_bringup/ROS1_332_QT_GUIDE.md)。

## 主要包

- `uav_usv_bringup`：ROS1 启动与 ArduPilot 实例管理
- `uav_usv_gazebo`：Gazebo 世界、模型和控制桥
- `uav_usv_uav_control`：ArduCopter FleetCommand agent
- `uav_usv_usv_control`：ArduRover FleetCommand agent
- `uav_usv_cooperative_algorithms`：护航、守卫与 GBSFLACS 控制
- `uav_usv_interfaces`：ROS1 消息接口
- `uav_usv_mission`：世界模型、岸基服务和 Qt 界面
- `uav_usv_perception`、`uav_usv_lv_dot*`：ROS1 感知链路

`third_party/ardupilot`、`third_party/ardupilot_gazebo` 和
`third_party/rapidjson` 是当前仿真依赖，不应删除。
