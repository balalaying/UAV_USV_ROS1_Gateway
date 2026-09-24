# Ubuntu 20.04 / ROS Noetic / heterogeneous_332 Qt 控制

本适配包保留仓库原 `fleet_base_station_gui.py` 界面，只把运行链路迁移到
ROS 1。运行时包含：

- `heterogeneous_332.sdf`（由仓库 `uav_usv_gazebo` 启动）；
- 三个 PX4 SITL 实例，对应 `uav_01`、`uav_02`、`uav_03`；
- 三个 Gazebo Twist 闭环控制器，对应 `usv_01`、`usv_02`、`usv_03`；
- Gazebo Harmonic 到 ROS 1 的 RGB、深度、LaserScan 桥；
- 基站相机六画面拼接、雷达显示、状态表和控制按钮。

## 1. 安装适配文件

在解压后的适配包根目录执行：

```bash
bash install_into_repo.sh ~/uav_usv_ros1_ws/src/UAV_USV
```

安装脚本只覆盖本适配涉及的包，并将原文件备份到仓库内带时间戳的目录。

## 2. 编译

```bash
cd ~/uav_usv_ros1_ws
source /opt/ros/noetic/setup.bash
catkin_make -j1 -l1 --pkg \
  uav_usv_interfaces \
  uav_usv_mission \
  uav_usv_uav_control \
  uav_usv_usv_control \
  uav_usv_cooperative_algorithms \
  uav_usv_bringup
source devel/setup.bash
```

继续使用 `-j1 -l1`，防止 16 线程、15 GiB 内存机器再次整机卡死。

## 3. 启动

建议先关闭原来手工启动的 Gazebo，然后一条命令启动完整链路：

```bash
roslaunch uav_usv_bringup heterogeneous_332_qt_ros1.launch
```

如果当前 `heterogeneous_332` 已在运行并且不想关闭：

```bash
roslaunch uav_usv_bringup heterogeneous_332_qt_ros1.launch start_world:=false
```

如暂时只验证船、相机和雷达，不启动 ArduPilot：

```bash
roslaunch uav_usv_bringup heterogeneous_332_qt_ros1.launch start_ardupilot:=false
```

ArduPilot 的大量输出不会刷屏，分别写入对应实例日志。

## 4. 检查

另开终端：

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_usv_ros1_ws/devel/setup.bash
rosrun uav_usv_bringup check_332_stack.sh
```

必须至少看到以下话题为 `OK`：

- `/fleet/state`
- `/fleet/command`
- `/fleet/base/camera_mosaic`
- `/fleet/base/radar/scan`

Qt 中点击“起飞”控制三架 UAV；输入协同目标并下发后，三架 UAV 与三艘
USV 都会收到目标；“暂停任务”和“紧急停止”会作用于全队。

## 5. 护航守卫 / GB-SFLA-CS 算法

主启动文件默认加载算法节点，但保持空闲。Qt 中先点击“启动护航守卫”，需要
转入围捕时点击“切换GBSFLACS围捕”。节点从 `/fleet/state` 和
`/fleet/world_model` 读取六个平台及敌方目标的实时状态，并通过现有
`/fleet/command` 下发航点。

也可以在启动时直接选择模式：

```bash
# 三维 GB-SFLA-CS 围捕
roslaunch uav_usv_bringup heterogeneous_332_qt_ros1.launch \
  algorithm_mode:=capture algorithm_auto_start:=true

# 护航守卫
roslaunch uav_usv_bringup heterogeneous_332_qt_ros1.launch \
  algorithm_mode:=escort algorithm_auto_start:=true
```

不希望加载算法节点时添加 `start_algorithms:=false`。仅观察算法、不实际下发
控制命令时，可单独启动：

```bash
roslaunch uav_usv_cooperative_algorithms dual_algorithms_ros1.launch \
  mode:=capture auto_start:=true publish_commands:=false
```

运行中可向 `/fleet/algorithm/action` 发布 `CAPTURE`、`ESCORT`、`PAUSE`、
`RESUME` 或 `STOP`。状态、分配结果和 RViz 航点分别发布到：

- `/fleet/algorithm/status`
- `/fleet/algorithm/assignments`
- `/fleet/algorithm/markers`

检查节点和接口：

```bash
rosrun uav_usv_cooperative_algorithms check_algorithms.sh
```

## 传感器话题

| 数据 | ROS 1 话题 |
|---|---|
| UAV 下视 RGB | `/fleet/uplink/uav_0X/camera/image_raw` |
| USV 前视 RGB | `/fleet/uplink/usv_0X/camera` |
| USV 深度图 | `/fleet/uplink/usv_0X/depth/image_raw` |
| USV 激光雷达 | `/fleet/uplink/usv_0X/scan` |
| Qt 雷达 | `/fleet/base/radar/scan` |
| 六画面拼接 | `/fleet/base/camera_mosaic` |

桥接节点会读取 `gz topic -l` 自动修正带世界名或嵌套模型名的 Gazebo 传感器
话题。如果 332 世界没有独立基地雷达，它会明确提示并用第一路 USV 扫描作为
Qt 雷达输入，界面不会保持空白。
