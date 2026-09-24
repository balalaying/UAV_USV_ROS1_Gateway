# UAV_USV ROS1 Gateway

本仓库为 UAV/USV 异构协同系统的 **ROS1 Noetic 迁移与集成版本**，基于 **Ubuntu 20.04、Gazebo Sim 8、Gazebo Classic 11 和 ArduPilot SITL** 构建。

在原 UAV_USV 工程基础上，当前版本主要完成：

- ROS2 功能向 ROS1 Noetic 的迁移
- 3 UAV + 3 USV 异构协同仿真
- ArduPilot 多实例 SITL 接入
- UAV / USV 统一 FleetCommand 控制
- 护航、守卫、围捕等协同算法接入
- ROS 与前后端之间的 Fleet Gateway 双向通信
- WebSocket 舰队状态上报与控制指令转发
- 感知模块及部分真实传感器接入工具

当前主要运行链路：

```text
Frontend / Backend
        │
        │ WebSocket
        ▼
   Fleet Gateway
        │
        │ ROS1
        ▼
Mission / Algorithms
        │
        ▼
UAV / USV Control
        │
        ▼
  ArduPilot SITL
        │
        ▼
   Gazebo Sim 8
```

> 当前仓库以 **ROS1 Noetic + Gazebo Sim 8 + ArduPilot SITL + Fleet Gateway** 为主要运行链路，不再使用 PX4 或 ROS2 运行链路。

---

## 快速启动

进入 ROS1 工作区：

```bash
cd ~/uav_usv_ros1_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

启动 3 UAV + 3 USV 主仿真：

```bash
roslaunch uav_usv_bringup heterogeneous_332_qt_ros1.launch \
  start_ardupilot:=true \
  start_algorithms:=false
```

启动护航、守卫、围捕任务：

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

---

## Fleet Gateway

Fleet Gateway 位于：

```text
src/uav_usv_fleet_gateway/
```

Gateway 是 ROS 仿真系统与前后端之间的通信中间层，主要负责：

- UAV / USV 舰队状态汇总
- ROS 与前后端之间的 WebSocket 通信
- 舰队状态与任务状态实时推送
- 前后端控制指令接收
- 外部控制指令向 ROS FleetCommand 的转换
- ROS 仿真、协同算法与上层应用之间的双向通信

核心 Gateway 程序：

```text
src/uav_usv_fleet_gateway/scripts/fleet_gateway
```

主要 ROS 接口：

```text
/fleet/state
/fleet/command
/fleet/control_lease
/fleet/command_ack
```

通信关系：

```text
Frontend / Backend
        │
        │ WebSocket
        ▼
   Fleet Gateway
        │
        │ ROS Topic
        ▼
Mission / Algorithms
        │
        ▼
 /fleet/command
        │
        ▼
UAV / USV Agent
        │
        │ MAVLink
        ▼
  ArduPilot SITL
```

仅进行 Gazebo 仿真时，Gateway 可以不启动；进行前后端联调时，需要启动 Gateway。

---

## 六载具端口

| 载具 | 固件 | MAVLink TCP | JSON-FDM |
|---|---|---:|---:|
| `uav_01` | ArduCopter | 5760 | 9002 |
| `uav_02` | ArduCopter | 5770 | 9012 |
| `uav_03` | ArduCopter | 5780 | 9022 |
| `usv_01` | ArduRover | 5790 | 9032 |
| `usv_02` | ArduRover | 5800 | 9042 |
| `usv_03` | ArduRover | 5810 | 9052 |

所有模型必须保持：

```xml
<lock_step>0</lock_step>
```

---

## 主要模块

| ROS 包 | 功能 |
|---|---|
| `uav_usv_bringup` | ROS1 总启动与 ArduPilot 实例管理 |
| `uav_usv_gazebo` | Gazebo 世界、模型和控制桥 |
| `uav_usv_uav_control` | UAV FleetCommand 控制 |
| `uav_usv_usv_control` | USV FleetCommand 控制 |
| `uav_usv_cooperative_algorithms` | 护航、守卫、围捕等协同算法 |
| `uav_usv_fleet_gateway` | ROS 与前后端 WebSocket 双向通信 |
| `uav_usv_interfaces` | ROS1 消息与控制接口 |
| `uav_usv_mission` | 任务、世界模型及相关功能 |
| `uav_usv_perception` | 感知数据处理 |
| `uav_usv_lv_dot` / `uav_usv_lv_dot_core` | LV-DOT 感知链路 |
| `uav_usv_ros1_compat` | ROS1 兼容与迁移支持 |
| `uav_usv_base_station` | 岸基端相关功能 |

---

## 构建与检查

```bash
cd ~/uav_usv_ros1_ws

catkin_make

source devel/setup.bash

rospack profile
```

检查主 Launch：

```bash
roslaunch --files \
  uav_usv_bringup \
  heterogeneous_332_qt_ros1.launch
```

完整环境检查：

```bash
cd ~/uav_usv_ros1_ws/src/UAV_USV
bash scripts/check_environment.sh
```

理想情况下应显示：

```text
Failures : 0
Environment is ready.
```

---

## 从零复现

创建工作区并克隆仓库：

```bash
mkdir -p ~/uav_usv_ros1_ws/src
cd ~/uav_usv_ros1_ws/src

git clone https://github.com/balalaying/UAV_USV_ROS1_Gateway.git UAV_USV

cd UAV_USV
git checkout ros1-param-align-20260920
```

恢复系统与依赖环境：

```bash
bash scripts/setup_system.sh
source ~/.bashrc

bash scripts/setup_gz_harmonic.sh
source ~/.bashrc

bash scripts/setup_third_party.sh
```

编译与检查：

```bash
bash scripts/build_workspace.sh
bash scripts/check_environment.sh
```

最后加载工作区：

```bash
source ~/uav_usv_ros1_ws/devel/setup.bash
```

---

## 推荐环境

当前主要验证环境：

```text
Ubuntu 20.04.6 LTS
ROS1 Noetic
Gazebo Classic 11
Gazebo Sim 8
ArduPilot SITL
Python 3.8
CMake 3.27.9
GCC/G++ 9.4.0
```

其他 Ubuntu、ROS 或 Gazebo 版本尚未完整验证。

---

## 文档

- [`REPRODUCTION.md`](REPRODUCTION.md)：从零开始的完整项目复现说明
- [`MIGRATION.md`](MIGRATION.md)：开发环境迁移、依赖恢复与版本锁定
- [`ROS1_332_QT_GUIDE.md`](src/uav_usv_bringup/ROS1_332_QT_GUIDE.md)：ROS1 332 仿真详细运行说明
- [`docs/environment/`](docs/environment/)：原开发环境版本记录

---

## 第三方依赖

当前主要第三方依赖包括：

```text
third_party/ardupilot
third_party/ardupilot_gazebo
third_party/rapidjson
```

第三方依赖精确版本记录：

```text
third_party_patches/VERSIONS.txt
```

ArduPilot Gazebo 自定义修改：

```text
third_party_patches/ardupilot_gazebo_ArduPilotPlugin.patch
```

首次复现时不建议直接将这些依赖升级到最新版。

---

## 仓库信息

当前仓库：

```text
https://github.com/balalaying/UAV_USV_ROS1_Gateway.git
```

当前推荐 ROS1 分支：

```text
ros1-param-align-20260920
```

原项目仓库：

```text
https://github.com/Suu0129/UAV_USV.git
```

当前仓库主要面向：

```text
ROS1 Noetic
+
Gazebo Sim 8
+
ArduPilot SITL
+
Cooperative Algorithms
+
Fleet Gateway
```
