# 无人机-无人船协同仿真系统

## 项目介绍

本项目基于 **ROS 2 Humble、Gazebo Sim、PX4 和 Nav2**，用于研究无人机
（UAV）与无人船（USV）的协同控制、海上导航、环境感知和避碰决策。

目前已经实现：

- PX4 无人机起飞、巡航、追踪船体停机坪并降落
- 无人船与无人机协同前往灯塔
- 在 RViz 中发送目标点，使用 Nav2 和 MPPI 控制无人船导航
- 船载相机、LaserScan、TF、局部避障和目标点可视化
- AIS 动态目标、碰撞风险计算和 COLREGs 测试场景
- 海浪、灯塔、浮标、目标船等 Gazebo 海面环境

原始完整功能保留在 `uav_usv_sim`，不会在拆包过程中删除。其他 ROS 2 包用于
把仿真、模型、控制、感知和导航分开，方便多人同时开发。

## 并行开发方式

项目采用 **一个公开主仓库 + 每人一个 Fork + Pull Request** 的方式协作：

```text
项目主仓库 Suu0129/UAV_USV
          │
          ├── 成员 A Fork：开发海面和 Gazebo 环境
          ├── 成员 B Fork：开发船体和传感器模型
          ├── 成员 C Fork：开发 Nav2 和路径规划
          └── 成员 D Fork：开发雷达、相机和 AIS
                         ↓
                  提交 Pull Request
                         ↓
                项目负责人审核并合并
```

### 1. 每个人负责什么

| ROS 2 包 | 主要工作 |
|---|---|
| `uav_usv_gazebo` | 海面、海浪、灯塔、浮标、仿真世界和插件 |
| `uav_usv_description` | 船体、无人机、URDF、惯量和传感器安装位置 |
| `uav_usv_usv_control` | 无人船速度和执行器控制 |
| `uav_usv_uav_control` | PX4、MAVLink 和无人机控制 |
| `uav_usv_perception` | MID-360、相机、AIS、检测和目标融合 |
| `uav_usv_localization` | 里程计、定位、SLAM 和 TF |
| `uav_usv_navigation` | Nav2、MPPI、代价地图和路径规划 |
| `uav_usv_colregs` | DCPA、TCPA 和海上避碰规则 |
| `uav_usv_mission` | 无人机和无人船的协同任务 |
| `uav_usv_interfaces` | 所有模块共同使用的消息、服务和动作 |
| `uav_usv_bringup` | 总 launch、参数和系统集成 |
| `uav_usv_tests` | 接口测试和完整流程测试 |
| `uav_usv_sim` | 当前原始可运行版本，由负责人维护 |

每名成员主要修改自己负责的包。需要修改 Topic、消息、TF、总 launch 或
`uav_usv_sim` 时，先与项目负责人确认。

### 2. 成员开始开发

成员先在 GitHub 页面点击右上角 **Fork**，然后克隆自己账号下的仓库：

```bash
git clone https://github.com/你的用户名/UAV_USV.git
cd UAV_USV
git remote add upstream https://github.com/Suu0129/UAV_USV.git
```

同步主仓库并创建自己的功能分支：

```bash
git switch main
git fetch upstream
git merge --ff-only upstream/main
git push origin main
git switch -c feature/模块名-功能名
```

分支名示例：

```text
feature/gazebo-ocean-world
feature/description-boat-model
feature/perception-mid360
feature/navigation-dynamic-costmap
fix/navigation-heading-oscillation
```

开发完成后执行检查并推送：

```bash
./tools/check_workspace.sh
git add .
git commit -m "feat(perception): add MID-360 simulation"
git push -u origin feature/perception-mid360
```

最后在 GitHub 创建 Pull Request：

```text
目标仓库：Suu0129/UAV_USV
目标分支：main
来源仓库：成员自己的 Fork
来源分支：本次 feature 分支
```

项目负责人审核通过后再合并，成员不直接修改主仓库的 `main`。

详细规则见 [协作开发指南](COLLABORATION.md)，公共 Topic、消息、坐标系和 TF
见 [接口约定](docs/INTERFACES.md)。

当前五人团队的具体任务、两周安排和验收方法见
[第一阶段五人并行开发任务书](docs/第一阶段五人并行开发任务书.md)。

海上防御演示的启动方式、Qt 基站页面、动态参数和 Topic 说明见
[Defense 防御演示说明](docs/DEFENSE_README.md)。

## 运行项目

### 1. 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Sim 8
- Python 3 和 `pymavlink`
- PX4-Autopilot（运行无人机任务时需要）
- Nav2（运行无人船导航时需要）

完整依赖和故障排查见
[复现运行指南](src/uav_usv_sim/docs/复现运行指南.md)。

### 2. 克隆和编译

```bash
git clone https://github.com/Suu0129/UAV_USV.git
cd UAV_USV

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

每次打开新终端，都需要重新执行：

```bash
source /opt/ros/humble/setup.bash
source /你的路径/UAV_USV/install/setup.bash
```

### 2.1 可选：把 PX4 下载到本项目目录

如果电脑上还没有 PX4，可以使用项目提供的脚本下载到 `third_party`，不需要
手动到其它目录克隆：

```bash
./tools/setup_px4.sh
```

默认下载位置：

```text
third_party/PX4-Autopilot
```

然后在当前终端设置：

```bash
export PX4_DIR="$PWD/third_party/PX4-Autopilot"
```

如果这台电脑从未安装过 PX4 依赖，可以显式执行：

```bash
./tools/setup_px4.sh --install-system-deps
```

该命令会调用 PX4 自带的 Ubuntu 依赖安装脚本，可能需要输入 sudo 密码。
`third_party/PX4-Autopilot` 已加入 `.gitignore`，不会被提交到本仓库。

### 3. 启动海面世界

```bash
ros2 launch uav_usv_sim uav_usv_world_keyboard.launch.py
```

### 4. 启动 PX4 和协同任务

终端 1：

```bash
export PX4_DIR=/你的路径/PX4-Autopilot
ros2 launch uav_usv_sim uav_usv_px4_sim.launch.py \
  px4_dir:="$PX4_DIR"
```

终端 2：

```bash
source /opt/ros/humble/setup.bash
source /你的路径/UAV_USV/install/setup.bash
ros2 launch uav_usv_sim cooperative_lighthouse_mission.launch.py
```

也可以一条命令启动完整协同演示：

```bash
export PX4_DIR=/你的路径/PX4-Autopilot
ros2 launch uav_usv_sim uav_usv_cooperation_demo.launch.py \
  px4_dir:="$PX4_DIR"
```

### 5. 启动 RViz 和 Nav2 无人船导航

先启动 PX4 仿真世界，再在另一个终端启动：

```bash
ros2 launch uav_usv_sim boat_nav2_navigation.launch.py
```

在 RViz 顶部选择 `Nav2 Goal`，在栅格地图上单击并拖动，设置目标位置和朝向。

### 6. 启动 COLREGs 测试

```bash
ros2 launch uav_usv_sim colregs_test_scenario.launch.py
```

### 7. 启动无人机视觉浮标协同导航

终端 1 启动带下视相机的 PX4 无人机和仿真世界：

```bash
export PX4_DIR=/你的路径/PX4-Autopilot
ros2 launch uav_usv_sim uav_usv_px4_sim.launch.py \
  px4_dir:="$PX4_DIR"
```

终端 2 只启动船体 Nav2。它可以先接收人工目标并正常导航：

```bash
source /opt/ros/humble/setup.bash
source /你的路径/UAV_USV/install/setup.bash
ros2 launch uav_usv_sim boat_nav2_navigation.launch.py start_rviz:=false
```

终端 3 单独启动无人机巡逻、视觉识别、相机桥和 RViz：

```bash
source /opt/ros/humble/setup.bash
source /你的路径/UAV_USV/install/setup.bash
ros2 launch uav_usv_sim uav_buoy_patrol.launch.py
```

无人机将自动起飞并按航点巡逻。下视相机连续确认红色浮标后，节点发布浮标
世界坐标，并通过 `/goal_pose` 覆盖船体当前 Nav2 目标，发送距浮标约 7 米的
安全目标点。无人机飞到浮标上方，
船体使用静态海岸地图和 LaserScan 动态避障前往目标。RViz 同时显示船头相机、
无人机检测画面、目标标记、路径和代价地图。

也可以使用集成 launch 同时启动 Nav2 和无人机巡逻：

```bash
ros2 launch uav_usv_sim uav_buoy_cooperative_navigation.launch.py
```

主要话题：

| 话题 | 作用 |
|---|---|
| `/uav/down_camera/image` | 无人机下视相机原始画面 |
| `/uav/down_camera/detections` | 标注浮标检测结果的画面 |
| `/uav_usv/camera_mosaic` | RViz 默认显示的船头与无人机双画面 |
| `/uav/detected_target` | 无人机确认的浮标世界坐标 |
| `/goal_pose` | 发给船体 Nav2 的安全目标点 |
| `/uav/visual_target_marker` | RViz 中的视觉目标标记 |

## 项目负责人日常操作

审核并合并 Pull Request 后，在本机同步和检查：

```bash
cd /你的路径/UAV_USV
git switch main
git pull origin main
./tools/check_workspace.sh
```

提交项目负责人自己的修改：

```bash
git add .
git commit -m "说明本次修改"
git push origin main
```
