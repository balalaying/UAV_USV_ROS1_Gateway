# Defense 防御演示说明

这个文档说明 `uav_usv_bringup` 中的海上防御演示。该演示用于验证：

- 多艘我方无人船在大本营周围巡逻和防守
- 多艘敌方船向大本营进攻
- 我方船根据敌方船和大本营连线计算防守点
- Qt 基站集中显示相机、雷达、载具状态和防御态势
- Qt 滑块实时调整防御算法参数

## 1. 启动方式

先编译并 source 工作区：

```bash
cd /你的路径/UAV_USV
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

启动完整防御演示：

```bash
ros2 launch uav_usv_bringup defense_sim.launch.py
```

兼容旧命令：

```bash
ros2 launch uav_usv_bringup defense.launch.py
```

`defense.launch.py` 只是兼容入口，内部会转到 `defense_sim.launch.py`。

## 2. 启动后会出现什么

默认会启动：

| 窗口 / 节点 | 作用 |
|---|---|
| Gazebo Sim | 显示空旷海域、灯塔、大本营平台、四艘我方船、四艘敌方船、四架无人机 |
| RViz | 显示防御圈、预警圈、防守点、船体 marker 和路径线 |
| Qt 基站 | 显示实时视频、LaserScan、载具在线状态、防御态势和参数滑块 |
| `defense_sim_demo` | 防御行为算法节点 |
| `defense_sim_base_station` | 基站数据聚合节点 |
| `defense_sim_sensor_agent` | 模拟传感器上行和载具状态 |
| camera bridge | 将 Gazebo 相机图像桥接到 ROS 2 |

## 3. Qt 基站页面

Qt 基站目前分为几个页面：

| 页面 | 内容 |
|---|---|
| 总览 | 传感器在线状态、载具在线状态 |
| 实时感知 | 四艘船首相机、四架无人机下视相机、船载 LaserScan |
| 防御任务 | 防御态势图、参数滑块、防御任务状态 |
| 基站控制 | 协同目标点、无人机起飞、保持、急停、命令回执 |

实时感知页中的“船载激光雷达局部视图”不是全局地图，而是船体周围的局部扫描：

- 中心黄点：当前船体
- 黄色短线：船头方向
- 青色回波：雷达扫到的障碍物、岸线或其它船
- 圆圈：距离本船的半径刻度

## 4. 防御算法逻辑

核心思想：

1. 敌方船默认向大本营移动。
2. 当敌方船进入预警半径 `trigger_radius`，我方船进入防守模式。
3. 对每艘敌方船，算法从大本营指向敌方船画一条线。
4. 以大本营为圆心，以 `defend_radius` 为半径画防守圈。
5. 这条线和防守圈的交点就是主要防守点。
6. 多艘我方船会分配到不同防守点，并用 `guard_spacing` 拉开间距。
7. 当我方船到达防守点附近，且敌方船也接近防守点，敌方船会被判定为拦停。
8. 如果我方船和敌方船距离足够近，也会触发近距离拦截。

防御行为代码位于：

```text
src/uav_usv_mission/scripts/defense_demo.py
```

## 5. Qt 滑块参数

防御任务页的滑块会实时写入 ROS 2 参数服务：

```text
/defense_sim_demo/set_parameters
```

主要参数含义：

| 参数 | 作用 | 调大后的效果 |
|---|---|---|
| `defend_radius` | 防守圈半径 | 我方船在离大本营更远的位置拦截 |
| `trigger_radius` | 预警半径 | 敌船更早触发防御逻辑 |
| `own_guard_speed` | 我方防守速度 | 我方船更快到达防守点，但可能更急 |
| `enemy_speed` | 敌方进攻速度 | 敌方船更快逼近大本营 |
| `guard_stop_distance` | 我方船到防守点的判定距离 | 更容易认为我方船已到位 |
| `enemy_guard_stop_distance` | 敌方船接近防守点的判定距离 | 敌方船更早被防守点拦停 |
| `intercept_stop_distance` | 近距离拦截距离 | 我方船靠近敌方船时更容易拦停 |
| `guard_spacing` | 多个防守点间距 | 我方船之间分得更开 |

也可以在终端实时修改，例如：

```bash
ros2 param set /defense_sim_demo defend_radius 85.0
ros2 param set /defense_sim_demo enemy_speed 5.0
ros2 param set /defense_sim_demo own_guard_speed 18.0
```

## 6. 常用 Topic

| Topic | 内容 |
|---|---|
| `/defense/status` | 防御状态、半径、目标点、敌方状态 |
| `/defense/own_ships` | 我方船 PoseArray |
| `/defense/enemy_ships` | 敌方船 PoseArray |
| `/defense/rviz_markers` | RViz 防御可视化 marker |
| `/fleet/base/camera_mosaic` | 基站融合后的相机画面 |
| `/fleet/base/usv_scan` | 基站接收到的船载 LaserScan |
| `/fleet/sensor_status` | 传感器频率、延迟、消息数、健康状态 |
| `/fleet/state` | 船和无人机在线状态、位姿和任务状态 |
| `/fleet/command_ack` | 基站命令反馈 |

查看防御状态：

```bash
ros2 topic echo /defense/status
```

查看相机融合画面频率：

```bash
ros2 topic hz /fleet/base/camera_mosaic
```

## 7. 文件结构

| 文件 / 目录 | 作用 |
|---|---|
| `src/uav_usv_bringup/launch/defense_sim.launch.py` | 完整防御演示启动入口 |
| `src/uav_usv_bringup/launch/defense.launch.py` | 兼容旧入口，转发到 `defense_sim.launch.py` |
| `src/uav_usv_bringup/rviz/defense.rviz` | RViz 防御可视化配置 |
| `src/uav_usv_bringup/config/gazebo_white_gui.config` | Gazebo GUI 配置 |
| `src/uav_usv_gazebo/worlds/open_ocean_defense_sim.sdf` | 空旷海域防御世界 |
| `src/uav_usv_gazebo/models/defense_own_*_boat` | 四艘我方船模型 |
| `src/uav_usv_gazebo/models/defense_enemy_*_boat` | 四艘敌方船模型 |
| `src/uav_usv_gazebo/models/four_level_uav_platform` | 四层无人机平台 |
| `src/uav_usv_gazebo/models/static_x500_fleet_uav_*` | 四架静态无人机模型 |
| `src/uav_usv_mission/scripts/defense_demo.py` | 防御算法和 Gazebo 船体控制 |
| `src/uav_usv_mission/scripts/fleet_base_station.py` | 基站数据聚合 |
| `src/uav_usv_mission/scripts/fleet_base_station_gui.py` | Qt 基站界面 |
| `src/uav_usv_mission/scripts/fleet_simulated_agent.py` | 模拟传感器和载具状态上报 |

## 8. 常见问题

### 画面启动后相机短暂 WAIT

Gazebo 相机、ROS bridge 和基站不是同一时刻启动的。刚启动时出现几秒
`WAIT` 正常，等 Gazebo 世界加载完成后会变成 `OK`。

### Qt 里雷达图看起来不是地图

这是正常的。该图显示的是 LaserScan 局部扫描，不是全局地图。全局防御态势看
“防御任务”页或 RViz。

### 拖动滑块后防御行为变化不明显

防御行为受多个参数共同影响。建议先调：

```text
trigger_radius
defend_radius
own_guard_speed
enemy_speed
```

再微调停止阈值：

```text
guard_stop_distance
enemy_guard_stop_distance
intercept_stop_distance
```

### 关闭 launch 时出现 context invalid

这是 ROS 2 / Qt / 多线程节点在 Ctrl-C 关闭过程中偶尔打印的退出栈。通常不是运行错误，
只发生在关闭过程，不影响演示启动和运行。
