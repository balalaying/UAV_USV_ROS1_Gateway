# 332异构协同仿真场景报告

## 1. 范围与安全边界

本场景现已设为舰队主线统一世界。`fleet_dynamic_capture.launch.py`和`fleet_dynamic_capture_live_perception.launch.py`均使用`heterogeneous_332.sdf`，独立`simulation_332_scenario.launch.py`仍保留为纯Gazebo环境检查入口。

控制节点、FleetCommand消息、PX4 Offboard实现、Nav2参数结构、capture_manager算法、LV-DOT算法和Camera-LiDAR算法均未改写；本次只同步场景实体映射、载具数量、初始坐标、显示名称和感知输入映射。

## 2. 场景组成

世界文件：`uav_usv_gazebo/worlds/heterogeneous_332.sdf`

该世界集合GitHub PR #11的Catalina岛屿环境，但保持海面为空旷任务区。场景只保留岛屿、岸线碰撞、岸基指挥站、三联无人机停机坪和完整332载具集合；PR中的海上障碍、任务点、航标、灯塔和重复演示载具均不在世界中加载。

| 展示ID | Gazebo实体 | 模型 | 初始位置(m) | 说明 |
|---|---|---|---|---|
| UAV_01 | `uav_01` | PX4 `x500_mono_cam_down` | `(-86.86,-222.43,19.75)` | 三联停机坪左侧 |
| UAV_02 | `uav_02` | PX4 `x500_mono_cam_down` | `(-75,-215,19.75)` | 三联停机坪中央 |
| UAV_03 | `uav_03` | PX4 `x500_mono_cam_down` | `(-63.14,-207.57,19.75)` | 三联停机坪右侧 |
| USV_01 | `usv_01` | `sim332_usv_blue` | `(-120,-305,0)` | 南侧开阔海域，蓝色识别带 |
| USV_02 | `usv_02` | `sim332_usv_green` | `(-75,-320,0)` | 南侧开阔海域，绿色识别带 |
| USV_03 | `usv_03` | `sim332_usv_cyan` | `(-30,-305,0)` | 南侧开阔海域，青色识别带 |
| FRIENDLY_SHIP | `friendly_ship` | `sim332_friendly_ship` | `(-150,-355,0)` | 南侧开阔海域，黄色甲板、红色侧舷标识 |
| ENEMY_SHIP | `enemy_ship` | `sim332_enemy_ship` | `(-80,-315,0)` | 位于USV_01初始感知范围内，黑白船体、红色敌方标识、随机运动 |

Gazebo实体和ROS namespace统一采用小写snake_case。大写名称仅作为人机界面展示ID。这样`PX4_GZ_MODEL_NAME=uav_01`、`PX4_UXRCE_DDS_NS=uav_01`和现有agent参数均无需修改。

## 3. Catalina环境集合

从PR #11选择性使用：

- `catalina_island`：约998 x 851米的卫星地形视觉mesh；
- `shore_collision_boundary`：约1050 x 900米的外围碰撞边界；
- `mountain_shore_command_base`：保留的岸基通信指挥站。

Catalina mesh保持visual-only，避免ODE/DART对glTF mesh collision产生断言。视觉模型整体下沉0.8米以消除岸边与水面的空隙；`sim332_catalina_collision_proxy`使用五段低高度复合碰撞体阻止USV穿过主要陆地区域，同时不形成妨碍UAV飞越岛屿的高墙。原始Catalina资产为CC-BY-4.0，许可证保留在`catalina_island/meshes/license.txt`。

海面配置与`fleet_dynamic_capture_live_perception`保持一致：半透明`ocean_plane`提供视觉和支撑碰撞，`model://waves`提供Gerstner动态海浪。Gazebo GUI默认使用同一个`gazebo_white_gui.config`，保持白色工具栏、插件面板和交互方式一致。

海面不加载`sea_task_obstacles`、`sea_mission_points`、`usv_avoidance_obstacles`、`pursuit_task_elements`、`sea_beacons`、`open_sea_task_extensions`、灯塔或浮标。没有合入PR对现有`waves`、`medium_buoy`和README的覆盖修改，也没有合入PR中的重复演示船/UAV。

## 4. 新增332模型与设计

新增模型：

- `sim332_usv_blue`
- `sim332_usv_green`
- `sim332_usv_cyan`
- `sim332_friendly_ship`
- `sim332_enemy_ship`
- `sim332_island_uav_base`
- `sim332_catalina_collision_proxy`

前三个USV通过SDF merge复用现有`defense_own_01/02/03_boat`。因此原质量、惯量、碰撞体、船载Camera、二维LiDAR和海浪随动插件保持不变；新增的`identity_shell`只有visual和固定关节，不参与碰撞。

`defense_own_01/02_boat`的Camera像素格式由Gazebo Harmonic不支持的`B8G8R8`修正为`R8G8B8`。topic、尺寸、帧率、相机内外参和ROS消息接口没有变化。

敌船复用`scalable_capture_usv`动力学。保护船改为独立的红黄大型指挥船，约13.5 x 5.1米，是普通9 x 3.4米任务船的1.5倍；碰撞体、质量和惯量同步适配，继续使用相同的`BoatWaveFollower`和`cmd_vel`接口。外观包含红色流线船壳、黄色甲板与舷侧识别带、深色驾驶舱窗、红色顶棚和通信桅杆。

`sim332_island_uav_base`由三块平行H停机坪、统一承重甲板和六根支柱组成。甲板顶面约为19.5米，指挥站连接栈桥最高处位于甲板下方，避免遮挡无人机起飞。三架UAV分别生成在对应停机坪中心；岸基指挥站保留在平台东北侧，两者碰撞体互不重叠。

## 5. namespace规划

| 载具 | ROS namespace | PX4 namespace/instance | 说明 |
|---|---|---|---|
| UAV_01 | `/uav_01` | `/uav_01`, instance 0 | 后续复用现有PX4启动方式 |
| UAV_02 | `/uav_02` | `/uav_02`, instance 1 | 后续复用现有PX4启动方式 |
| UAV_03 | `/uav_03` | `/uav_03`, instance 2 | 后续复用现有PX4启动方式 |
| USV_01 | `/usv_01` | 不适用 | 现有控制源名`own_01` |
| USV_02 | `/usv_02` | 不适用 | 现有控制源名`own_02` |
| USV_03 | `/usv_03` | 不适用 | 现有控制源名`own_03` |
| FRIENDLY_SHIP | `/friendly_ship`（预留） | 不适用 | 当前无行为约束和agent |
| ENEMY_SHIP | `/enemy_ship`（预留） | 不适用 | 当前只由Gazebo随机运动插件驱动 |

主线launch为三架PX4分别启动instance 0、1、2，并为三艘USV启动原有boat interface、Nav2和USV agent。UAV起飞高度仍为相对初始点18米；由于岸上停机坪位于`z=19.75 m`，观察点绝对高度同步为`42 m`，避免飞行器贴近平台或指挥站。

## 6. TF规划

现有TF约定保持不变：

```text
map
|- uav_01/base_link
|  `- uav_01/camera_link
|- uav_02/base_link
|  `- uav_02/camera_link
|- uav_03/base_link
|  `- uav_03/camera_link
|- usv_01/odom
|  `- usv_01/base_link
|     |- usv_01/camera_link
|     |- usv_01/front_lidar
|     `- usv_01/mid360_link
|- usv_02/odom
|  `- usv_02/base_link
|     |- usv_02/camera_link
|     `- usv_02/front_lidar
`- usv_03/odom
   `- usv_03/base_link
      |- usv_03/camera_link
      `- usv_03/front_lidar
```

独立场景launch只启动Gazebo，不伪造ROS TF。UAV TF仍由PX4 agent提供，USV TF仍由现有boat interface提供，Mid-360仍按当前`usv_01`挂载流程生成。

## 7. Topic规划

场景直接产生的Gazebo Transport topic：

| Topic | 内容 |
|---|---|
| `/world/heterogeneous_332/pose/info` | 所有实体位姿 |
| `/world/heterogeneous_332/clock` | 仿真时钟 |
| `/world/heterogeneous_332/model/uav_XX/link/camera_link/sensor/camera/image` | UAV相机 |
| `/defense/own_XX/front_camera` | USV船载相机 |
| `/defense/own_XX/scan` | USV二维LiDAR |
| `/model/own_XX/cmd_vel` | 现有USV模型控制入口 |
| `/model/enemy_ship/cmd_vel` | 随机运动插件到敌船运动模型的内部接口 |

接入现有`gz_sensor_bridge`后沿用的ROS 2接口：

- `/fleet/uplink/uav_XX/camera`
- `/fleet/uplink/uav_XX/camera_info_raw`
- `/fleet/uplink/usv_XX/camera`
- `/fleet/uplink/usv_XX/camera_info_raw`
- `/usv_XX/scan_raw`
- `/fleet/uplink/usv_01/mid360/points`（由既有Mid-360准备流程产生）
- `/perception/usv_01/mid360/points_filtered`

本次没有更改任何感知topic或消息类型。Qt会显示3 UAV和3 USV的相机与载具状态，但真实LV-DOT及Camera-LiDAR融合严格只使用USV_01：

```text
/fleet/uplink/usv_01/mid360/points
  -> /perception/usv_01/mid360/points_filtered
  -> /perception/lv_dot/*

/fleet/uplink/usv_01/camera/image_raw
  + /perception/usv_01/mid360/points_filtered
  -> Camera-LiDAR association
  -> Qt Perception Monitor
```

USV_02和USV_03只接入状态与相机显示，不启动LV-DOT实例，也不参与Camera-LiDAR融合。

## 8. 敌船随机行为

`RandomVesselMotion`是Gazebo系统插件，不依赖ROS回调或任务节点。状态机为：

```text
TURN -> CRUISE -> (STOP或TURN) -> ...
```

- 航向变化：每个航段随机选择`[-1.75, 1.75] rad`转角；
- 速度：巡航速度在`0.35~1.15 m/s`之间随机；
- 停留：每段巡航后有`28%`概率停留`2~6 s`；
- 换向：转向角速度为`0.22 rad/s`；
- 航段：持续`5~14 s`；
- 边界：接近半径`400 m`活动区边缘时自动转向世界中心，与Catalina外围碰撞边界留出安全距离；
- 随机种子：默认`332`，便于复现实验。

插件只向敌船自己的Gazebo `cmd_vel`入口发布，不发布FleetCommand，也不进入capture_manager。

## 9. 启动方式

所有实体由世界文件中的`<include>`生成，启动顺序确定，不依赖运行时spawn服务。启动命令：

```bash
cd <你的UAV_USV工作区>
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_usv_bringup simulation_332_scenario.launch.py
```

完整主线（Gazebo、3 PX4 UAV、3 Nav2 USV、Mid-360、LV-DOT、融合和Qt）：

```bash
ros2 launch uav_usv_bringup \
  fleet_dynamic_capture_live_perception.launch.py
```

只验证场景、USV和感知链，不启动PX4：

```bash
ros2 launch uav_usv_bringup \
  fleet_dynamic_capture_live_perception.launch.py \
  start_px4:=false start_dds_agent:=false
```

无GUI测试（显式覆盖默认白色GUI参数）：

```bash
ros2 launch uav_usv_bringup simulation_332_scenario.launch.py gz_args:="-r -s"
```

如PX4目录不在默认位置：

```bash
ros2 launch uav_usv_bringup simulation_332_scenario.launch.py \
  px4_dir:=<你的PX4-Autopilot路径>
```

## 10. 兼容性与验证

- SDF 1.10完整展开成功；
- `uav_usv_gazebo`和`uav_usv_bringup`构建成功；
- Gazebo Harmonic无GUI运行成功；
- 3 UAV、3 USV、FRIENDLY_SHIP和ENEMY_SHIP全部生成；
- UAV继续使用PX4原生`x500_mono_cam_down`模型和既有命名；
- 三艘USV保留现有传感器和控制入口；
- 敌船5秒位姿采样确认同时发生平移和转向；
- `fleet_dynamic_capture`已使用1050 x 900米空白Nav2地图，三艘USV初始位姿均处于地图范围内；
- Mid-360运行模型生成器已支持`merge=true`的332 USV视觉包装模型；
- Mid-360原始和过滤点云约18 Hz，LV-DOT诊断约18 Hz；
- LV-DOT track与`enemy_ship`真值实测位置相差约2米；
- 3 USV与3 UAV相机接口均可见，实测约11至14 Hz；
- Qt统一显示3 UAV、3 USV，感知页只展示USV_01融合相机和USV_01点云；
- Qt俯视画布会按USV_01 TF或点云中位数自动定位到新世界坐标，实测渲染约22 FPS；
- source mux仍保持`ground_truth`，真实感知继续处于Shadow模式。

已知边界：本轮回归在关闭PX4/DDS的模式下完成完整Gazebo、三USV、Mid-360、LV-DOT、Camera-LiDAR和Qt运行验证；PX4启动参数、instance和现有实体绑定已做静态检查，但未在本轮重复执行三机解锁飞行。
