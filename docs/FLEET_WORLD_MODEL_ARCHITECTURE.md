# Fleet World Model 架构说明

## 目标

`Fleet World Model` 是舰队系统的唯一权威态势出口。后续 Qt、WebGL、Mission Manager、Behavior Manager 都应优先订阅它，而不是直接读取 Camera、PointCloud、LV-DOT 或零散调试话题。

当前第一版节点：

```bash
ros2 run uav_usv_mission fleet_world_model
```

主场景已默认启动该节点：

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture_live_perception.launch.py
```

## 输入

节点只读取标准化后的系统话题：

| 输入 | 类型 | 含义 |
| --- | --- | --- |
| `/fleet/state` | `uav_usv_interfaces/VehicleState` | UAV/USV在线、位姿、速度、模式、电量、任务状态 |
| `/fleet/perception/targets` | `TrackedObjectArray` | 当前控制主线使用的目标源，默认仍为 ground truth |
| `/fleet/perception/usv_tracks` | `TrackedObjectArray` | 三艘USV感知输出后的舰队级USV目标轨迹 |
| `/fleet/perception/fused_targets` | `TrackedObjectArray` | 舰队级融合目标 |
| `/fleet/sensor_status` | `SensorStatus` | 相机、Mid360、预处理等传感器健康状态 |
| `/fleet/command_ack` | `CommandAck` | 控制命令反馈 |
| `/capture/state` | `CaptureState` | 围捕任务状态机 |
| `/capture/roles` | `CaptureAssignmentArray` | 载具角色与任务点分配 |
| `/fleet/behavior/shadow_state` | `std_msgs/String` JSON | Shadow Behavior Manager的行为判断和角色建议 |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | 当前已知TF边 |

节点不直接订阅：

- 原始相机图像；
- 原始点云；
- LV-DOT debug topic；
- Gazebo私有调试数据；
- Qt内部显示数据。

## 输出

| 输出 | 类型 | 频率 | 用途 |
| --- | --- | --- | --- |
| `/fleet/world_model` | `std_msgs/String` JSON | 默认5Hz | 完整舰队世界模型 |
| `/fleet/world_model_summary` | `std_msgs/String` JSON | 默认5Hz | 轻量状态摘要 |

两类输出使用独立schema：

```text
/fleet/world_model         -> fleet_world_model.v1
/fleet/world_model_summary -> fleet_world_model.summary.v1
```

摘要中的 `world_model_schema_version` 用于标明它对应的完整世界模型版本。

## 全局TF约定

舰队级态势统一使用 `map` 作为最高坐标系。当前332主世界中，Gazebo真值节点发布：

```text
map
├── usv_01/base_link
├── usv_02/base_link
├── usv_03/base_link
├── uav_01/base_link
├── uav_02/base_link
├── uav_03/base_link
├── friendly_ship/base_link
└── enemy_ship/base_link
```

每艘USV的传感器静态安装关系为：

```text
usv_xx/base_link
├── usv_xx/camera_link
├── usv_xx/depth_camera_link
├── usv_xx/front_lidar
└── usv_xx/mid360_link
```

局部Nav2链路仍保留在 `/usv_xx/tf` 内部使用，但不再转发到全局 `/tf`。这样Qt、FWM和后续WebGL只看到统一的 `map` 世界坐标，不会把局部 odom 与舰队 map 混在一起。

无人机相机采用相同原则：

```text
map
└── uav_xx/base_link
    └── uav_xx/camera_link
```

传感器安装关系通过全局 `/tf_static` 发布。FWM将这些边标记为
`static=true`，不会因为静态变换只在启动时发布一次而误判为过期。

## JSON结构

`/fleet/world_model` 当前 schema 为：

```json
{
  "schema_version": "fleet_world_model.v1",
  "world_time": {},
  "map_frame": "map",
  "fleet": {
    "uav": [],
    "usv": [],
    "unknown": []
  },
  "entities": [],
  "targets": [],
  "predictions": [],
  "threats": [],
  "obstacles": [],
  "perception": {
    "primary_source": "fused_targets",
    "fused_targets": {},
    "usv_tracks": {},
    "ground_truth": {}
  },
  "sensors": {},
  "mission": {
    "capture": {},
    "capture_roles": {},
    "behavior": {}
  },
  "communication": {
    "recent_command_acks": []
  },
  "tf": {
    "edge_count": 0,
    "edges": []
  },
  "environment": {},
  "health": {}
}
```

新增态势字段说明：

| 字段 | 含义 | 当前实现 |
| --- | --- | --- |
| `predictions` | 目标未来位置预测 | 基于目标当前 `map` 坐标和速度，生成2s、5s、10s常速预测点 |
| `threats` | 威胁评分 | 根据身份、类别、速度、置信度、与保护船/基站距离生成 `LOW/MEDIUM/HIGH/CRITICAL` |
| `obstacles` | 障碍物列表 | 从融合目标中提取 `BUOY/DEBRIS/LANDMARK/UNKNOWN` 等非船只目标 |

这三个字段只服务态势显示和后续行为管理器，不会直接改变 PX4、Nav2 或 `capture_manager` 控制输入。

## 设计原则

1. `Fleet World Model` 只聚合标准化结果，不做图像、点云或算法调试计算。
2. 感知算法仍保持 Shadow Mode，不自动改变 `capture_manager` 的控制输入。
3. `targets` 优先使用 `/fleet/perception/fused_targets`；如果没有融合目标，则退回 `/fleet/perception/targets`。
4. 每个对象包含 `map` 坐标、时间戳、来源、置信度、类别、身份等字段，便于未来 WebGL 和行为管理器使用。
5. `fleet.uav`、`fleet.usv` 优先使用 `/fleet/state`；如果某个平台还没有 VehicleState，但已经存在 `map -> */base_link`，则生成 `state_source=tf_only` 的只读占位实体，保证岸基端仍能看到仿真/实机位姿。
6. `entities` 用于保护船、敌船、岸基设施等非受控任务实体；当前332主世界包含 `friendly_ship` 和 `enemy_ship`。
7. 旧 Qt 页面可以继续作为调试工具存在，但最终应迁移为只读 `/fleet/world_model`。
8. Shadow Behavior Manager只读取完整World Model；其结果回写到
   `mission.behavior`，但不会发布控制命令。

## Qt读取模式

`dynamic_capture_console.launch.py` 增加了：

```bash
fleet_world_model_only:=true
```

默认开启后，Qt的载具、传感器、目标、任务状态等业务数据优先只从 `/fleet/world_model` 读取。当前 Qt 仍保留图像、点云、LV-DOT debug topic 等直接订阅，用于感知调试画面；这些订阅只负责显示，不作为任务态势权威来源。

## 当前在线验证

轻量验证命令：

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture_live_perception.launch.py \
  start_px4:=false start_dds_agent:=false start_rviz:=false enable_console:=true
```

验证结果示例：

```text
/fleet/world_model: 约5Hz
/fleet/world_model_summary: 约5Hz
fleet.usv: usv_01, usv_02, usv_03
fleet.uav: uav_01, uav_02, uav_03（轻量模式下state_source=tf_only）
entities: friendly_ship, enemy_ship
targets: 来自 fused_targets
sensors: 包含USV相机、深度相机、Mid360、UAV相机状态
mission.capture.state: TRACKING
tf.edge_count: 已记录map、USV、UAV、传感器相关TF边
```

轻量模式未启动PX4，所以 `fleet.uav` 中的 UAV 为 `state_source=tf_only`、`online=false`。完整PX4模式启动后，UAV agent 会继续通过 `/fleet/state` 自动覆盖这些占位实体，`state_source` 变为 `vehicle_state`。

## 后续迁移方向

1. Qt Perception Monitor 改为订阅 `/fleet/world_model`。
2. WebSocket Gateway 改为优先推送 `/fleet/world_model` 和 `/fleet/world_model_summary`。
3. WebGL只消费世界坐标下的 `fleet`、`entities`、`targets`、`tf`、`mission` 数据。
4. Behavior Manager当前只读取World Model并输出Shadow建议；后续通过
   ControlLease安全仲裁后，才能由独立执行器转换为`FleetCommand`。
5. 如需要强类型接口，再新增正式 `FleetWorldModel.msg`，但不要破坏当前 JSON 调试出口。

当前阶段不启动或继续开发网页端。远程展示接口保留为：

```text
/fleet/world_model
/fleet/world_model_summary
```

后续Gateway只应读取以上世界坐标摘要；原始图像、点云和完整TF不直接通过公网传输。
