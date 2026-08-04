# Qt 岸基雷达态势显示契约

## 用途与边界

“岸基雷达态势”是 Qt 基站中的只读业务页面。其唯一业务输入为：

- `/base_station/state`
- `/base_station/events`

页面不直接订阅 Fleet World Model、TF、相机、点云、LV-DOT 或相机-激光调试话题；
也不发布任何控制指令。底层传感器调试仍由 Perception Monitor 页面负责。

```text
/base_station/state -> BaseStationStateParser -> SituationViewModel
                      -> RadarCanvas -> Qt 目标详情面板
```

## 状态字段

输入 schema 固定为 `base_station_service.v1`。显示端使用以下字段：

| 字段 | 用途 |
|---|---|
| `map_frame` | 固定世界坐标系，当前为 `map`。 |
| `base_station.position/orientation` | 雷达中心、朝向和相对方位计算。 |
| `fleet.uav/usv/unknown` | 舰队载具的位置、状态、航向和速度。 |
| `entities` | `FRIENDLY_SHIP`、`ENEMY_SHIP` 等环境任务实体。 |
| `targets` | 融合目标或控制回退目标。 |
| `target_history` | 目标历史轨迹。 |
| `predictions` | 目标预测轨迹。 |
| `threats` | 威胁等级及告警圈。 |
| `perception` | 主感知源、融合数量与回退状态。 |

## 坐标与方位

所有上游位置保持 `map` 坐标，不创建第二套世界坐标。显示时以基站为参考：

```text
dx = entity.map.x - base_station.map.x
dy = entity.map.y - base_station.map.y
distance = sqrt(dx^2 + dy^2)
bearing = normalize(atan2(dy, dx) - base_station.yaw)
```

约定：`map +Y` 为北、`map +X` 为东；雷达顶端为北、右侧为东。目标详情同时显示绝对
`map` 坐标、相对距离和相对方位角。

## 颜色与图标

| 对象 | 显示规则 |
|---|---|
| Base Station | 中央亮黄色十字标记。 |
| `USV_01/02/03` | 蓝色、绿色、青色船形图标。 |
| UAV | 浅蓝三角形，标签包含高度。 |
| `FRIENDLY_SHIP` / FRIENDLY | 黄色主体。 |
| `ENEMY_SHIP` / HOSTILE | 红色主体。 |
| UNKNOWN | 黄色。 |
| NEUTRAL | 灰色。 |
| 真实融合目标 | 绿色来源标签 `REAL FUSION`。 |
| Ground Truth 回退 | 明确标签 `GROUND TRUTH`，不得伪装为真实融合。 |
| 离线/过期对象 | 降低透明度；高威胁对象增加红色虚线告警圈。 |

颜色表达阵营和载具身份。传感器来源通过标签、来源字段和目标详情表达，不能只依赖颜色。

## 图层与交互

图层包含网格、距离环、角度刻度、扫描扇区、舰队、任务实体、融合目标、历史轨迹、预测
轨迹、速度向量、威胁圈、标签、离线对象、过期对象和来源标签。图层开关只影响绘制，不会
影响 ROS 2 节点、感知算法或控制链。

- 鼠标滚轮：缩放量程；
- 鼠标右键或中键拖动：平移视图；
- 双击或“回到基站中心”：恢复基站居中；
- 点击对象：高亮对象并更新右侧详情；
- “自动量程”：根据当前可见对象更新量程；
- 扫描扇区：仅由 Qt 定时器绘制，不参与任何传感器处理。

## 性能约束

ROS 回调仅把最新 `/base_station/state` 快照发送给 Qt；画布不保存每一帧状态消息。
`RadarCanvas` 以约 25 FPS 的轻量 Qt Timer 动画绘制扫描线，状态快照通常以 5 Hz 更新。
JSON 仅在状态到达时解析，绘制时只使用已标准化的内部对象。

## WebGL 复用

后续 WebGL 可直接复用本契约中的：`map` 坐标、基站相对坐标公式、方位定义、对象分类、
颜色规则、图层名称、过期状态和融合/回退标记。浏览器只需消费
`base_station_snapshot`，无需解析 ROS TF 或底层传感器 Topic。
