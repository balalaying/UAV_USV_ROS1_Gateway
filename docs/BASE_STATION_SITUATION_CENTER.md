# 岸基统一态势中心

## 目标

`Fleet Situation View` 是 Qt 基站中的岸基态势原型。它把已有的
`/fleet/world_model` 转换为统一的二维雷达式地图，用于查看舰队状态，
而不是替代传感器调试页或任务控制页。

本阶段没有修改 LV-DOT、Camera-LiDAR Fusion、Perception Fusion、
Fleet World Model、PX4、Nav2、`capture_manager` 或 `FleetCommand`。

## 架构

```text
USV Mid360 / Camera / RGB-D       UAV Camera
           |                         |
           +---- existing perception/fusion ----+
                                                    |
                                          /fleet/world_model
                                                    |
                                      Qt Base Station GUI
                                                    |
                         Fleet Situation View (display only)
```

Qt 新页面没有订阅原始相机、点云、LV-DOT 调试或 TF 话题。它只消费
世界模型已经整理好的舰队状态、实体、目标、预测和威胁字段。因此，
显示层不会增加感知链路负载，也不会影响控制闭环。

## Qt 显示结构

`舰队态势` 页由三个区域组成：

1. 左侧 `岸基雷达图层`：控制网格、距离环、覆盖区域、历史轨迹、预测、
   威胁和标签的可见性。雷达覆盖半径仅改变画面比例，不修改任务或传感器参数。
2. 中央 `Fleet Situation View`：以 `map` 为固定坐标系的俯视雷达图。
   UAV 显示为蓝色三角形；USV-01/02/03 分别显示为蓝/绿/青；友方船为黄，
   敌方船与敌对目标为红。目标历史、预测轨迹和威胁等级来自 World Model。
3. 右侧 `目标来源与威胁信息`：列出每个融合目标的 ID、来源、传感器详情、
   置信度、更新时间和威胁等级，用于验证多船融合的可追溯性。

## Base Station 设计

Base Station 是显示和未来岸基通信的统一参考概念。当前 World Model
接口尚未发布一个名为 `base_station` 的正式实体，且本阶段禁止改动该接口。
因此界面以 `map` 原点显示一个逻辑岸基参考点，并清楚标为 `map origin`。

未来 World Model 增加如下实体后，Qt 会自动使用它而无需改动传感器或控制层：

```json
{
  "id": "base_station",
  "pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}}
}
```

这保持当前 TF 和 World Model 契约不变：所有载具、目标和预测仍在 `map`
坐标中表达；Base Station View 不创建或修改 TF。

## World Model 到 Qt 数据流

| World Model 字段 | Qt 用途 |
| --- | --- |
| `map_frame` | 固定坐标系显示 |
| `fleet.uav/usv/unknown` | 舰队位置、航向与在线数量 |
| `entities` | `FRIENDLY_SHIP`、`ENEMY_SHIP` 等非自治实体 |
| `targets` | 融合目标、来源、置信度、速度 |
| `predictions` | 目标预测轨迹 |
| `threats` | 威胁等级和雷达告警圈 |
| `perception.primary_source` | 当前态势数据源说明 |

## 启动与使用

使用现有完整系统启动入口即可，Qt 会自动出现 `舰队态势` 标签页：

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture_live_perception.launch.py
```

该页面不增加新的 launch 参数或 ROS topic。若 `/fleet/world_model` 尚未有数据，
页面保留空雷达背景并显示等待状态；数据恢复后自动更新。

## 面向 WebGL 的复用

WebGL 岸基平台应复用同一个 `/fleet/world_model` 数据契约或其网关转发版本：

```text
/fleet/world_model -> gateway -> WebSocket -> WebGL radar / 3D situation view
```

Web 前端同样只能使用 `map` 坐标的世界级实体、融合目标和预测结果，不能直接把
相机、LiDAR 或机体系坐标当作世界位置。这使 Qt 原型和未来网页端保持一致的
数据边界与显示语义。

## 兼容性边界

- 不新增或改名 ROS topic。
- 不改变 TF。
- 不改变 Fleet World Model JSON 结构。
- 不向任何任务、PX4、Nav2 或车辆 agent 发布命令。
- 历史轨迹仅保存在 Qt 进程内，用于显示；不会写回 ROS。
