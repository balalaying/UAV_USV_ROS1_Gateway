# Base Station Reference 设计

## 目的

岸基雷达态势必须拥有可配置、可复现的物理参考点。此前 Qt 在没有正式参考数据时
退化到 `map` 原点，导致距离环与舰队态势中心不对应真实指挥设施。

本设计把 332 场景岛屿上的 `shore_command_base` 作为岸基参考。该模型是位于三块
平行无人机起飞平台上方的指挥房屋，适合作为演示中的岸基站位置。

## 配置

配置文件：[base_station.yaml](/home/dji/UAV_USV/src/uav_usv_base_station/config/base_station.yaml)

```yaml
base_station_service:
  ros__parameters:
    base_station_id: shore_command_base
    map_frame: map
    base_station_x: -35.0
    base_station_y: -190.0
    base_station_z: 17.5
    base_station_yaw: 0.559
    radar_display_range_m: 300.0
```

该位置与 [heterogeneous_332.sdf](/home/dji/UAV_USV/src/uav_usv_gazebo/worlds/heterogeneous_332.sdf:51)
中 `shore_command_base` 的 Gazebo pose 一致。它是服务内部参考配置，不发布新的 TF，
也不修改 Fleet World Model。

## 数据流

```text
base_station.yaml
       |
       v
Base Station Service
       |                         /fleet/world_model
       +----------+----------------------+
                  |                      |
                  v                      v
         /base_station/state       immutable source data
                  |
                  v
          Qt Fleet Situation View
```

`/base_station/state.base_station` 至少包含：

- `id`
- `frame_id`
- `position`
- `orientation`
- `radar_display_range_m`
- `communication_status`
- 在线/连接载具列表
- 当前任务状态

所有舰队、目标、轨迹和预测仍保持 `map` 坐标。Qt 绘制时执行的是显示转换：

```text
object position in map - base station position in map -> radar canvas
```

这只是二维显示坐标换算，不更改目标消息、点云、相机、TF或感知算法的任何坐标。

## Qt 显示

`舰队态势` 页在完整启动模式下仅订阅 `/base_station/state`。

- 雷达中心：`base_station.position`
- 固定参考系：`base_station.frame_id`，默认 `map`
- 初始范围：`radar_display_range_m`
- 左侧调试栏显示基站 ID、位置、Frame 和当前比例尺
- 距离环、方向、舰队、融合目标、预测和威胁均相对同一岸基参考绘制

为了不破坏旧的独立调试启动，`dynamic_capture_console.launch.py` 保留
`use_base_station_service:=false` 回退选项；完整实时感知启动入口显式开启该参数。

## 验证

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture_live_perception.launch.py
ros2 topic echo /base_station/state --once
```

预期在快照中看到：

```json
"base_station": {
  "id": "shore_command_base",
  "frame_id": "map",
  "position": {"x": -35.0, "y": -190.0, "z": 17.5},
  "radar_display_range_m": 300.0
}
```

## 兼容性边界

本阶段未修改 LV-DOT、Camera-LiDAR Fusion、Perception Fusion、Fleet World
Model、TF、PX4、Nav2、`capture_manager` 或 `FleetCommand`。Base Station仅作为
服务输出与显示参考，不是控制器。
