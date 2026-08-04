# Behavior Manager Shadow Mode

## 目标

`fleet_behavior_manager` 是 Fleet World Model 驱动架构中的第一版行为层。
它只读取 `/fleet/world_model`，输出行为判断和载具角色建议，不发布
`FleetCommand`、`ControlLease` 或任何 PX4/Nav2 底层话题。

因此当前链路是：

```text
标准化感知与平台状态
        |
        v
/fleet/world_model
        |
        v
fleet_behavior_manager
        |
        v
/fleet/behavior/shadow_state
        |
        v
Fleet World Model mission.behavior
```

这条反馈链只用于把行为分析结果纳入统一世界模型。行为节点不会读取
Camera、PointCloud、单个传感器话题或 Gazebo 真值私有接口。

## 行为状态

| 状态 | 触发条件 | 当前输出 |
| --- | --- | --- |
| `WAITING` | 尚未收到 World Model | 等待，不生成控制 |
| `SEARCH` | 无目标且系统可用 | 搜索/巡逻角色建议 |
| `TRACK` | 存在目标但未达到防御阈值 | 空中观察和水面跟踪建议 |
| `ESCORT` | World Model 中存在显式护航请求 | 保护船周围护航建议 |
| `DEFENSE` | 高威胁目标接近保护船，或达到严重威胁阈值 | 观察、拦截、防御屏障建议 |
| `CAPTURE` | 现有围捕状态机进入 `APPROACHING/ENCIRCLING/HOLDING` 执行阶段 | 复用已有围捕角色和任务点 |
| `RETURN` | World Model 中存在显式返航请求 | 返航建议 |
| `DEGRADED` | World Model 格式、坐标系或时效异常 | 禁止形成控制建议 |

策略支持任意数量和任意命名的 UAV/USV。分配依据是 World Model 中的
平台类型、在线状态、位置、目标距离、威胁评分和现有任务角色，不固定
`uav_01`、`usv_01`。

## 输出接口

Topic：

```text
/fleet/behavior/shadow_state
```

消息类型：

```text
std_msgs/msg/String
```

JSON schema：

```text
fleet_behavior_state.v1
```

关键字段：

```json
{
  "behavior": "DEFENSE",
  "reason": "hostile_target_inside_defense_policy",
  "shadow_mode": true,
  "control_enabled": false,
  "publishes_fleet_command": false,
  "target_id": "enemy_ship",
  "fleet_readiness": {},
  "recommendations": []
}
```

每条 `recommendations` 只包含角色、建议动作、目标和 `map` 坐标提示。
`control_command` 固定为 `null`，用于明确表示该输出不能直接驱动车辆。

## 主场景启动

`fleet_dynamic_capture.launch.py` 默认开启 Shadow Behavior Manager：

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture.launch.py
```

如需对照原系统，可关闭：

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture.launch.py \
  enable_shadow_behavior_manager:=false
```

单独查看：

```bash
ros2 topic echo /fleet/behavior/shadow_state --once
ros2 topic echo /fleet/world_model_summary --once
```

完整 World Model 中的结果位于：

```text
mission.behavior
```

轻量摘要中的当前状态位于：

```text
behavior_state
```

## 参数

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `publish_rate_hz` | 2.0 | 行为状态发布频率 |
| `world_model_timeout_sec` | 2.5 | World Model 超时阈值 |
| `default_behavior` | `SEARCH` | 无目标时的建议行为 |
| `defense_trigger_distance_m` | 120.0 | 高威胁目标触发防御的保护距离 |
| `high_threat_score` | 0.55 | 高威胁评分阈值 |
| `critical_threat_score` | 0.80 | 严重威胁立即防御阈值 |
| `transition_hold_seconds` | 1.5 | 普通状态切换防抖时间 |

`DEFENSE`、`CAPTURE`、`DEGRADED` 和 `WAITING` 属于立即切换状态，避免安全
事件被普通防抖延迟。

`capture_manager` 的 `TRACKING` 只表示已经获得目标轨迹，即使任务尚未由
操作者启动也可能出现，因此不会被解释为围捕正在执行。

## 启用真实控制前的门槛

当前版本不得接管控制。未来只有同时满足以下条件，才能新增独立的命令
执行器：

1. 行为建议在录包和在线场景中完成 Shadow 对比；
2. 岸基任务请求进入 World Model，并具备身份认证、有效期和审计记录；
3. `ControlLease` 完成统一仲裁，保证与 `capture_manager` 不抢控制权；
4. 每条行为建议经过任务级安全约束、地理围栏和可达性检查；
5. 命令执行器继续使用已有 `FleetCommand -> agent -> PX4/Nav2` 链路；
6. 失联、超时、拒绝和人工接管路径完成实测。

## 设计边界

- 不修改 `capture_manager`；
- 不修改 `FleetCommand` 接口；
- 不修改 PX4、Nav2 或 UAV/USV agent；
- 不直接读取传感器和 Gazebo 私有数据；
- 不把 Shadow 建议伪装成已执行任务；
- 后续 Qt、Gateway 和 Web 只需读取更新后的 Fleet World Model。

332主场景在线验证记录见：

```text
docs/BEHAVIOR_MANAGER_332_SHADOW_VALIDATION.md
```
