# Qt 载具速度控制页

## 页面位置

Qt 基站新增 `速度控制` 页面，包含两组全舰队滑块：

- 无人船航行上限：同时作用于 `USV_01`、`USV_02`、`USV_03`；
- 无人机飞行上限：同时作用于 `UAV_01`、`UAV_02`、`UAV_03`。

滑块为 `0` 时显示“自动（原有控制）”。这是默认状态，完全保留修改前的行为。

## 控制链路

页面不向 Gazebo 发布 `cmd_vel`，也不绕过 FleetCommand。

```text
Qt speed page
  -> ROS 2 SetParameters service
    -> existing USV Nav2 interface / PX4 DDS agent
      -> existing Nav2 / PX4 Offboard command path
```

### USV

Qt 给以下节点写入 `speed_limit_mps`：

```text
/usv_01/boat_nav2_interface
/usv_02/boat_nav2_interface
/usv_03/boat_nav2_interface
```

`boat_nav2_interface` 仍接收 Nav2 的 `cmd_vel`。正数限制仅降低前向线速度上限；`0.0` 取消额外限制，恢复原有 `max_linear_output` 与 PID 控制。

332主场景当前Nav2自动航速上限为`10.0 m/s`，Gazebo输出保护上限为
`11.0 m/s`；Qt滑块范围同步扩展到`0--11.0 m/s`。

### UAV

Qt 给以下节点写入 `flight_speed_limit_mps`：

```text
/uav_01_dds_agent
/uav_02_dds_agent
/uav_03_dds_agent
```

PX4 agent 仍订阅 `/fleet/command`，仍通过 PX4 uXRCE-DDS 发布 Offboard position setpoint。正数使 agent 以该速度平滑推进目标 setpoint；`0.0` 继续直接转发目标点，因此保持之前的 PX4 飞行行为。

## 使用方法

1. 启动完整 332 主入口。
2. 打开 Qt 基站的 `速度控制` 标签页。
3. 设置无人船或无人机滑块。
4. 点击“应用到全部 USV/UAV”。
5. 需要恢复默认行为时点击“恢复自动速度”。

服务尚未启动时，界面会向事件日志写入提示；不会重试发布底层控制指令。
