# Qt 岸基雷达态势页面验证报告

## 修改内容

- 新增 `uav_usv_mission/base_station_radar.py`：
  - `BaseStationStateParser`：校验并标准化 `base_station_service.v1`；
  - `SituationViewModel`：维护最新快照、图层、选择状态、相对距离与方位；
  - `RadarCanvas`：只负责传统 360° 雷达绘制与鼠标交互。
- 修改 `scripts/fleet_base_station_gui.py`：
  - 将“舰队态势”页命名为“岸基雷达态势”；
  - 使用 `RadarCanvas` 替代旧的简单静态地图；
  - 增加量程选择、自动量程、回到基站中心和完整图层控制；
  - 增加载具/目标点击详情，明确融合与 Ground Truth 回退来源。
- 新增显示契约：`docs/base_station/RADAR_SITUATION_DISPLAY_CONTRACT.md`。

## 数据与安全边界

业务态势页面继续只经由 `/base_station/state` 和 `/base_station/events` 获取数据。
没有新增 ROS 订阅、发布器或控制指令；LV-DOT、相机-激光融合、Fleet World Model、
Base Station Service、PX4、Nav2、FleetCommand 和 `capture_manager` 未改动。

## 验证项目

| 项目 | 结果 | 说明 |
|---|---|---|
| Python 语法 | 通过 | `py_compile` 覆盖新增模块、Qt 主脚本和控制台 launch。 |
| Base Station schema | 通过 | 合成 `base_station_service.v1` 快照能被解析；错误 schema 进入等待状态而不崩溃。 |
| 坐标计算 | 通过 | 基站 `(10,-5,4)`、目标 `(10,55,0)` 的结果为 `60.0 m / 90.0°`。 |
| 332 对象解析 | 通过 | 合成快照解析出三 UAV、三 USV、友方船、敌方船及一个真实融合目标。 |
| 离屏绘制 | 通过 | `QT_QPA_PLATFORM=offscreen` 成功绘制 `900 x 700` 雷达画布。 |
| Qt 启动烟雾测试 | 通过 | 控制台启动 8 秒，无 Traceback、KeyError 或 QPainter 错误。 |
| 图层与交互 | 代码检查通过 | 提供缩放、右/中键平移、双击复位、对象点击和图层切换。 |
| Shadow 边界 | 通过 | 扫描动画只由 Qt Timer 绘制，不影响感知与控制。 |

## 性能目标

- `/base_station/state`：约 5 Hz；
- 雷达绘制：20 至 30 FPS；
- 状态快照：只保留最新一份；
- 历史轨迹：使用服务端既有上限，避免无限增长。

## 已知限制

本页面是二维岸基态势显示，不显示三路原始点云；原始点云和算法诊断仍在 Perception
Monitor。扫描扇区为演示动画，不等价于真实 Mid-360 扫描，也不表示雷达覆盖或检测概率。

本次未在完整 332 Gazebo 长时运行 30 分钟；该稳定性验证应在演示前通过主入口完成，
并观察 Qt 进程内存、`/base_station/state` 频率和状态恢复行为。
