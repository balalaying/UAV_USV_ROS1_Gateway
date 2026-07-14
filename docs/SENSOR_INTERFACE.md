# UAV-USV 传感器接口

本文冻结舰队主线中 `usv_01` 的 Mid-360 仿真接口。当前数据可供后续
LV-DOT 或融合节点订阅，但尚未替代围捕任务使用的 Gazebo 真值目标。

## 数据链路

```text
RGL Gazebo Mid-360
  -> /fleet/uplink/usv_01/mid360/points
  -> mid360_preprocessor
  -> /perception/usv_01/points_filtered
  -> 后续 LV-DOT / 融合模块

mid360_preprocessor
  -> /fleet/sensor_status
  -> Qt 基站（只读取轻量状态，不解析完整点云）
```

## Topic

| Topic | 类型 | 默认频率 | 用途 |
| --- | --- | ---: | --- |
| `/fleet/uplink/usv_01/mid360/points` | `sensor_msgs/msg/PointCloud2` | 10 Hz | RGL 原始点云 |
| `/perception/usv_01/points_filtered` | `sensor_msgs/msg/PointCloud2` | 10 Hz | NaN/Inf、距离、自身裁剪和体素过滤后的标准出口 |
| `/perception/usv_01/mid360/preview` | `sensor_msgs/msg/PointCloud2` | 2 Hz | RViz 轻量预览，最多 5000 点 |
| `/fleet/sensor_status` | `uav_usv_interfaces/msg/SensorStatus` | 1 Hz | 频率、点数、延迟、处理耗时、超时和 TF 健康 |
| `/perception/usv_01/mid360/set_visualization` | `std_srvs/srv/SetBool` | 按需 | 仅开关预览，不停止原始和过滤点云 |

原始与过滤点云的 `header.frame_id` 固定为：

```text
usv_01/mid360_link
```

时间戳由 ROS 2 点云桥在收到 Gazebo 数据时写入，使用节点时钟；同一帧经过预处理后
保留原时间戳和 frame。预处理节点不会修改点坐标来掩盖 TF 错误。

## TF

```text
map
  -> usv_01/odom
    -> usv_01/base_link
      -> usv_01/mid360_link
```

- `map -> usv_01/odom -> usv_01/base_link` 由现有 USV/Nav2 接口发布。
- 船体动态 TF 从 `/usv_01/tf` 只读转发到全局 `/tf`。
- `usv_01/base_link -> usv_01/mid360_link` 是静态安装变换：
  `x=0.9075 m, y=0 m, z=1.5625 m`。
- `mid360_preprocessor` 使用 TF2 检查 `map -> usv_01/mid360_link`，结果写入
  `SensorStatus.tf_available`；TF 缺失时传感器健康状态为异常。

RViz 使用 `Fixed Frame=map`。验证命令：

```bash
ros2 run tf2_ros tf2_echo map usv_01/mid360_link
ros2 topic echo /fleet/sensor_status --once
```

## PointCloud2 字段与范围

RGL 当前输出至少包含标量 `FLOAT32 x/y/z` 字段，预处理节点保留输入记录中的全部
字段、字节序、`point_step` 和时间戳。默认参数：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `mid360_update_rate` | `10.0` | 扫描及期望健康频率，Hz |
| `mid360_min_range` | `0.5` | 最小距离，m |
| `mid360_range` | `70.0` | 最大距离，m |
| `mid360_voxel_size` | `0.12` | 体素边长，m；设为 0 关闭降采样 |
| `mid360_visual_scale` | `1.0` | 只缩放雷达外观，不改变射线、碰撞或动力学 |

自身裁剪盒在传感器坐标系中默认覆盖船体区域：
`x=[-4.3, 2.5] m`、`y=[-1.8, 1.8] m`、`z=[-2.4, 0.35] m`。

RGL 的 `Livox Mid360` preset 使用 `LivoxMid360.mat3x4f` 非重复扫描图案，因此主线
不提供会把它替换成普通栅格扫描的水平/垂直采样数参数。

## 启动与模式

```bash
# 默认开启 Mid-360
ros2 launch uav_usv_bringup fleet_dynamic_capture.launch.py

# 明确开启并放大传感器外观
ros2 launch uav_usv_bringup fleet_dynamic_capture.launch.py \
  enable_mid360:=true mid360_visual_scale:=1.5

# 关闭传感器，恢复原舰队负载
ros2 launch uav_usv_bringup fleet_dynamic_capture.launch.py \
  enable_mid360:=false
```

`perception_source` 接受 `ground_truth`、`mid360`、`hybrid`。本阶段该参数只冻结未来
接口；无论选择值为何，`capture_manager` 的目标输入仍保持现有真值链路，避免未经
目标检测的点云影响控制主线。

## 外观与动力学边界

Mid-360 外观是仓库内用 SDF box/cylinder 组合的简化 CAD 风格模型，不使用官方
Livox mesh。`mid360_visual_link` 是附着在原 `hull` 上的语义 frame；所有雷达几何均为
visual。运行时生成器不会添加 collision、inertial、joint，也不会修改 USV 的质量、
碰撞、波浪插件、Nav2 或速度控制插件。
