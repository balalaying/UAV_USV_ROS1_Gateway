# 332 仿真尺度更新报告

## 目标

将 332 主世界中的三架 PX4 无人机和三艘我方无人船放大为当前显示尺寸的两倍，同时保持原有 PX4、Nav2、感知、融合和任务链路不变。

本次不是视觉外壳放大。运行时生成的模型会同时调整 visual、collision、传感器安装位姿、质量和惯量。

## 默认尺度

主入口 `fleet_dynamic_capture_live_perception.launch.py` 现在默认传入：

| 参数 | 原默认值 | 新默认值 | 含义 |
| --- | ---: | ---: | --- |
| `uav_model_scale` | 6.0 | 12.0 | 相对 PX4 原始 x500 的实体物理尺度 |
| `usv_model_scale` | 1.0 | 2.0 | 相对 332 原始 USV 的实体物理尺度 |

因此，在当前主世界中，UAV 和 USV 都比修改前大两倍。`friendly_ship` 与 `enemy_ship` 保持原始外观和行为；仅将敌船初始位置从 `(-80, -315)` 调整为 `(-80, -345)`，避免它与放大后的 `usv_02` 初始重叠。

## 实现方式

### PX4 UAV

文件：`src/uav_usv_sim/tools/prepare_large_x500.py`

该脚本始终从 PX4 模型目录中的 `.uav_usv_unscaled` 备份重新生成：

- `x500_base`：机体、旋翼、碰撞、IMU 等实体几何和位姿按 `s` 缩放；
- `x500_mono_cam_down` 与 `mono_cam`：相机模块安装位姿、几何和碰撞按 `s` 缩放，图像分辨率与频率保持 launch 参数；
- 所有质量按 `s^3` 缩放；
- 所有惯量矩阵项按 `s^5` 缩放；
- `x500` 电机插件的推力、阻力和力矩系数同步缩放，避免扩大质量后仍使用原始推力而无法起飞。

因此 PX4 继续控制原来的 `x500_mono_cam_down` 实体，没有引入视觉跟随模型，也没有改动 Offboard DDS 接口。

### USV

文件：`src/uav_usv_gazebo/tools/prepare_fleet_mid360.py`

332 世界启动时在 `/var/tmp/UAV_USV_fleet_mid360` 生成运行时模型副本：

- 三个包装模型的识别色带、编号外观按 `s` 缩放；
- 合并的船体模型 visual、collision、mesh scale、相对 pose 按 `s` 缩放；
- 质量按 `s^3`，惯量按 `s^5` 缩放；
- Mid-360、RGB 相机和深度相机的安装坐标同步缩放；
- 船的既有速度控制插件、Nav2 接口、控制话题和命名空间保持不变。

即使使用 `enable_mid360:=false`，该运行时生成器仍会生成放大后的 USV；此时只不附加 RGL 插件和 Mid-360 传感器，避免同一 launch 参数在不同传感器模式下表现不一致。

## 关键传感器 TF

当 `usv_model_scale:=2.0` 时，三艘 USV 的静态安装位姿为：

| Frame | 相对 `usv_xx/base_link` 的平移（m） |
| --- | --- |
| `camera_link` | `(6.48, 0, 3.10)` |
| `depth_camera_link` | `(6.48, 0, 3.10)` |
| `front_lidar` | `(1.815, 0, 3.125)` |
| `mid360_link` | `(1.815, 0, 3.125)` |

这些值与生成后的 SDF 传感器挂载位置一致，避免点云在 `map` 下出现偏移。

## 验证

已执行：

```bash
python3 -m py_compile \
  src/uav_usv_sim/tools/prepare_large_x500.py \
  src/uav_usv_gazebo/tools/prepare_fleet_mid360.py \
  src/uav_usv_bringup/launch/fleet_dynamic_capture.launch.py \
  src/uav_usv_bringup/launch/fleet_dynamic_capture_live_perception.launch.py

colcon build --packages-select \
  uav_usv_sim uav_usv_gazebo uav_usv_bringup --symlink-install
```

结果：通过。

运行时模型副本验证结果：

| 项目 | 原值 | `usv_model_scale=2` 后 |
| --- | ---: | ---: |
| USV hull mass | 150 kg | 1200 kg |
| USV hull `ixx` | 85 | 2720 |
| USV hull collision | `5.61 x 3.135 x 0.625 m` | `11.22 x 6.27 x 1.25 m` |
| UAV base mass（`uav_model_scale=12`） | 2 kg | 3456 kg |

还执行了不启动 PX4 的 Gazebo/感知烟雾测试，三路话题均存在：

```text
/fleet/uplink/usv_01/mid360/points
/fleet/uplink/usv_02/mid360/points
/fleet/uplink/usv_03/mid360/points
```

并通过 `tf2_echo` 验证：

```text
usv_01/base_link -> usv_01/mid360_link
translation: [1.815, 0.000, 3.125]
```

## 可回退参数

无需修改源模型即可临时恢复旧尺寸：

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture_live_perception.launch.py \
  uav_model_scale:=6.0 usv_model_scale:=1.0
```

当前默认值为两倍展示尺度；后续速度控制页面不会修改这些尺度参数。
