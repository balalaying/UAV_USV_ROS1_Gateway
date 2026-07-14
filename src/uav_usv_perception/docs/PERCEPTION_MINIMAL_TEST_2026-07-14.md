# 感知层最小闭环测试

日期：2026-07-14。

场景：`uav_01 + usv_01 + target_vessel`。

## 构建与单元测试

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  uav_usv_interfaces uav_usv_perception uav_usv_bringup \
  --symlink-install

colcon test --packages-select uav_usv_perception
colcon test-result --verbose --test-result-base build/uav_usv_perception
```

结果：3 个包构建成功；track association 的 4 个 pytest case 全部通过；colcon 汇总
为 5 tests、0 errors、0 failures、0 skipped。

## 合成双传感器融合

独立启动：

```bash
ros2 launch uav_usv_perception perception_layer.launch.py \
  start_ground_truth_adapter:=false perception_source:=sensor
```

向 UAV observation 发布 `source_mask=CAMERA, confidence=0.7`，向 USV observation
发布 `source_mask=LIDAR, confidence=0.8`。两者位置相距约 1.12 m，位于 8 m 关联门内。

输出：

```text
track_id: target_vessel
source_mask: 11 = LIDAR | CAMERA | FUSED
position: approximately [10.53, 20.27, 0.50]
confidence: approximately 0.94
mux source: sensor
mux online: true
track_count: 1
```

稳定 UUID 在 fused 和 `/fleet/perception/targets` 中一致。

## 真实最小世界

启动：

```bash
ros2 launch uav_usv_bringup minimal_dynamic_capture.launch.py \
  start_rviz:=false perception_source:=ground_truth
```

Topic 实测：

```text
/perception/ground_truth/tracks    TrackedObjectArray
/perception/uav_01/observations   TrackedObjectArray
/perception/usv_01/observations   TrackedObjectArray
/perception/fused/tracks           TrackedObjectArray
/perception/hybrid/tracks          TrackedObjectArray
/fleet/perception/targets          TrackedObjectArray
/perception/source_status          String
```

结果：

- ground truth adapter 持续找到 `target_vessel`；
- ground truth 和 canonical topic 均稳定约 10.0 Hz；
- hybrid fusion 保持相同稳定 `track_id` 与 UUID；
- mux 报告 `ground_truth` online，track_count=1；
- PX4 DDS 上线、UAV 解锁并起飞；
- Nav2 active，USV 接受并执行动态目标；
- 未修改的 capture manager 依次进入：
  `SEARCH -> TRACKING -> APPROACHING -> ENCIRCLING -> HOLDING -> SUCCESS`。

## 运行时来源切换

运行中向 `/perception/usv_01/observations` 发布一个 LiDAR 来源的
`target_vessel`，然后执行：

```bash
ros2 param set /perception_source_mux perception_source sensor
ros2 param set /perception_source_mux perception_source ground_truth
```

结果：

```text
ground_truth -> sensor: successful
sensor source online: true
sensor track_count: 1
canonical source_mask: LIDAR
capture state while sensor selected: SUCCESS
sensor -> ground_truth: successful
ground_truth source online: true
```

切换期间没有重启 capture manager、PX4、Nav2 或 Gazebo。

## 关闭

测试结束后使用 Ctrl+C。perception adapter、两个 fusion、mux、capture manager、agent、
Nav2 和 Gazebo 均退出；MicroXRCEAgent 的 `exit code -2` 是收到 SIGINT 的正常退出表现。

## 已知限制

1. sensor 模式使用合成 `TrackedObjectArray` 验证接口；尚未接入 LV-DOT 检测结果。
2. 最小场景的任务目标仍要求稳定 `track_id=target_vessel`。
3. 第一版关联是带时间门限的最近邻，不覆盖密集多目标交叉场景。
