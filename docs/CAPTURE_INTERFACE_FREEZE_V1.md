# 围捕核心接口冻结 V1

冻结日期：2026-07-13

以下五个消息是载具、感知和任务层之间的稳定接口。本阶段只允许新增消息，
不修改这些文件的字段、常量、顺序或语义。

| 消息 | SHA-256 |
|---|---|
| `FleetCommand.msg` | `4be7a9f16d65feba95a245fce53d89e5f6df1ed27d0b26e431de08d303400a00` |
| `VehicleState.msg` | `86fc67ee21cef7b57af9ded10d55ea768c45ff3aedc42b904338289c2b1191dd` |
| `CommandAck.msg` | `0d4f4073b9fa812966b4c1df00a72c6717659d3e5a0ae6867a4ce083c5aab1e2` |
| `ControlLease.msg` | `61bc1dceee0b6e7b8c85151b0045c895a3f1f3ad1381fdb26d33497d36fcb8f0` |
| `TrackedObjectArray.msg` | `e3ae24a4bc2f200044be54f8f219504fa2e721f58b846c188b4ce38ed111251a` |

## 结构化围捕状态

原来的 `/capture/state`、`/capture/roles`、`/capture/target_status` 使用
`std_msgs/String` 携带 JSON，缺少编译期字段检查，也不利于 rosbag、Qt 和测试工具
直接读取。本阶段新增：

- `CaptureState`
- `CaptureAssignment`
- `CaptureAssignmentArray`
- `CaptureTargetStatus`

三个原 topic 改用上述强类型消息。过渡期保留：

- `/capture/state_text`
- `/capture/roles_json`
- `/capture/target_status_json`

兼容话题只用于旧界面迁移，不属于冻结接口。
