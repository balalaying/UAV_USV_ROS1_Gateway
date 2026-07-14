# TrackedObject 感知契约

现有 `uav_usv_interfaces/msg/TrackedObject` 已满足本阶段需要，不修改消息定义。

| 能力 | 字段 | 结论 |
| --- | --- | --- |
| 稳定身份 | `uuid`, `track_id` | 已支持 |
| 来源 | `source_mask` | 位掩码支持多源融合 |
| 语义 | `classification` | 支持 vessel/buoy/debris/landmark |
| 时间 | `first_seen`, `last_update` | 已支持 |
| 状态 | `pose`, `twist` | 包含 6x6 covariance |
| 尺寸 | `dimensions` | 已支持 3D bbox |
| 质量 | `confidence` | 已支持 |
| AIS | `mmsi` | 保留但本阶段不使用 |

## 发布约束

1. `TrackedObjectArray.header.frame_id` 必须描述数组中所有对象的坐标系。
2. `last_update` 是产生观测的时间，不是 fusion 定时器再次发布的时间。
3. `track_id` 在同一目标生命周期内稳定；不得使用数组下标。
4. `uuid` 由稳定 `track_id` 确定性生成。
5. `source_mask` 按位组合，fusion 不覆盖已有来源位。
6. 位置、速度和 covariance 必须处于 header 指定坐标系。
7. 未知类别使用 `CLASS_UNKNOWN`，不得为了让任务启动而伪造语义类别。
8. confidence 限定在 `[0, 1]`。

## 非破坏性扩展策略

本阶段字段充足。未来若需要检测器内部信息，优先新增旁路消息，而不是修改冻结消息：

- 图像框、mask、类别分布：独立 `Detection2DArray`；
- 点云 cluster：独立 `PointCloud2` 或调试 topic；
- 传感器贡献权重：诊断消息；
- 航迹历史：`Path`/Marker，仅用于显示；
- 算法内部状态：后端私有消息。

只有多个正式消费者都需要、且无法从现有字段表达时，才建立版本化
`TrackedObjectV2`，并提供 V1/V2 bridge。禁止直接改变现有字段顺序或语义。
