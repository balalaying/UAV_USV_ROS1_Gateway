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

## LV-DOT Phase 5兼容映射

`/perception/lv_dot_ros2/dynamic_tracks`只包含已经通过连续动态投票的轨迹。
标准适配器发布`/perception/lv_dot/observations`时采用以下非破坏性映射：

| LV-DOT内部含义 | V1正式字段 | 旁路状态 |
| --- | --- | --- |
| dynamic probability | `confidence`包含跟踪置信度与动态概率的组合值 | `observation_status.compatibility`说明映射语义 |
| motion state | 不写入V1 | 固定为`CONFIRMED_DYNAMIC` |
| sensor source | `source_mask`，至少包含`SOURCE_LIDAR` | 同步报告字符串 |
| covariance | `pose.covariance`, `twist.covariance` | 原样保留 |
| semantic class | `classification` | 不根据运动状态伪造类别 |

若后续需要同时发布`STATIC`、`MOVING_CANDIDATE`和
`CONFIRMED_DYNAMIC`，应新增版本化消息或独立分类状态数组；不能重新定义V1
`confidence`或`classification`的含义。

## 多传感器Observation统一约束

LV-DOT和UAV视觉Observation使用完全相同的`TrackedObjectArray`，正式消息中禁止
加入LiDAR点数、图像像素框等传感器私有字段。

| 语义 | LV-DOT Observation | UAV Camera Observation |
| --- | --- | --- |
| UUID | 由来源内稳定track ID确定生成 | 由来源内稳定track ID确定生成 |
| track_id | LV-DOT tracker ID | `<vehicle>_camera_<target>` |
| position/velocity | `map`坐标 | `map`坐标 |
| covariance | tracker估计 | 视觉代理噪声模型 |
| source_mask | `SOURCE_LIDAR` | `SOURCE_CAMERA` |
| confidence | 动态track置信度 | 视觉Observation置信度 |
| classification | 不推断未知类别 | 保留代理真值类别 |
| timestamp | 动态track测量时间 | 触发图像时间 |
| frame | 数组header指定 | 数组header指定 |

当前UAV视觉第一阶段使用真值构造Observation，但只有在图像、CameraInfo、对应图像
时间戳的TF和视锥判定均有效时才发布非空观测。该来源在正式消息中标记为CAMERA，
并在独立状态topic明确标记`mode=ground_truth_proxy`；不得把该模式描述成真实视觉
检测精度。
