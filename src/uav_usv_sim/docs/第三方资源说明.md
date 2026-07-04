# 第三方资源说明

## Sydney Regatta coastline

- 原始资源名称：`sydney_regatta`
- 作者与发布者：Open Robotics
- 来源：<https://app.gazebosim.org/OpenRobotics/fuel/models/sydney_regatta>
- 许可证：Creative Commons Attribution 4.0 International
- 许可证地址：<https://creativecommons.org/licenses/by/4.0/>

本项目没有把原始大文件提交到 Git 仓库。`prepare_coastline.sh` 在首次运行时
从 Gazebo Fuel 下载官方资源，并将缓存保存在 `/var/tmp/UAV_USV_gz_fuel`。

本项目对资源的集成修改：

- 保留原始视觉网格、PBR 材质和独立岸线碰撞网格。
- 将网格统一缩放为原尺寸的 `0.15`。
- 将水道旋转 `0.475 rad`，使主航道与原灯塔任务方向一致。
- 使用本项目海浪和海面替代原场景的水面。
