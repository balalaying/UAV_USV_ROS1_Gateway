# UAV_USV ROS1 / Gateway 工程迁移指南

本文用于将当前 UAV_USV ROS1、Gazebo/GZ 仿真、Gateway、协同算法及第三方依赖环境迁移到新的 Ubuntu 电脑。

## 1. 基准环境

当前已验证开发环境：

```text
Ubuntu 20.04.6 LTS
ROS Noetic
Gazebo Classic 11.15.1
Gazebo Sim 8.15.0
CMake 3.27.9
Python 3.8.10
GCC/G++ 9.4.0
```

Gazebo Sim 8 并非系统 apt 二进制安装，而是通过源码工作区：

```text
~/gz_harmonic_ws
```

构建得到。

完整环境快照位于：

```text
docs/environment/
```

---

## 2. 推荐工作区结构

新电脑建议保持以下目录：

```text
~/uav_usv_ros1_ws/
├── build/
├── devel/
└── src/
    └── UAV_USV/
```

工程仓库必须位于：

```text
~/uav_usv_ros1_ws/src/UAV_USV
```

---

## 3. Clone 工程

创建 ROS1 工作区：

```bash
mkdir -p ~/uav_usv_ros1_ws/src
cd ~/uav_usv_ros1_ws/src
```

Clone 私有仓库：

```bash
git clone https://github.com/balalaying/UAV_USV_ROS1_Gateway.git UAV_USV
```

进入工程：

```bash
cd ~/uav_usv_ros1_ws/src/UAV_USV
```

切换到当前 ROS1 开发分支：

```bash
git checkout ros1-param-align-20260920
```

如果仓库为 Private，需要先完成 GitHub 身份认证。

推荐：

```bash
gh auth login
gh auth setup-git
```

---

## 4. 安装系统与 ROS1 环境

执行：

```bash
cd ~/uav_usv_ros1_ws/src/UAV_USV
bash scripts/setup_system.sh
```

该脚本主要负责：

```text
Ubuntu 基础开发工具
ROS Noetic
Gazebo Classic 11
colcon
vcstool
rosdep
catkin 工具
CMake 3.27.9
基础 Shell 环境
```

脚本可能调用 `sudo`。

执行完成后建议重新加载：

```bash
source ~/.bashrc
```

---

## 5. 恢复 Gazebo Harmonic

执行：

```bash
cd ~/uav_usv_ros1_ws/src/UAV_USV
bash scripts/setup_gz_harmonic.sh
```

脚本会创建或恢复：

```text
~/gz_harmonic_ws
```

其中包含：

```text
gz-cmake3
gz-common5
gz-fuel_tools9
gz-gui8
gz-launch7
gz-math7
gz-msgs10
gz-physics7
gz-plugin2
gz-rendering8
gz-sensors8
gz-sim8
gz-tools2
gz-transport13
gz-utils2
sdformat14
```

所有源码仓库均固定到当前开发机记录的精确 Git commit。

版本记录：

```text
docs/environment/gz_harmonic_revisions.txt
```

源码清单：

```text
docs/environment/gz_harmonic_source.yaml
```

构建完成后：

```bash
source ~/gz_harmonic_ws/install/setup.bash
```

检查：

```bash
gz sim --version
```

预期为 Gazebo Sim 8.x。

---

## 6. 恢复第三方依赖

执行：

```bash
cd ~/uav_usv_ros1_ws/src/UAV_USV
bash scripts/setup_third_party.sh
```

该脚本负责恢复：

```text
third_party/ardupilot
third_party/ardupilot_gazebo
third_party/rapidjson
ArduPilot littlefs
```

其中 `ardupilot_gazebo` 会重新应用项目当前使用的自定义修改：

```text
third_party_patches/ardupilot_gazebo_ArduPilotPlugin.patch
```

第三方精确版本记录：

```text
third_party_patches/VERSIONS.txt
```

不要直接将第三方仓库更新到最新版本，否则可能导致当前仿真接口不兼容。

---

## 7. 构建 ROS1 工作区

执行：

```bash
cd ~/uav_usv_ros1_ws/src/UAV_USV
bash scripts/build_workspace.sh
```

脚本会执行项目依赖解析及：

```bash
catkin_make
```

成功后应生成：

```text
~/uav_usv_ros1_ws/devel/setup.bash
```

加载工作区：

```bash
source ~/uav_usv_ros1_ws/devel/setup.bash
```

---

## 8. 一键环境检查

全部恢复完成后执行：

```bash
cd ~/uav_usv_ros1_ws/src/UAV_USV
bash scripts/check_environment.sh
```

脚本会检查：

```text
Ubuntu 版本
ROS Noetic
catkin
colcon
vcstool
CMake
Gazebo Classic
Gazebo Sim
Gazebo Harmonic isolated install
ArduPilot
ardupilot_gazebo
RapidJSON
littlefs
ArduPilotPlugin 自定义 patch
ROS1 关键功能包
```

理想结果：

```text
Failures : 0
Environment is ready.
```

---

## 9. 完整恢复顺序

新电脑完成 GitHub 登录后，推荐严格按照以下顺序执行：

```bash
mkdir -p ~/uav_usv_ros1_ws/src
cd ~/uav_usv_ros1_ws/src

git clone https://github.com/balalaying/UAV_USV_ROS1_Gateway.git UAV_USV

cd UAV_USV
git checkout ros1-param-align-20260920

bash scripts/setup_system.sh

source ~/.bashrc

bash scripts/setup_gz_harmonic.sh

source ~/.bashrc

bash scripts/setup_third_party.sh

bash scripts/build_workspace.sh

bash scripts/check_environment.sh
```

最后：

```bash
source ~/uav_usv_ros1_ws/devel/setup.bash
```

---

## 10. 日常开发 Git 流程

当前远程仓库关系：

```text
origin
https://github.com/balalaying/UAV_USV_ROS1_Gateway.git

upstream
https://github.com/Suu0129/UAV_USV.git
```

当前 ROS1 开发分支：

```text
ros1-param-align-20260920
```

日常保存修改：

```bash
git status

git add <修改的文件>

git commit -m "说明本次修改"

git push
```

获取负责人原仓库的新提交：

```bash
git fetch upstream
```

不要误执行：

```bash
git push upstream
```

正常开发只向：

```text
origin
```

推送。

---

## 11. 当前重要恢复文件

```text
scripts/
├── setup_system.sh
├── setup_gz_harmonic.sh
├── setup_third_party.sh
├── build_workspace.sh
└── check_environment.sh
```

环境记录：

```text
docs/environment/
```

第三方版本及自定义 patch：

```text
third_party_patches/
├── VERSIONS.txt
└── ardupilot_gazebo_ArduPilotPlugin.patch
```

---

## 12. 注意事项

不要把以下生成目录提交到 Git：

```text
build/
devel/
log/
```

不要随意删除：

```text
third_party_patches/
docs/environment/
scripts/
```

它们是当前工程跨电脑恢复的重要组成部分。

迁移后如果出现仿真版本、插件加载或编译异常，优先运行：

```bash
bash scripts/check_environment.sh
```

再根据失败项目定位问题。
