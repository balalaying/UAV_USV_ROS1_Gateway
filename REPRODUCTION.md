# UAV_USV ROS1 Gateway 协同仿真系统复现说明

## 一、项目概述

本项目面向无人机（UAV）与无人船（USV）异构集群协同仿真与通信测试，当前版本基于 **Ubuntu 20.04、ROS1 Noetic、Gazebo Sim 8、Gazebo Classic 11** 构建，并集成了无人机/无人船仿真环境、ArduPilot、协同控制算法、舰队 Gateway、感知模块以及部分真实传感器数据接入工具。

当前仓库是在原 UAV_USV 工程基础上完成的 ROS1 迁移与二次集成版本，主要工作包括：

1. 将原有部分 ROS2 功能迁移至 ROS1 Noetic；
2. 建立 3 架无人机与 3 艘无人船的异构协同仿真环境；
3. 接入无人机与无人船控制模块；
4. 接入护航、围捕等协同算法；
5. 建立 ROS 与前后端之间的 Gateway 通信链路；
6. 整理 Gazebo Harmonic 源码环境及第三方依赖；
7. 增加 SAN60 电子探测仪等真实传感器的数据接入与测试工具；
8. 提供系统环境、第三方依赖、编译和环境检查脚本，以支持跨计算机复现。

项目公开仓库：

```text
https://github.com/balalaying/UAV_USV_ROS1_Gateway.git
```

当前推荐复现分支：

```text
ros1-param-align-20260920
```

---

## 二、复现目标

完成本说明中的操作后，应能够在新的 Ubuntu 20.04 计算机上恢复当前工程的主要软件环境，并实现以下目标：

```text
GitHub 项目源码
        │
        ▼
Ubuntu 20.04
        │
        ▼
ROS1 Noetic
        │
        ├── Gazebo Classic 11
        │
        └── Gazebo Sim 8
                │
                ▼
        UAV / USV 仿真环境
                │
        ┌───────┴────────┐
        ▼                ▼
   ArduPilot          ROS1节点
        │                │
        └───────┬────────┘
                ▼
           UAV / USV控制
                │
                ▼
          协同任务算法
                │
                ▼
             Gateway
                │
                ▼
          前端 / 后端系统
```

正常情况下，复现完成后应至少满足：

```text
ROS1工作区可以成功编译；
Gazebo Sim 8能够正常启动；
heterogeneous_332世界文件能够加载；
ROS1关键功能包能够被rospack识别；
ArduPilot等第三方依赖版本正确；
Gateway及协同算法源码完整；
环境检查脚本无关键错误。
```

---

## 三、推荐系统环境

本项目当前主要在以下环境中完成开发与验证。

| 项目 | 推荐版本 |
|---|---|
| 操作系统 | Ubuntu 20.04.6 LTS |
| ROS | ROS1 Noetic |
| Python | Python 3.8 |
| GCC/G++ | 9.4.0 |
| CMake | 3.27.9 |
| Gazebo Classic | 11.15.x |
| Gazebo Sim | 8.x |
| Git | 2.25.x 或以上 |
| colcon | Ubuntu 软件源版本 |
| vcstool | 0.3.x |

其中需要特别说明：

本项目使用的 Gazebo Sim 8 并不是直接通过 Ubuntu 20.04 系统中的 `apt install gz-harmonic` 安装，而是通过源码工作区进行编译。

默认源码工作区为：

```bash
~/gz_harmonic_ws
```

完整环境版本快照保存在：

```text
docs/environment/
```

其中包含：

```text
system_versions.txt
tool_sources.txt
apt_manual_packages.txt
ros_noetic_packages.txt
gazebo_gz_packages.txt
python3_pip_freeze.txt
gz_harmonic_source.yaml
gz_harmonic_revisions.txt
```

这些文件用于记录原开发计算机的实际环境，发生版本兼容问题时可用于比对。

---

## 四、项目目录结构

推荐按照以下目录结构进行复现：

```text
~/uav_usv_ros1_ws/
├── build/
├── devel/
└── src/
    └── UAV_USV/
```

其中：

```text
~/uav_usv_ros1_ws
```

为 ROS1 Catkin 工作区，

```text
~/uav_usv_ros1_ws/src/UAV_USV
```

为本项目 Git 仓库。

仓库主要目录如下：

```text
UAV_USV/
├── docs/
│   └── environment/
│
├── scripts/
│   ├── setup_system.sh
│   ├── setup_gz_harmonic.sh
│   ├── setup_third_party.sh
│   ├── build_workspace.sh
│   └── check_environment.sh
│
├── src/
│   ├── uav_usv_base_station/
│   ├── uav_usv_bringup/
│   ├── uav_usv_cooperative_algorithms/
│   ├── uav_usv_fleet_gateway/
│   ├── uav_usv_gazebo/
│   ├── uav_usv_interfaces/
│   ├── uav_usv_lv_dot/
│   ├── uav_usv_lv_dot_core/
│   ├── uav_usv_mission/
│   ├── uav_usv_perception/
│   ├── uav_usv_ros1_compat/
│   ├── uav_usv_uav_control/
│   └── uav_usv_usv_control/
│
├── third_party/
│
├── third_party_patches/
│   ├── VERSIONS.txt
│   └── ardupilot_gazebo_ArduPilotPlugin.patch
│
├── tools/
│   └── harogic_stream/
│
├── MIGRATION.md
├── REPRODUCTION.md
└── README.md
```

---

## 五、从零开始复现

### 5.1 创建 ROS1 工作区

打开终端：

```bash
mkdir -p ~/uav_usv_ros1_ws/src
cd ~/uav_usv_ros1_ws/src
```

克隆项目：

```bash
git clone https://github.com/balalaying/UAV_USV_ROS1_Gateway.git UAV_USV
```

进入项目：

```bash
cd UAV_USV
```

切换到当前推荐 ROS1 分支：

```bash
git checkout ros1-param-align-20260920
```

检查：

```bash
git branch
```

应能够看到：

```text
* ros1-param-align-20260920
```

---

## 六、安装 Ubuntu 与 ROS1 基础环境

仓库中提供：

```text
scripts/setup_system.sh
```

用于安装或检查项目所需的基础开发环境。

执行：

```bash
cd ~/uav_usv_ros1_ws/src/UAV_USV

bash scripts/setup_system.sh
```

脚本主要负责准备：

```text
基础编译工具；
Git；
Python3；
pip；
ROS Noetic；
rosdep；
catkin；
vcstool；
colcon；
Gazebo Classic 11；
CMake 3.27.9；
相关 Shell 环境。
```

执行过程中可能需要输入当前 Ubuntu 用户的 `sudo` 密码。

完成后重新加载终端环境：

```bash
source ~/.bashrc
```

检查 ROS：

```bash
echo $ROS_DISTRO
```

预期输出：

```text
noetic
```

进一步检查：

```bash
rosversion -d
```

预期：

```text
noetic
```

---

## 七、恢复 Gazebo Sim 8 源码环境

本项目当前 Gazebo Sim 环境位于：

```text
~/gz_harmonic_ws
```

执行：

```bash
cd ~/uav_usv_ros1_ws/src/UAV_USV

bash scripts/setup_gz_harmonic.sh
```

该脚本会根据仓库中记录的源码版本恢复 Gazebo 相关组件。

当前共锁定 16 个 Gazebo / SDFormat 源码仓库，包括：

```text
gz-cmake3
gz-common5
gz-fuel-tools9
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

各源码仓库的精确 Git commit 保存在：

```text
docs/environment/gz_harmonic_revisions.txt
```

源码仓库配置保存在：

```text
docs/environment/gz_harmonic_source.yaml
```

Gazebo 源码编译耗时与计算机性能有关，第一次执行通常是整个复现过程中耗时最长的步骤之一。

完成后执行：

```bash
source ~/gz_harmonic_ws/install/setup.bash
```

检查：

```bash
gz sim --version
```

正常情况下应显示：

```text
Gazebo Sim 8.x
```

如果新终端中无法找到 `gz`，可执行：

```bash
source ~/.bashrc
```

或者：

```bash
source ~/gz_harmonic_ws/install/setup.bash
```

---

## 八、恢复第三方依赖

本项目没有直接将完整 ArduPilot、RapidJSON 等第三方仓库上传至当前 Git 仓库，而是记录其来源和精确版本。

执行：

```bash
cd ~/uav_usv_ros1_ws/src/UAV_USV

bash scripts/setup_third_party.sh
```

该脚本将恢复：

```text
ArduPilot
ArduPilot Gazebo
RapidJSON
ArduPilot littlefs
```

当前第三方版本信息保存在：

```text
third_party_patches/VERSIONS.txt
```

其中 ArduPilot Gazebo 包含本项目使用的一处本地修改。

对应 patch：

```text
third_party_patches/ardupilot_gazebo_ArduPilotPlugin.patch
```

恢复脚本会自动检查该 patch 是否已经应用，如果尚未应用则进行恢复。

复现过程中不建议直接将这些第三方仓库更新到最新版，否则可能导致：

```text
Gazebo插件接口变化；
ArduPilot接口变化；
编译错误；
仿真数据异常；
原有ROS控制逻辑失效。
```

---

## 九、编译 ROS1 工作区

完成系统、Gazebo 和第三方依赖恢复后，执行：

```bash
cd ~/uav_usv_ros1_ws/src/UAV_USV

bash scripts/build_workspace.sh
```

脚本会首先执行 ROS 依赖解析，然后在：

```text
~/uav_usv_ros1_ws
```

中执行 Catkin 编译。

也可以手动执行：

```bash
cd ~/uav_usv_ros1_ws

source /opt/ros/noetic/setup.bash

catkin_make
```

编译成功后应生成：

```text
~/uav_usv_ros1_ws/devel/setup.bash
```

加载工作区：

```bash
source ~/uav_usv_ros1_ws/devel/setup.bash
```

检查某个 ROS 包，例如：

```bash
rospack find uav_usv_gazebo
```

如果正常，应返回类似：

```text
/home/<用户名>/uav_usv_ros1_ws/src/UAV_USV/src/uav_usv_gazebo
```

---

## 十、一键检查复现环境

项目提供：

```text
scripts/check_environment.sh
```

完成环境安装和工作区编译后，执行：

```bash
cd ~/uav_usv_ros1_ws/src/UAV_USV

bash scripts/check_environment.sh
```

该脚本会检查：

```text
Ubuntu版本；
ROS Noetic；
Git；
Python3；
catkin_make；
colcon；
vcstool；
rosdep；
CMake；
Gazebo Classic；
Gazebo Sim；
Gazebo源码工作区；
ArduPilot；
ardupilot_gazebo；
RapidJSON；
littlefs；
ArduPilot Gazebo自定义patch；
ROS1关键功能包。
```

理想情况下最后应显示：

```text
Failures : 0
Environment is ready.
```

如果存在：

```text
[FAIL]
```

建议先解决所有 FAIL 项目，再启动完整仿真系统。

`WARN` 一般表示版本或配置与原开发环境存在差异，需要结合实际情况判断。

---

## 十一、启动 Gazebo 332 异构协同仿真世界

本项目当前主要使用的 3 UAV + 3 USV 世界为：

```text
src/uav_usv_gazebo/worlds/heterogeneous_332.sdf
```

建议打开新的终端：

```bash
source /opt/ros/noetic/setup.bash
source ~/gz_harmonic_ws/install/setup.bash
source ~/uav_usv_ros1_ws/devel/setup.bash

cd ~/uav_usv_ros1_ws/src/UAV_USV
```

Gazebo 世界启动脚本位于：

```text
src/uav_usv_gazebo/tools/run_gz_world.sh
```

可执行：

```bash
bash src/uav_usv_gazebo/tools/run_gz_world.sh
```

如果需要确认脚本当前实际启动的世界，可检查：

```bash
grep -n "heterogeneous_332" \
src/uav_usv_gazebo/tools/run_gz_world.sh
```

正常启动后，应能够看到 Gazebo Sim 窗口以及 UAV、USV 等仿真实体。

如果没有 GUI，但终端中进程仍在运行，应首先检查：

```bash
gz sim --version
```

以及：

```bash
echo $GZ_CONFIG_PATH
```

---

## 十二、ArduPilot 仿真接入

本项目第三方 ArduPilot 位于：

```text
third_party/ardupilot
```

相关仿真接入脚本主要位于：

```text
src/uav_usv_bringup/scripts/
```

项目中包含针对 332 场景的启动脚本，例如：

```text
start_ardupilot_332.sh
```

在运行前应首先加载：

```bash
source /opt/ros/noetic/setup.bash
source ~/gz_harmonic_ws/install/setup.bash
source ~/uav_usv_ros1_ws/devel/setup.bash
```

随后进入：

```bash
cd ~/uav_usv_ros1_ws/src/UAV_USV
```

再根据当前脚本参数启动 ArduPilot。

由于 ArduPilot 启动参数和仿真实例配置可能随着工程后续开发变化，复现时应以：

```text
src/uav_usv_bringup/scripts/
```

中的当前版本脚本为准。

---

## 十三、ROS1 基础验证

启动 ROS 系统前，可单独验证 ROS Master。

终端 1：

```bash
source /opt/ros/noetic/setup.bash
roscore
```

终端 2：

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_usv_ros1_ws/devel/setup.bash

rosnode list
```

如果 ROS Master 正常，应不再出现：

```text
ERROR: Unable to communicate with master!
```

查看当前 ROS Topic：

```bash
rostopic list
```

如果完整系统已经启动，可以进一步检查舰队相关 Topic：

```bash
rostopic list | grep fleet
```

以及查看指定 Topic 的发布频率，例如：

```bash
rostopic hz /fleet/state
```

实际 Topic 名称以当前项目代码和启动配置为准。

---

## 十四、Gateway 模块

ROS 与前后端之间的数据交互主要由：

```text
uav_usv_fleet_gateway
```

负责。

Gateway 代码位于：

```text
src/uav_usv_fleet_gateway/
```

核心执行脚本位于：

```text
src/uav_usv_fleet_gateway/scripts/
```

其中包含：

```text
fleet_gateway
```

使用前应加载 ROS1 工作区：

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_usv_ros1_ws/devel/setup.bash
```

然后根据当前 Gateway 配置启动对应节点。

Gateway 主要承担：

```text
ROS Topic读取；
舰队状态汇总；
前后端数据封装；
WebSocket通信；
控制指令接收；
ROS控制命令下发。
```

在只验证 ROS/Gazebo 仿真时，Gateway 并非必须启动。

需要进行完整前后端联调时，再启动 Gateway。

---

## 十五、协同算法模块

协同算法主要位于：

```text
src/uav_usv_cooperative_algorithms/
```

当前工程已经接入包括护航、围捕等任务逻辑。

算法模块一般通过 ROS Topic 接收：

```text
舰队状态；
无人机状态；
无人船状态；
任务目标；
目标位置信息。
```

并向控制层输出：

```text
任务指令；
目标点；
速度或航迹规划结果；
UAV/USV协同行为。
```

进行算法复现前，应先确认：

```text
ROS Master正常；
Gazebo仿真实体已经启动；
UAV/USV状态Topic能够正常发布。
```

然后再启动算法模块。

如果算法没有输出，首先检查：

```bash
rosnode list
rostopic list
```

其次检查算法所依赖 Topic：

```bash
rostopic echo <topic_name>
```

或：

```bash
rostopic hz <topic_name>
```

---

## 十六、SAN60 电子探测仪工具

项目中还保留了 SAN60 电子探测仪相关数据传输和 ROS 接入工具：

```text
tools/harogic_stream/
```

该部分主要用于真实设备实验，并不是完成基础 Gazebo 仿真复现的必要条件。

相关功能包括：

```text
SAN60数据采集；
频谱数据网络传输；
TCP中继；
接收端频谱显示；
ROS频谱数据发布；
rosbag数据记录。
```

如果只复现 UAV/USV 仿真，可以暂时忽略该目录。

如需进行 SAN60 实机测试，应根据：

```text
tools/harogic_stream/README.md
```

单独配置设备 IP、TCP 端口和采集环境。

---

## 十七、推荐复现顺序

新电脑上建议严格按照以下顺序操作：

```bash
# 1. 创建工作区
mkdir -p ~/uav_usv_ros1_ws/src
cd ~/uav_usv_ros1_ws/src

# 2. 下载代码
git clone https://github.com/balalaying/UAV_USV_ROS1_Gateway.git UAV_USV

cd UAV_USV

# 3. 切换ROS1开发分支
git checkout ros1-param-align-20260920

# 4. 安装Ubuntu与ROS环境
bash scripts/setup_system.sh

source ~/.bashrc

# 5. 编译Gazebo Sim源码环境
bash scripts/setup_gz_harmonic.sh

source ~/.bashrc

# 6. 恢复ArduPilot等第三方依赖
bash scripts/setup_third_party.sh

# 7. 编译ROS1工作区
bash scripts/build_workspace.sh

# 8. 环境检查
bash scripts/check_environment.sh

# 9. 加载ROS工作区
source ~/uav_usv_ros1_ws/devel/setup.bash
```

如果：

```text
Failures : 0
```

即可继续启动 Gazebo 世界、ArduPilot、协同算法和 Gateway。

---

## 十八、每次重新打开终端后的操作

正常情况下，新开终端至少需要加载：

```bash
source /opt/ros/noetic/setup.bash
source ~/gz_harmonic_ws/install/setup.bash
source ~/uav_usv_ros1_ws/devel/setup.bash
```

如果已经由安装脚本写入 `.bashrc`，部分环境可能会自动加载。

可通过以下命令确认：

```bash
echo $ROS_DISTRO
which gz
rospack find uav_usv_gazebo
```

---

## 十九、常见问题

### 19.1 ROS Master 无法连接

出现：

```text
ERROR: Unable to communicate with master!
```

说明 ROS Master 尚未启动。

执行：

```bash
roscore
```

然后在其他终端加载 ROS 环境后重新执行命令。

---

### 19.2 找不到 `gz`

执行：

```bash
source ~/gz_harmonic_ws/install/setup.bash
```

然后：

```bash
which gz
gz sim --version
```

---

### 19.3 找不到 ROS 包

例如：

```text
package 'uav_usv_gazebo' not found
```

首先确认已经编译：

```bash
cd ~/uav_usv_ros1_ws
catkin_make
```

然后：

```bash
source ~/uav_usv_ros1_ws/devel/setup.bash
```

再次检查：

```bash
rospack find uav_usv_gazebo
```

---

### 19.4 Gazebo 插件找不到

如果出现类似：

```text
Failed to load system plugin
shared library not found
```

应依次检查：

```text
Gazebo源码工作区是否已经编译；
第三方依赖是否已经恢复；
插件是否已经生成.so文件；
Gazebo环境变量是否已经加载。
```

建议先运行：

```bash
bash scripts/check_environment.sh
```

---

### 19.5 ArduPilot 无法启动

首先检查：

```text
third_party/ardupilot
```

是否存在。

然后确认：

```bash
git -C third_party/ardupilot rev-parse HEAD
```

如果依赖缺失，重新执行：

```bash
bash scripts/setup_third_party.sh
```

---

### 19.6 编译后仍然使用错误环境

有时终端中同时加载过 ROS2 和 ROS1 环境，会造成路径污染。

建议关闭当前终端，重新打开，再仅加载：

```bash
source /opt/ros/noetic/setup.bash
source ~/gz_harmonic_ws/install/setup.bash
source ~/uav_usv_ros1_ws/devel/setup.bash
```

---

## 二十、复现成功判据

基础环境复现成功建议满足以下条件：

```text
1. Ubuntu系统为20.04；
2. ROS_DISTRO为noetic；
3. rosversion -d返回noetic；
4. gazebo --version能够正常执行；
5. gz sim --version能够正常执行；
6. catkin_make编译成功；
7. devel/setup.bash存在；
8. rospack能够找到项目关键ROS包；
9. 第三方依赖commit与记录一致；
10. check_environment.sh中Failures为0。
```

完整仿真复现还应满足：

```text
heterogeneous_332.sdf成功加载；
Gazebo内UAV/USV实体正常生成；
ROS节点能够正常启动；
舰队状态Topic存在；
UAV/USV控制链路能够工作；
协同算法能够获取ROS数据；
Gateway能够读取ROS状态并完成数据交互。
```

若需要进行实机与前后端联调，还需进一步验证：

```text
Gateway WebSocket；
控制指令下发；
真实传感器接入；
语义盒子通信链路；
SAN60数据传输。
```

这些属于完整系统联调内容，不是基础软件环境复现的必要条件。

---

## 二十一、版本固定与开发注意事项

本项目对 Gazebo、ArduPilot 等依赖进行了版本固定。

不建议在首次复现过程中主动执行：

```bash
git pull
```

更新第三方依赖。

也不建议直接安装与本项目不同的大版本：

```text
ROS2 Humble
Ubuntu 22.04
其他Gazebo Sim版本
最新版ArduPilot
```

首先应严格按照当前版本完成复现。

在确认现有版本能够正常运行之后，再单独进行版本升级实验。

项目环境版本可参考：

```text
docs/environment/
```

第三方依赖版本可参考：

```text
third_party_patches/VERSIONS.txt
```

---

## 二十二、仓库关系说明

本仓库为 ROS1 迁移与集成版本。

当前用户仓库：

```text
origin
balalaying/UAV_USV_ROS1_Gateway
```

原项目仓库：

```text
upstream
Suu0129/UAV_USV
```

本仓库当前主要开发分支：

```text
ros1-param-align-20260920
```

普通复现用户只需要克隆当前公开仓库即可，不需要配置 `upstream`。

需要继续参与原项目协同开发的开发者，可以进一步添加：

```bash
git remote add upstream https://github.com/Suu0129/UAV_USV.git
```

---

## 二十三、说明

当前版本以：

```text
Ubuntu 20.04
+
ROS1 Noetic
+
Gazebo Sim 8
```

作为主要验证环境。

其他 Linux 发行版、Ubuntu 版本以及 ROS2 环境尚不能保证能够按照本说明直接复现。

由于 ROS、Gazebo、ArduPilot 和第三方依赖之间存在较强版本关联，建议首次复现时优先保持与本文记录一致的软件版本。

如果在复现过程中出现问题，建议按照以下顺序进行排查：

```text
系统环境
   ↓
ROS1
   ↓
Gazebo Sim
   ↓
第三方依赖
   ↓
Catkin编译
   ↓
Gazebo世界
   ↓
ArduPilot
   ↓
ROS控制节点
   ↓
协同算法
   ↓
Gateway
   ↓
真实设备
```

基础环境问题解决后，再进行上层模块联调，可以明显降低排错复杂度。

---

## 附：最简复现命令

对于已经熟悉 Ubuntu、ROS 和 Git 的用户，可以直接按照下面的流程执行：

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

source ~/uav_usv_ros1_ws/devel/setup.bash
```

当环境检查结果为：

```text
Failures : 0
Environment is ready.
```

即可进入 UAV/USV 仿真、算法及 Gateway 的进一步启动与测试阶段。
