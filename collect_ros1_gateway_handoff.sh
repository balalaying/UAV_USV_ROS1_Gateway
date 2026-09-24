#!/usr/bin/env bash
set -u

source /opt/ros/noetic/setup.bash
source "$HOME/uav_usv_ros1_ws/devel/setup.bash"

desktop_dir="$(xdg-user-dir DESKTOP 2>/dev/null)"
[ -n "$desktop_dir" ] || desktop_dir="$HOME/Desktop"

stamp="$(date +%Y%m%d_%H%M%S)"
out="$desktop_dir/ROS1_GATEWAY_HANDOFF_$stamp"

mkdir -p "$out"/{runtime_samples,message_definitions,node_info,source_packages}

echo "===== 0. 基本环境 ====="
{
  echo "recorded_at=$(date -Is)"
  echo "hostname=$(hostname)"
  echo "user=$(whoami)"
  echo "pwd=$(pwd)"
  echo "ROS_DISTRO=${ROS_DISTRO:-UNKNOWN}"
  echo "ROS_MASTER_URI=${ROS_MASTER_URI:-UNKNOWN}"
  echo "ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH:-UNKNOWN}"
  echo
  echo "===== uname ====="
  uname -a
  echo
  echo "===== Python ====="
  python3 --version 2>&1
  echo
  echo "===== ROS ====="
  rosversion -d 2>&1
} > "$out/system_info.txt"

echo "===== 1. Git / 工程版本 ====="
repo="$HOME/uav_usv_ros1_ws/src/UAV_USV"
{
  echo "repo=$repo"
  echo
  cd "$repo" 2>/dev/null || exit 0
  echo "===== branch ====="
  git branch --show-current 2>&1
  echo
  echo "===== commit ====="
  git rev-parse HEAD 2>&1
  echo
  echo "===== status ====="
  git status --short 2>&1
  echo
  echo "===== remotes ====="
  git remote -v 2>&1
  echo
  echo "===== latest commit ====="
  git log -1 --oneline --decorate 2>&1
} > "$out/git_version.txt"

echo "===== 2. 当前启动进程 ====="
ps -eo pid,ppid,args | grep -E \
'roslaunch|rosrun|python.*uav_usv|fleet_|algorithm|agent|sim_vehicle|arducopter|ardurover|mavlink|gazebo|gz sim|gzserver|gzclient' | \
grep -v grep > "$out/relevant_processes.txt"

echo "===== 3. ROS 节点 / Topic / Service ====="
timeout 10s rosnode list | sort > "$out/rosnode_list.txt" 2>&1 || true
timeout 10s rostopic list -v > "$out/rostopic_list_verbose.txt" 2>&1 || true
timeout 10s rosservice list | sort > "$out/rosservice_list.txt" 2>&1 || true
timeout 10s rosparam list | sort > "$out/rosparam_list.txt" 2>&1 || true

echo "===== 4. 保存完整 ROS 参数快照 ====="
timeout 20s rosparam dump "$out/rosparams_all.yaml" 2>&1 || true

echo "===== 5. 核心节点详细信息 ====="
for node in \
  /cooperative_algorithm_controller \
  /fleet_base_station \
  /base_station_service \
  /fleet_world_model \
  /uav_01_agent \
  /uav_02_agent \
  /uav_03_agent \
  /usv_01_agent \
  /usv_02_agent \
  /usv_03_agent
do
  safe_name="$(echo "$node" | sed 's#^/##; s#/#_#g')"
  timeout 6s rosnode info "$node" \
    > "$out/node_info/${safe_name}.txt" 2>&1 || true
done

echo "===== 6. 核心 Topic 类型和连接关系 ====="
topics="
/fleet/algorithm/action
/fleet/algorithm/status
/fleet/algorithm/assignments
/fleet/command
/fleet/command_ack
/fleet/control_lease
/fleet/state
/fleet/world_model
/fleet/world_model_summary
/base_station/state
/base_station/events
/capture/state
/capture/roles
/fleet/sensor_status
"

: > "$out/key_topic_interfaces.txt"
for topic in $topics
do
  {
    echo
    echo "=================================================="
    echo "TOPIC: $topic"
    echo "=================================================="
    echo "[type]"
    timeout 5s rostopic type "$topic" 2>&1 || true
    echo
    echo "[info]"
    timeout 6s rostopic info "$topic" 2>&1 || true
  } >> "$out/key_topic_interfaces.txt"
done

echo "===== 7. 保存关键自定义消息定义 ====="
for type in \
  uav_usv_interfaces/FleetCommand \
  uav_usv_interfaces/CommandAck \
  uav_usv_interfaces/ControlLease \
  uav_usv_interfaces/VehicleState \
  uav_usv_interfaces/CaptureState \
  uav_usv_interfaces/CaptureAssignmentArray \
  uav_usv_interfaces/SensorStatus \
  uav_usv_interfaces/TrackedObjectArray
do
  safe_name="$(echo "$type" | tr '/' '_')"
  timeout 5s rosmsg show "$type" \
    > "$out/message_definitions/${safe_name}.txt" 2>&1 || true
done

echo "===== 8. 采集真实运行消息 ====="
echo "--- algorithm status ---"
timeout 8s rostopic echo -n 1 /fleet/algorithm/status \
  > "$out/runtime_samples/algorithm_status.txt" 2>&1 || true
echo "--- algorithm assignments ---"
timeout 8s rostopic echo -n 1 /fleet/algorithm/assignments \
  > "$out/runtime_samples/algorithm_assignments.txt" 2>&1 || true
echo "--- control lease ---"
timeout 8s rostopic echo -n 1 /fleet/control_lease \
  > "$out/runtime_samples/control_lease.txt" 2>&1 || true
echo "--- fleet state ---"
timeout 8s rostopic echo -n 3 /fleet/state \
  > "$out/runtime_samples/fleet_state.txt" 2>&1 || true
echo "--- world model ---"
timeout 8s rostopic echo -n 1 /fleet/world_model \
  > "$out/runtime_samples/world_model.txt" 2>&1 || true
echo "--- world model summary ---"
timeout 8s rostopic echo -n 1 /fleet/world_model_summary \
  > "$out/runtime_samples/world_model_summary.txt" 2>&1 || true
echo "--- base station state ---"
timeout 8s rostopic echo -n 1 /base_station/state \
  > "$out/runtime_samples/base_station_state.txt" 2>&1 || true
echo "--- capture state ---"
timeout 8s rostopic echo -n 1 /capture/state \
  > "$out/runtime_samples/capture_state.txt" 2>&1 || true
echo "--- capture roles ---"
timeout 8s rostopic echo -n 1 /capture/roles \
  > "$out/runtime_samples/capture_roles.txt" 2>&1 || true
echo "--- command ack（若当前刚好没有消息，空着也没关系） ---"
timeout 8s rostopic echo -n 1 /fleet/command_ack \
  > "$out/runtime_samples/command_ack.txt" 2>&1 || true

echo "===== 9. 记录核心参数 ====="
{
  echo "===== fleet_base_station ====="
  for p in owner_id lease_duration monitor_only auto_demo topic_namespace uav_ids usv_ids
  do
    echo
    echo "--- $p ---"
    timeout 5s rosparam get "/fleet_base_station/$p" 2>&1 || true
  done

  echo
  echo "===== cooperative_algorithm_controller ====="
  for p in auto_start mode publish_commands control_rate state_timeout world_model_timeout \
           uav_ids usv_ids capture_min_agents capture_radius
  do
    echo
    echo "--- $p ---"
    timeout 5s rosparam get "/cooperative_algorithm_controller/$p" 2>&1 || true
  done

  echo
  echo "===== UAV / USV Agent MAVLink ====="
  for v in uav_01 uav_02 uav_03 usv_01 usv_02 usv_03
  do
    echo
    echo "--- $v ---"
    timeout 5s rosparam get "/${v}_agent/vehicle_id" 2>&1 || true
    timeout 5s rosparam get "/${v}_agent/mavlink_url" 2>&1 || true
  done
} > "$out/key_params.txt"

echo "===== 10. 当前网络监听信息 ====="
{
  echo "===== listening sockets ====="
  ss -lntup 2>&1
  echo
  echo "===== established TCP related to ROS / Python / ArduPilot ====="
  ss -ntp 2>&1
} > "$out/network_sockets.txt"

echo "===== 11. 保存关键 ROS1 源码包 ====="
packages="
uav_usv_fleet_gateway
uav_usv_interfaces
uav_usv_bringup
uav_usv_cooperative_algorithms
uav_usv_base_station
uav_usv_mission
uav_usv_uav_control
uav_usv_usv_control
uav_usv_ros1_compat
"

: > "$out/source_package_paths.txt"
for pkg in $packages
do
  pkg_path="$(rospack find "$pkg" 2>/dev/null || true)"
  if [ -n "$pkg_path" ] && [ -d "$pkg_path" ]; then
    echo "$pkg -> $pkg_path" >> "$out/source_package_paths.txt"
    tar -czf "$out/source_packages/${pkg}.tar.gz" \
      -C "$(dirname "$pkg_path")" \
      "$(basename "$pkg_path")" 2>/dev/null || true
  else
    echo "$pkg -> NOT FOUND" >> "$out/source_package_paths.txt"
  fi
done

echo "===== 12. 单独保存 Gateway 文件树 ====="
gateway_pkg="$(rospack find uav_usv_fleet_gateway 2>/dev/null || true)"
if [ -n "$gateway_pkg" ]; then
  {
    echo "gateway_pkg=$gateway_pkg"
    echo
    find "$gateway_pkg" -maxdepth 5 -type f | sort
  } > "$out/gateway_package_tree.txt"
else
  echo "uav_usv_fleet_gateway NOT FOUND" > "$out/gateway_package_tree.txt"
fi

echo "===== 13. 搜索 Gateway / Lease / 状态机相关实现 ====="
grep -Rni -E \
'control_lease|ControlLease|lease_id|algorithm/action|algorithm/status|command_ack|FleetCommand|CommandAck|SUCCEEDED|FAILED|CANCELED|CAPTURE|ESCORT|PAUSE|RESUME|STOP|last_error|phase|publish_commands|websocket|WebSocket|protobuf' \
"$repo/src" 2>/dev/null \
> "$out/gateway_algorithm_lease_source_hits.txt" || true

echo "===== 14. 保存 launch / config 文件清单 ====="
find "$repo/src" \
  \( -path '*/launch/*' \
  -o -path '*/config/*' \
  -o -name '*.launch' \
  -o -name '*.yaml' \
  -o -name '*.yml' \
  -o -name '*.json' \) \
  -type f | sort \
  > "$out/launch_and_config_files.txt"

echo "===== 15. 找出当前主启动文件内容 ====="
bringup_pkg="$(rospack find uav_usv_bringup 2>/dev/null || true)"
if [ -n "$bringup_pkg" ]; then
  find "$bringup_pkg" \
    -type f \
    -name 'heterogeneous_332_qt_ros1.launch' \
    -exec cp {} "$out/" \; 2>/dev/null || true
fi

echo "===== 16. 生成 SHA256 ====="
find "$out" -type f -print0 | sort -z | \
xargs -0 sha256sum \
> "$out/SHA256SUMS.txt"

echo "===== 17. 最终打包 ====="
final="${out}.tar.gz"
tar -czf "$final" \
  -C "$(dirname "$out")" \
  "$(basename "$out")"

echo
echo "========================================"
echo "采集完成"
echo "========================================"
echo
echo "只需要把下面这个文件发给对接同学："
echo
echo "$final"
echo
ls -lh "$final"
