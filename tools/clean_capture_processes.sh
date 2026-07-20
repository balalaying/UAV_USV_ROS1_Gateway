#!/usr/bin/env bash
set -u

echo "[1/4] Stopping UAV-USV processes..."

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

patterns=(
  "fleet_dynamic_capture_live_perception.launch.py"
  "fleet_dynamic_capture.launch.py"
  "camera_lidar_fusion.launch.py"
  "minimal_dynamic_capture.launch.py"
  "$project_root/install/"
  "capture_manager"
  "target_tracker"
  "capture_visualizer"
  "uav_dds_fleet_agent"
  "usv_fleet_agent"
  "MicroXRCEAgent"
  "gz sim"
  "gz-sim"
  "parameter_bridge"
  "lifecycle_manager"
)

exact_names=(
  "px4"
  "rviz2"
  "controller_server"
  "planner_server"
  "behavior_server"
  "bt_navigator"
  "waypoint_follower"
  "map_server"
)

for signal in INT TERM KILL; do
  for pattern in "${patterns[@]}"; do
    pkill "-${signal}" -u "$USER" -f "$pattern" 2>/dev/null || true
  done

  for name in "${exact_names[@]}"; do
    pkill "-${signal}" -u "$USER" -x "$name" 2>/dev/null || true
  done

  if [[ "$signal" != "KILL" ]]; then
    sleep 2
  fi
done

echo "[2/4] Stopping ROS 2 daemon..."
ros2 daemon stop >/dev/null 2>&1 || true

echo "[3/4] Remaining related processes:"
pgrep -a -u "$USER" -f \
'minimal_dynamic_capture|capture_manager|target_tracker|capture_visualizer|fleet_agent|MicroXRCEAgent|px4|gz sim|gz-sim|rviz2|controller_server|planner_server|bt_navigator|parameter_bridge' \
|| echo "None"

echo "[4/4] Relevant UDP ports:"
ss -lunp 2>/dev/null | grep -E '8888|14540|14550|14580|18570' \
|| echo "No related UDP ports detected"

echo "Cleanup complete."
