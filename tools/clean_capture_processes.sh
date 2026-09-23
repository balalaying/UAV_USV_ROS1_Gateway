#!/usr/bin/env bash
set -u

echo "[1/3] Stopping UAV-USV ROS 1, ArduPilot and Gazebo processes..."

patterns=(
  "heterogeneous_332_qt_ros1.launch"
  "capture_manager"
  "target_tracker"
  "capture_visualizer"
  "uav_fleet_agent"
  "usv_gz_fleet_agent"
  "arducopter"
  "ardurover"
  "gz sim"
  "gz-sim"
)

for signal in INT TERM KILL; do
  for pattern in "${patterns[@]}"; do
    pkill "-${signal}" -u "$USER" -f "$pattern" 2>/dev/null || true
  done
  if [[ "$signal" != "KILL" ]]; then
    sleep 2
  fi
done

echo "[2/3] Remaining related processes:"
pgrep -a -u "$USER" -f \
'heterogeneous_332|capture_manager|target_tracker|capture_visualizer|fleet_agent|arducopter|ardurover|gz sim|gz-sim' \
|| echo "None"

echo "[3/3] Relevant ArduPilot ports:"
ss -ltnup 2>/dev/null | grep -E \
'5760|5770|5780|5790|5800|5810|9002|9012|9022|9032|9042|9052' \
|| echo "No related ports detected"

echo "Cleanup complete. Runtime logs were retained."
