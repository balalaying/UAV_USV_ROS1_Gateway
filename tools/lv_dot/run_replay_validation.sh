#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 INPUT_BAG OUTPUT_BAG [RATE]" >&2
  exit 2
fi

INPUT_BAG="$1"
OUTPUT_BAG="$2"
RATE="${3:-1.0}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DOCKER_BIN="${DOCKER_BIN:-/home/dji/.local/lib/docker-static/docker}"
LOG_DIR="${OUTPUT_BAG}_logs"

if [[ ! -f "${INPUT_BAG}/metadata.yaml" ]]; then
  echo "Not a rosbag2 directory: ${INPUT_BAG}" >&2
  exit 2
fi
if [[ -e "${OUTPUT_BAG}" ]]; then
  echo "Output already exists: ${OUTPUT_BAG}" >&2
  exit 2
fi

mkdir -p "${LOG_DIR}"
PROCESS_GROUPS=()

stop_process_group() {
  local pgid="$1"
  if kill -0 -- "-${pgid}" 2>/dev/null; then
    kill -INT -- "-${pgid}" 2>/dev/null || true
    for _ in $(seq 1 15); do
      kill -0 -- "-${pgid}" 2>/dev/null || return 0
      sleep 0.2
    done
    kill -TERM -- "-${pgid}" 2>/dev/null || true
  fi
}

cleanup() {
  set +e
  for ((index=${#PROCESS_GROUPS[@]}-1; index>=0; index--)); do
    stop_process_group "${PROCESS_GROUPS[index]}"
  done
  "${DOCKER_BIN}" rm -f uav_usv_lv_dot >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

setsid ros2 launch uav_usv_perception perception_layer.launch.py \
  start_ground_truth_adapter:=false perception_source:=ground_truth \
  >"${LOG_DIR}/perception_layer.log" 2>&1 &
PROCESS_GROUPS+=("$!")

setsid ros2 launch uav_usv_perception lv_dot_shadow.launch.py \
  start_lv_dot_pose_adapter:=false \
  >"${LOG_DIR}/lv_dot_shadow.log" 2>&1 &
PROCESS_GROUPS+=("$!")

setsid env DOCKER_BIN="${DOCKER_BIN}" \
  "${ROOT_DIR}/tools/lv_dot/run_isolated.sh" \
  >"${LOG_DIR}/lv_dot_backend.log" 2>&1 &
PROCESS_GROUPS+=("$!")

# ROS 1 startup plus the two TCP bridge connections need a short warmup.
sleep 10

setsid ros2 bag record --storage sqlite3 --output "${OUTPUT_BAG}" \
  /perception/ground_truth/tracks \
  /perception/lv_dot/observations \
  /perception/lv_dot/shadow_metrics \
  /lv_dot/onboard_detector/dynamic_bboxes \
  /lv_dot/onboard_detector/velocity_visualizaton \
  /lv_dot/diagnostics/lidar_bboxes \
  /lv_dot/diagnostics/filtered_bboxes \
  /lv_dot/diagnostics/tracked_bboxes \
  /lv_dot/tuning/target_motion \
  >"${LOG_DIR}/record.log" 2>&1 &
RECORDER_PID="$!"
PROCESS_GROUPS+=("${RECORDER_PID}")
sleep 2

"${ROOT_DIR}/tools/lv_dot/replay_tuning_bag.sh" "${INPUT_BAG}" "${RATE}" \
  >"${LOG_DIR}/play.log" 2>&1
sleep 2
stop_process_group "${RECORDER_PID}"

echo "Replay validation bag: ${OUTPUT_BAG}"
