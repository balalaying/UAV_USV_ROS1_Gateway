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
    for _ in $(seq 1 20); do
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
}
trap cleanup EXIT INT TERM

setsid ros2 launch uav_usv_lv_dot_ros2 lv_dot_ros2.launch.py \
  vehicle_id:=usv_01 \
  points_topic:=/perception/usv_01/points_filtered \
  output_frame:=map use_sim_time:=true \
  >"${LOG_DIR}/detector.log" 2>&1 &
PROCESS_GROUPS+=("$!")
sleep 3

setsid python3 "${ROOT_DIR}/tools/lv_dot/sample_phase2_resources.py" \
  "${LOG_DIR}/resources.jsonl" \
  >"${LOG_DIR}/resources.log" 2>&1 &
RESOURCE_PID="$!"
PROCESS_GROUPS+=("${RESOURCE_PID}")

setsid ros2 bag record --storage sqlite3 --output "${OUTPUT_BAG}" \
  /perception/usv_01/points_filtered \
  /perception/lv_dot_ros2/diagnostics/lidar_bboxes \
  /perception/lv_dot_ros2/diagnostics \
  /perception/lv_dot_ros2/observations \
  >"${LOG_DIR}/record.log" 2>&1 &
RECORDER_PID="$!"
PROCESS_GROUPS+=("${RECORDER_PID}")
sleep 2

ros2 bag play "${INPUT_BAG}" --rate "${RATE}" --clock 100 \
  --topics /perception/usv_01/points_filtered /tf /tf_static \
  >"${LOG_DIR}/play.log" 2>&1
sleep 2
stop_process_group "${RECORDER_PID}"
stop_process_group "${RESOURCE_PID}"

echo "Phase 2 replay bag: ${OUTPUT_BAG}"
