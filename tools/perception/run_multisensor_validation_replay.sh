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

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${ROOT_DIR}/install/setup.bash"
set -u
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

setsid ros2 launch uav_usv_perception \
  multisensor_fusion_validation.launch.py use_sim_time:=true \
  >"${LOG_DIR}/validation.log" 2>&1 &
PROCESS_GROUPS+=("$!")
sleep 3

setsid ros2 bag record --storage sqlite3 --output "${OUTPUT_BAG}" \
  /perception/ground_truth/tracks \
  /perception/lv_dot_ros2/dynamic_tracks \
  /perception/lv_dot/observations \
  /perception/uav_01/observations \
  /perception/uav_01/observation_status \
  /perception/fused/tracks \
  /perception/multisensor/metrics \
  >"${LOG_DIR}/record.log" 2>&1 &
RECORDER_PID="$!"
PROCESS_GROUPS+=("${RECORDER_PID}")
sleep 2

ros2 bag play "${INPUT_BAG}" --rate "${RATE}" --clock 100 \
  >"${LOG_DIR}/play.log" 2>&1
sleep 2
stop_process_group "${RECORDER_PID}"

echo "Multisensor validation bag: ${OUTPUT_BAG}"
