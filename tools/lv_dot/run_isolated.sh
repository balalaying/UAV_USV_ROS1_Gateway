#!/usr/bin/env bash
set -euo pipefail

DOCKER_BIN="${DOCKER_BIN:-docker}"
LV_DOT_IMAGE="${LV_DOT_IMAGE:-uav-usv/lv-dot-noetic:449bf2c}"
ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
LV_DOT_INGRESS_PORT="${LV_DOT_INGRESS_PORT:-19090}"
LV_DOT_EGRESS_PORT="${LV_DOT_EGRESS_PORT:-19091}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LV_DOT_CONFIG="${LV_DOT_CONFIG:-${ROOT_DIR}/containers/lv_dot_noetic/detector_param.yaml}"

if [[ ! -f "${LV_DOT_CONFIG}" ]]; then
  echo "LV-DOT config does not exist: ${LV_DOT_CONFIG}" >&2
  exit 2
fi

cleanup() {
  "${DOCKER_BIN}" rm -f uav_usv_lv_dot >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM
cleanup

"${DOCKER_BIN}" run --rm --name uav_usv_lv_dot --network host \
  --env ROS_MASTER_URI="${ROS_MASTER_URI}" \
  --env LV_DOT_INGRESS_PORT="${LV_DOT_INGRESS_PORT}" \
  --env LV_DOT_EGRESS_PORT="${LV_DOT_EGRESS_PORT}" \
  --volume "${LV_DOT_CONFIG}:/opt/uav_usv/lv_dot/detector_param.yaml:ro" \
  --volume "${ROOT_DIR}/containers/lv_dot_noetic/run_lv_dot.launch:/opt/uav_usv/lv_dot/run_lv_dot.launch:ro" \
  --volume "${ROOT_DIR}/containers/lv_dot_noetic/ros2_ingress.py:/opt/uav_usv/lv_dot/ros2_ingress.py:ro" \
  --volume "${ROOT_DIR}/containers/lv_dot_noetic/ros2_egress.py:/opt/uav_usv/lv_dot/ros2_egress.py:ro" \
  "${LV_DOT_IMAGE}" \
  bash -lc 'roscore & sleep 3; python3 /opt/uav_usv/lv_dot/ros2_ingress.py & python3 /opt/uav_usv/lv_dot/ros2_egress.py & exec roslaunch /opt/uav_usv/lv_dot/run_lv_dot.launch' &

wait
