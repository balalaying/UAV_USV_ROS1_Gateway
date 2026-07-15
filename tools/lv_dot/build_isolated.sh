#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DOCKER_BIN="${DOCKER_BIN:-docker}"
IMAGE_NAME="${LV_DOT_IMAGE:-uav-usv/lv-dot-noetic:449bf2c}"

"${DOCKER_BIN}" build \
  --tag "${IMAGE_NAME}" \
  "${ROOT_DIR}/containers/lv_dot_noetic"

"${DOCKER_BIN}" run --rm "${IMAGE_NAME}" \
  bash -lc 'rospack find onboard_detector && catkin locate --workspace /opt/lv_dot_ws'
