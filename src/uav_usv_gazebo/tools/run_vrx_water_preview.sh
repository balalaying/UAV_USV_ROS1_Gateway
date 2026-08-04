#!/usr/bin/env bash
set -euo pipefail

# Standalone preview for the copied VRX-water world.  This script deliberately
# points Gazebo at heterogeneous_332_vrx_water.sdf; the original
# heterogeneous_332.sdf is never changed or selected here.

PX4_ROOT="${PX4_DIR:-/home/dji/PX4-Autopilot}"
GAZEBO_SHARE_DIR="$(ros2 pkg prefix --share uav_usv_gazebo)"
GAZEBO_PREFIX="$(ros2 pkg prefix uav_usv_gazebo)"
SIM_PREFIX="$(ros2 pkg prefix uav_usv_sim)"
PX4_MODEL_DIR="${PX4_ROOT}/Tools/simulation/gz/models"
PX4_PLUGIN_DIR="${PX4_ROOT}/build/px4_sitl_default/src/modules/simulation/gz_plugins"

if [[ ! -d "${PX4_MODEL_DIR}" ]]; then
  echo "PX4 model directory not found: ${PX4_MODEL_DIR}" >&2
  echo "Set PX4_DIR to the root of a PX4-Autopilot checkout." >&2
  exit 2
fi

python3 "${SIM_PREFIX}/lib/uav_usv_sim/prepare_large_x500.py" \
  --px4-dir "${PX4_ROOT}" \
  --scale "${UAV_USV_X500_SCALE:-12}" \
  --camera-width "${UAV_USV_CAMERA_WIDTH:-320}" \
  --camera-height "${UAV_USV_CAMERA_HEIGHT:-180}" \
  --camera-rate "${UAV_USV_CAMERA_RATE:-20}"

export GZ_FUEL_CACHE_PATH="${GZ_FUEL_CACHE_PATH:-/var/tmp/UAV_USV_gz_fuel}"
export UAV_USV_ASSET_ROOT="${UAV_USV_ASSET_ROOT:-/var/tmp/UAV_USV_assets}"
export GZ_CONFIG_PATH="${GZ_CONFIG_PATH:-}:/usr/share/gz"
export GZ_SIM_RESOURCE_PATH="${PX4_MODEL_DIR}:${GAZEBO_SHARE_DIR}/models:${UAV_USV_ASSET_ROOT}:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GAZEBO_PREFIX}/lib/uav_usv_gazebo/plugins:${PX4_PLUGIN_DIR}:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

WORLD_PATH="${GAZEBO_SHARE_DIR}/worlds/heterogeneous_332_vrx_water.sdf"
read -r -a gz_args <<< "${GZ_SIM_ARGS:--r}"
exec gz sim "${gz_args[@]}" "${WORLD_PATH}" "$@"
