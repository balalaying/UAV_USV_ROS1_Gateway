#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GAZEBO_SHARE_DIR="$(rospack find uav_usv_gazebo)"
REPO_ROOT="$(cd "${GAZEBO_SHARE_DIR}/../.." && pwd)"
SOURCE_WORLD="${GAZEBO_SHARE_DIR}/worlds/heterogeneous_332.sdf"
MODEL_ROOT="${GAZEBO_SHARE_DIR}/models"

RGL_ROOT="${UAV_USV_RGL_ROOT:-/var/tmp/RGLGazeboPlugin_v0.2.0_focal_custom}"
RGL_PLUGIN_DIR="${RGL_ROOT}/install/RGLServerPlugin"
RGL_PATTERNS_DIR_VALUE="${RGL_ROOT}/lidar_patterns"
RUNTIME_ROOT="${UAV_USV_MID360_RUNTIME_ROOT:-/var/tmp/UAV_USV_fleet_mid360}"
UPDATE_RATE="${UAV_USV_MID360_RATE:-10.0}"
MIN_RANGE="${UAV_USV_MID360_MIN_RANGE:-0.5}"
MAX_RANGE="${UAV_USV_MID360_MAX_RANGE:-70.0}"

required=(
  "${RGL_PLUGIN_DIR}/libRGLServerPluginInstance.so"
  "${RGL_PLUGIN_DIR}/libRGLServerPluginManager.so"
  "${RGL_PLUGIN_DIR}/libRobotecGPULidar.so"
  "${RGL_PATTERNS_DIR_VALUE}/LivoxMid360.mat3x4f"
)
for path in "${required[@]}"; do
  if [[ ! -f "${path}" ]]; then
    echo "错误：Mid-360 依赖不存在：${path}" >&2
    exit 20
  fi
done

"${SCRIPT_DIR}/verify_repo_models.py" \
  --world "${SOURCE_WORLD}" \
  --model-root "${MODEL_ROOT}"

mkdir -p "${RUNTIME_ROOT}/models" "${RUNTIME_ROOT}/worlds"
GENERATED_WORLD="$(python3 "${SCRIPT_DIR}/prepare_fleet_mid360.py" \
  --world "${SOURCE_WORLD}" \
  --models-dir "${MODEL_ROOT}" \
  --output-root "${RUNTIME_ROOT}" \
  --vehicle-ids usv_01,usv_02,usv_03 \
  --link-name hull \
  --mount-pose '0.9075 0 1.5625 0 0 0' \
  --raw-topic '/fleet/uplink/{vehicle_id}/mid360/rgl_points' \
  --frame-id '{vehicle_id}/mid360_link' \
  --update-rate "${UPDATE_RATE}" \
  --min-range "${MIN_RANGE}" \
  --max-range "${MAX_RANGE}" \
  --model-scale 1.0 \
  --visual-scale 1.0 \
  --enable-mid360)"
# run_gz_world.sh verifies an input named heterogeneous_332.sdf against the
# unmodified repository model URIs.  That verification was already performed
# above; give the generated copy a distinct filename so its intentional
# *_mid360_runtime URIs are not mistaken for stale repository models.
RUNTIME_WORLD="${RUNTIME_ROOT}/worlds/heterogeneous_332_mid360.sdf"
cp -f -- "${GENERATED_WORLD}" "${RUNTIME_WORLD}"

export GZ_SIM_RESOURCE_PATH="${RUNTIME_ROOT}/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${RGL_PLUGIN_DIR}:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="${RGL_PLUGIN_DIR}:${LD_LIBRARY_PATH:-}"
export RGL_PATTERNS_DIR="${RGL_PATTERNS_DIR_VALUE}"

echo "Mid-360 runtime world: ${RUNTIME_WORLD}"
echo "Mid-360 RGL root: ${RGL_ROOT}"
echo "Mid-360: 3 vessels, ${UPDATE_RATE} Hz, ${MIN_RANGE}-${MAX_RANGE} m"
exec "${SCRIPT_DIR}/run_gz_world.sh" "${RUNTIME_WORLD}"
