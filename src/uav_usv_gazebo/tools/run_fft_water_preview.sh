#!/usr/bin/env bash
set -euo pipefail

# Preview the copied world using the external asv_wave_sim FFT renderer.
# asv_wave_sim is GPL-3.0, so its source, binaries and model assets are kept
# outside this Apache-2.0 package.  Set ASV_WAVE_ROOT / ASV_WAVE_INSTALL when
# the external checkout or install prefix is elsewhere.

ASV_WAVE_ROOT="${ASV_WAVE_ROOT:-/tmp/asv_wave_sim}"
ASV_WAVE_INSTALL="${ASV_WAVE_INSTALL:-/tmp/asv_wave_install}"
PX4_ROOT="${PX4_DIR:-/home/dji/PX4-Autopilot}"

GAZEBO_SHARE_DIR="$(ros2 pkg prefix --share uav_usv_gazebo)"
GAZEBO_PREFIX="$(ros2 pkg prefix uav_usv_gazebo)"
SIM_PREFIX="$(ros2 pkg prefix uav_usv_sim)"
PX4_MODEL_DIR="${PX4_ROOT}/Tools/simulation/gz/models"
PX4_PLUGIN_DIR="${PX4_ROOT}/build/px4_sitl_default/src/modules/simulation/gz_plugins"
if [[ -z "${ASV_WAVE_MODEL_DIR:-}" ]]; then
  "${GAZEBO_PREFIX}/lib/uav_usv_gazebo/prepare_fft_preview_assets.sh"
  ASV_MODEL_DIR="/tmp/UAV_USV_asv_fft_models"
else
  ASV_MODEL_DIR="${ASV_WAVE_MODEL_DIR}"
fi
ASV_PLUGIN_DIR="${ASV_WAVE_INSTALL}/lib"

for required_path in \
  "${ASV_MODEL_DIR}/waves/model.sdf" \
  "${ASV_PLUGIN_DIR}/libgz-waves1-waves-model-system.so" \
  "${ASV_PLUGIN_DIR}/libgz-waves1-waves-visual-system.so" \
  "${PX4_MODEL_DIR}"; do
  if [[ ! -e "${required_path}" ]]; then
    echo "Required FFT preview path not found: ${required_path}" >&2
    echo "Build/install asv_wave_sim and set ASV_WAVE_ROOT / ASV_WAVE_INSTALL." >&2
    exit 2
  fi
done

python3 "${SIM_PREFIX}/lib/uav_usv_sim/prepare_large_x500.py" \
  --px4-dir "${PX4_ROOT}" \
  --scale "${UAV_USV_X500_SCALE:-12}" \
  --camera-width "${UAV_USV_CAMERA_WIDTH:-320}" \
  --camera-height "${UAV_USV_CAMERA_HEIGHT:-180}" \
  --camera-rate "${UAV_USV_CAMERA_RATE:-20}"

export GZ_FUEL_CACHE_PATH="${GZ_FUEL_CACHE_PATH:-/var/tmp/UAV_USV_gz_fuel}"
export UAV_USV_ASSET_ROOT="${UAV_USV_ASSET_ROOT:-/var/tmp/UAV_USV_assets}"
export GZ_CONFIG_PATH="${GZ_CONFIG_PATH:-}:/usr/share/gz"
export LD_LIBRARY_PATH="${ASV_PLUGIN_DIR}:${LD_LIBRARY_PATH:-}"
export GZ_SIM_RESOURCE_PATH="${ASV_MODEL_DIR}:${PX4_MODEL_DIR}:${GAZEBO_SHARE_DIR}/models:${UAV_USV_ASSET_ROOT}:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ASV_PLUGIN_DIR}:${GAZEBO_PREFIX}/lib/uav_usv_gazebo/plugins:${PX4_PLUGIN_DIR}:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

WORLD_PATH="${GAZEBO_SHARE_DIR}/worlds/heterogeneous_332_fft_water.sdf"
read -r -a gz_args <<< "${GZ_SIM_ARGS:--r}"
exec gz sim "${gz_args[@]}" "${WORLD_PATH}" "$@"
