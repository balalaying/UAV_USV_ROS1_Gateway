#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export GZ_FUEL_CACHE_PATH="${GZ_FUEL_CACHE_PATH:-/var/tmp/UAV_USV_gz_fuel}"
export UAV_USV_ASSET_ROOT="${UAV_USV_ASSET_ROOT:-/var/tmp/UAV_USV_assets}"
GZ_INSTALL_ROOT="${GZ_INSTALL_ROOT:-$HOME/gz_harmonic_ws/install}"
GZ_SOURCE_CONFIG_PATH="$(find "${GZ_INSTALL_ROOT}" -type d -path "*/share/gz" -printf "%p:" 2>/dev/null | sed "s/:$//")"
export GZ_CONFIG_PATH="${GZ_SOURCE_CONFIG_PATH}:/usr/share/gz:${GZ_CONFIG_PATH:-}"

GAZEBO_SHARE_DIR="$(rospack find uav_usv_gazebo)"
WORKSPACE_ROOT="$(cd "${GAZEBO_SHARE_DIR}/../../../.." && pwd)"
REPO_ROOT="$(cd "${GAZEBO_SHARE_DIR}/../.." && pwd)"
GUI_CONFIG="${UAV_USV_GZ_GUI_CONFIG:-${REPO_ROOT}/src/uav_usv_bringup/config/gazebo_white_gui.config}"
PLUGIN_DIR="${WORKSPACE_ROOT}/devel/lib"
WORLD_NAME="${1:-${UAV_USV_GZ_WORLD:-heterogeneous_332.sdf}}"

export GZ_SIM_RESOURCE_PATH="${GAZEBO_SHARE_DIR}/models:${UAV_USV_ASSET_ROOT}:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PLUGIN_DIR}:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_GUI_PLUGIN_PATH="$HOME/gz_harmonic_ws/install/gz-sim8/lib/gz-sim-8/plugins/gui:$HOME/gz_harmonic_ws/install/gz-gui8/lib/gz-gui-8/plugins:${GZ_GUI_PLUGIN_PATH:-}"

if [[ "${WORLD_NAME}" != */* ]]; then
  [[ "${WORLD_NAME}" == *.sdf ]] || WORLD_NAME="${WORLD_NAME}.sdf"
  WORLD_PATH="${GAZEBO_SHARE_DIR}/worlds/${WORLD_NAME}"
else
  WORLD_PATH="${WORLD_NAME}"
fi

case "$(basename "${WORLD_PATH}")" in
  default.sdf|vrx_sydney_regatta_custom.sdf)
    "${SCRIPT_DIR}/prepare_coastline.sh"
    ;;
esac

if [[ "$(basename "${WORLD_PATH}")" == "heterogeneous_332.sdf" ]]; then
  "${SCRIPT_DIR}/verify_repo_models.py" \
    --world "${WORLD_PATH}" \
    --model-root "${GAZEBO_SHARE_DIR}/models"
fi

read -r -a gz_args <<< "${GZ_SIM_ARGS:--r}"
[[ -f "${GUI_CONFIG}" ]] || { echo "Gazebo GUI config not found: ${GUI_CONFIG}" >&2; exit 1; }
gz sim "${gz_args[@]}" --gui-config "${GUI_CONFIG}" "${WORLD_PATH}"
