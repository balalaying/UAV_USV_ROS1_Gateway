#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export GZ_FUEL_CACHE_PATH="${GZ_FUEL_CACHE_PATH:-/var/tmp/UAV_USV_gz_fuel}"
export UAV_USV_ASSET_ROOT="${UAV_USV_ASSET_ROOT:-/var/tmp/UAV_USV_assets}"
export GZ_CONFIG_PATH="${GZ_CONFIG_PATH:-}:/usr/share/gz"

GAZEBO_SHARE_DIR="$(ros2 pkg prefix --share uav_usv_gazebo)"
GAZEBO_PREFIX="$(ros2 pkg prefix uav_usv_gazebo)"
PLUGIN_DIR="${GAZEBO_PREFIX}/lib/uav_usv_gazebo/plugins"
WORLD_NAME="${1:-${UAV_USV_GZ_WORLD:-default.sdf}}"

export GZ_SIM_RESOURCE_PATH="${UAV_USV_ASSET_ROOT}:${GAZEBO_SHARE_DIR}/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PLUGIN_DIR}:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

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

read -r -a gz_args <<< "${GZ_SIM_ARGS:--r}"
gz sim "${gz_args[@]}" "${WORLD_PATH}"
