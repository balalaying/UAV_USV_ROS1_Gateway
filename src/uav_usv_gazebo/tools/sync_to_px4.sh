#!/usr/bin/env bash
set -euo pipefail

if [ -z "${PX4_DIR:-}" ]; then
  echo "PX4_DIR is not set. Please export it first, for example:" >&2
  echo "  export PX4_DIR=/your/path/PX4-Autopilot" >&2
  exit 1
fi

case "${PX4_DIR}" in
  /path|/path/*)
    echo "PX4_DIR is still a placeholder: ${PX4_DIR}" >&2
    echo "Please set PX4_DIR to your real PX4-Autopilot directory." >&2
    exit 1
    ;;
esac

if [ ! -d "${PX4_DIR}" ]; then
  echo "PX4_DIR does not exist: ${PX4_DIR}" >&2
  exit 1
fi

if [ ! -f "${PX4_DIR}/Makefile" ]; then
  echo "PX4_DIR does not look like a PX4-Autopilot source tree: ${PX4_DIR}" >&2
  echo "Missing ${PX4_DIR}/Makefile" >&2
  exit 1
fi

export GZ_FUEL_CACHE_PATH="${GZ_FUEL_CACHE_PATH:-/var/tmp/UAV_USV_gz_fuel}"
export UAV_USV_ASSET_ROOT="${UAV_USV_ASSET_ROOT:-/var/tmp/UAV_USV_assets}"

GAZEBO_SHARE_DIR="$(ros2 pkg prefix --share uav_usv_gazebo)"
GAZEBO_PREFIX="$(ros2 pkg prefix uav_usv_gazebo)"
PLUGIN_DIR="${GAZEBO_PREFIX}/lib/uav_usv_gazebo/plugins"

"${GAZEBO_PREFIX}/lib/uav_usv_gazebo/prepare_coastline.sh"

mkdir -p "${PX4_DIR}/Tools/simulation/gz/worlds"

GAZEBO_MODELS=(
  simple_boat
  waves
  medium_buoy
  target_vessel
  rock_outcrop
  shore_platform
  green_channel_buoy
  aquaculture_cage
  floating_barrel
  life_raft
  driftwood
  marina_pier
  offshore_wind_turbine
  harbor_breakwater
  harbor_tug
  fishing_boat
  person_overboard
)

for model in "${GAZEBO_MODELS[@]}"; do
  mkdir -p "${HOME}/.gz/models/${model}"
  cp -a "${GAZEBO_SHARE_DIR}/models/${model}/." "${HOME}/.gz/models/${model}/"
  sed -i \
    -e "s#filename=\"libBoatWaveFollower.so\"#filename=\"${PLUGIN_DIR}/libBoatWaveFollower.so\"#g" \
    -e "s#filename='libBoatWaveFollower.so'#filename='${PLUGIN_DIR}/libBoatWaveFollower.so'#g" \
    "${HOME}/.gz/models/${model}/model.sdf"
done

DRONE_INITIALLY_RELEASED="${UAV_USV_DRONE_INITIALLY_RELEASED:-false}"
case "${DRONE_INITIALLY_RELEASED}" in
  true|false) ;;
  *)
    echo "UAV_USV_DRONE_INITIALLY_RELEASED must be true or false" >&2
    exit 1
    ;;
esac

sed \
  -e "s#model://waves#file://${HOME}/.gz/models/waves#g" \
  -e "s#model://simple_boat#file://${HOME}/.gz/models/simple_boat#g" \
  -e "s#model://medium_buoy#file://${HOME}/.gz/models/medium_buoy#g" \
  -e "s#model://target_vessel#file://${HOME}/.gz/models/target_vessel#g" \
  -e "s#model://rock_outcrop#file://${HOME}/.gz/models/rock_outcrop#g" \
  -e "s#model://shore_platform#file://${HOME}/.gz/models/shore_platform#g" \
  -e "s#model://green_channel_buoy#file://${HOME}/.gz/models/green_channel_buoy#g" \
  -e "s#model://aquaculture_cage#file://${HOME}/.gz/models/aquaculture_cage#g" \
  -e "s#model://floating_barrel#file://${HOME}/.gz/models/floating_barrel#g" \
  -e "s#model://life_raft#file://${HOME}/.gz/models/life_raft#g" \
  -e "s#model://driftwood#file://${HOME}/.gz/models/driftwood#g" \
  -e "s#model://marina_pier#file://${HOME}/.gz/models/marina_pier#g" \
  -e "s#model://offshore_wind_turbine#file://${HOME}/.gz/models/offshore_wind_turbine#g" \
  -e "s#model://harbor_breakwater#file://${HOME}/.gz/models/harbor_breakwater#g" \
  -e "s#model://harbor_tug#file://${HOME}/.gz/models/harbor_tug#g" \
  -e "s#model://fishing_boat#file://${HOME}/.gz/models/fishing_boat#g" \
  -e "s#model://person_overboard#file://${HOME}/.gz/models/person_overboard#g" \
  -e "s#model://sydney_coast#file://${UAV_USV_ASSET_ROOT}/sydney_coast#g" \
  -e "s#<initially_released>[^<]*</initially_released>#<initially_released>${DRONE_INITIALLY_RELEASED}</initially_released>#g" \
  -e "s#filename=\"libDroneDeckFollower.so\"#filename=\"${PLUGIN_DIR}/libDroneDeckFollower.so\"#g" \
  -e "s#filename='libDroneDeckFollower.so'#filename='${PLUGIN_DIR}/libDroneDeckFollower.so'#g" \
  "${GAZEBO_SHARE_DIR}/worlds/default.sdf" > "${PX4_DIR}/Tools/simulation/gz/worlds/default.sdf"

sed -i \
  -e "s#filename=\"libBoatWaveFollower.so\"#filename=\"${PLUGIN_DIR}/libBoatWaveFollower.so\"#g" \
  -e "s#filename='libBoatWaveFollower.so'#filename='${PLUGIN_DIR}/libBoatWaveFollower.so'#g" \
  "${HOME}/.gz/models/simple_boat/model.sdf"

sed -i \
  -e "s#filename=\"libBoatWaveFollower.so\"#filename=\"${PLUGIN_DIR}/libBoatWaveFollower.so\"#g" \
  -e "s#filename='libBoatWaveFollower.so'#filename='${PLUGIN_DIR}/libBoatWaveFollower.so'#g" \
  "${HOME}/.gz/models/medium_buoy/model.sdf" \
  "${HOME}/.gz/models/target_vessel/model.sdf"

echo "Synced UAV_USV world and marine models into PX4 / Gazebo model paths."
echo "Plugin directory for standalone use: ${PLUGIN_DIR}"
