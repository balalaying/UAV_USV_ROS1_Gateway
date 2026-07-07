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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_PREFIX="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SOURCE_PREFIX="$(cd "${SCRIPT_DIR}/.." && pwd)"
export GZ_FUEL_CACHE_PATH="${GZ_FUEL_CACHE_PATH:-/var/tmp/UAV_USV_gz_fuel}"
export UAV_USV_ASSET_ROOT="${UAV_USV_ASSET_ROOT:-/var/tmp/UAV_USV_assets}"

python3 "${SCRIPT_DIR}/prepare_large_x500.py" \
  --px4-dir "${PX4_DIR}" \
  --scale "${UAV_USV_X500_SCALE:-3.5}" \
  --camera-width "${UAV_USV_CAMERA_WIDTH:-640}" \
  --camera-height "${UAV_USV_CAMERA_HEIGHT:-480}" \
  --camera-rate "${UAV_USV_CAMERA_RATE:-20}"

if [ -d "${INSTALL_PREFIX}/share/uav_usv_sim" ]; then
  SHARE_DIR="${INSTALL_PREFIX}/share/uav_usv_sim"
  PLUGIN_DIR="${INSTALL_PREFIX}/lib/uav_usv_sim/plugins"
else
  SHARE_DIR="${SOURCE_PREFIX}"
  PLUGIN_DIR="${PX4_DIR}/build/uav_usv_plugins"
  mkdir -p "${PLUGIN_DIR}"
  cp "${PX4_DIR}/build/drone_deck_follower/libDroneDeckFollower.so" "${PLUGIN_DIR}/" 2>/dev/null || true
  cp "${PX4_DIR}/build/boat_wave_follower/libBoatWaveFollower.so" "${PLUGIN_DIR}/" 2>/dev/null || true
fi

"${SCRIPT_DIR}/prepare_coastline.sh"

mkdir -p "${PX4_DIR}/Tools/simulation/gz/worlds"
mkdir -p "${HOME}/.gz/models/simple_boat"
mkdir -p "${HOME}/.gz/models/waves"
mkdir -p "${HOME}/.gz/models/medium_buoy"
mkdir -p "${HOME}/.gz/models/shore_helipad"
mkdir -p "${HOME}/.gz/models/fleet_boat_blue"
mkdir -p "${HOME}/.gz/models/fleet_boat_orange"
mkdir -p "${HOME}/.gz/models/fleet_uav_blue"
mkdir -p "${HOME}/.gz/models/fleet_uav_orange"

cp "${SHARE_DIR}/models/simple_boat/model.sdf" "${HOME}/.gz/models/simple_boat/model.sdf"
cp "${SHARE_DIR}/models/simple_boat/model.config" "${HOME}/.gz/models/simple_boat/model.config"
cp -a "${SHARE_DIR}/models/waves/." "${HOME}/.gz/models/waves/"
cp "${SHARE_DIR}/models/medium_buoy/model.sdf" "${HOME}/.gz/models/medium_buoy/model.sdf"
cp "${SHARE_DIR}/models/medium_buoy/model.config" "${HOME}/.gz/models/medium_buoy/model.config"
cp "${SHARE_DIR}/models/shore_helipad/model.sdf" "${HOME}/.gz/models/shore_helipad/model.sdf"
cp "${SHARE_DIR}/models/shore_helipad/model.config" "${HOME}/.gz/models/shore_helipad/model.config"
cp "${SHARE_DIR}/models/fleet_boat_blue/model.sdf" "${HOME}/.gz/models/fleet_boat_blue/model.sdf"
cp "${SHARE_DIR}/models/fleet_boat_blue/model.config" "${HOME}/.gz/models/fleet_boat_blue/model.config"
cp "${SHARE_DIR}/models/fleet_boat_orange/model.sdf" "${HOME}/.gz/models/fleet_boat_orange/model.sdf"
cp "${SHARE_DIR}/models/fleet_boat_orange/model.config" "${HOME}/.gz/models/fleet_boat_orange/model.config"
cp "${SHARE_DIR}/models/fleet_uav_blue/model.sdf" "${HOME}/.gz/models/fleet_uav_blue/model.sdf"
cp "${SHARE_DIR}/models/fleet_uav_blue/model.config" "${HOME}/.gz/models/fleet_uav_blue/model.config"
cp "${SHARE_DIR}/models/fleet_uav_orange/model.sdf" "${HOME}/.gz/models/fleet_uav_orange/model.sdf"
cp "${SHARE_DIR}/models/fleet_uav_orange/model.config" "${HOME}/.gz/models/fleet_uav_orange/model.config"

sed \
  -e "s#model://waves#file://${HOME}/.gz/models/waves#g" \
  -e "s#model://simple_boat#file://${HOME}/.gz/models/simple_boat#g" \
  -e "s#model://medium_buoy#file://${HOME}/.gz/models/medium_buoy#g" \
  -e "s#model://shore_helipad#file://${HOME}/.gz/models/shore_helipad#g" \
  -e "s#model://fleet_boat_blue#file://${HOME}/.gz/models/fleet_boat_blue#g" \
  -e "s#model://fleet_boat_orange#file://${HOME}/.gz/models/fleet_boat_orange#g" \
  -e "s#model://fleet_uav_blue#file://${HOME}/.gz/models/fleet_uav_blue#g" \
  -e "s#model://fleet_uav_orange#file://${HOME}/.gz/models/fleet_uav_orange#g" \
  -e "s#model://sydney_coast#file://${UAV_USV_ASSET_ROOT}/sydney_coast#g" \
  -e "s#filename=\"libDroneDeckFollower.so\"#filename=\"${PLUGIN_DIR}/libDroneDeckFollower.so\"#g" \
  -e "s#filename='libDroneDeckFollower.so'#filename='${PLUGIN_DIR}/libDroneDeckFollower.so'#g" \
  "${SHARE_DIR}/worlds/default.sdf" > "${PX4_DIR}/Tools/simulation/gz/worlds/default.sdf"

sed -i \
  -e "s#filename=\"libBoatWaveFollower.so\"#filename=\"${PLUGIN_DIR}/libBoatWaveFollower.so\"#g" \
  -e "s#filename='libBoatWaveFollower.so'#filename='${PLUGIN_DIR}/libBoatWaveFollower.so'#g" \
  "${HOME}/.gz/models/simple_boat/model.sdf"
sed -i \
  -e "s#filename=\"libBoatWaveFollower.so\"#filename=\"${PLUGIN_DIR}/libBoatWaveFollower.so\"#g" \
  -e "s#filename='libBoatWaveFollower.so'#filename='${PLUGIN_DIR}/libBoatWaveFollower.so'#g" \
  "${HOME}/.gz/models/fleet_boat_blue/model.sdf"
sed -i \
  -e "s#filename=\"libBoatWaveFollower.so\"#filename=\"${PLUGIN_DIR}/libBoatWaveFollower.so\"#g" \
  -e "s#filename='libBoatWaveFollower.so'#filename='${PLUGIN_DIR}/libBoatWaveFollower.so'#g" \
  "${HOME}/.gz/models/fleet_boat_orange/model.sdf"

echo "Synced UAV_USV world and simple_boat model into PX4 / Gazebo model paths."
echo "PX4 x500 scale: ${UAV_USV_X500_SCALE:-3.5}x"
echo "Plugin directory for standalone use: ${PLUGIN_DIR}"
