#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_PREFIX="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SOURCE_PREFIX="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [ -d "${INSTALL_PREFIX}/share/uav_usv_sim" ]; then
  SHARE_DIR="${INSTALL_PREFIX}/share/uav_usv_sim"
else
  SHARE_DIR="${SOURCE_PREFIX}"
fi

FUEL_CACHE="${GZ_FUEL_CACHE_PATH:-/var/tmp/UAV_USV_gz_fuel}"
ASSET_ROOT="${UAV_USV_ASSET_ROOT:-/var/tmp/UAV_USV_assets}"
MODEL_ROOT="${FUEL_CACHE}/fuel.gazebosim.org/openrobotics/models/sydney_regatta"
TARGET_DIR="${ASSET_ROOT}/sydney_coast"
FUEL_URL="https://fuel.gazebosim.org/1.0/openrobotics/models/sydney_regatta"

if [ "${UAV_USV_REFRESH_COASTLINE:-0}" = "1" ] || \
   ! find "${MODEL_ROOT}" -mindepth 3 -maxdepth 3 \
     -path '*/meshes/sydney_regatta.dae' -print -quit 2>/dev/null | grep -q .; then
  mkdir -p "${FUEL_CACHE}"
  GZ_FUEL_CACHE_PATH="${FUEL_CACHE}" gz fuel download -u "${FUEL_URL}" -v 1
fi

SOURCE_DIR="$(
  find "${MODEL_ROOT}" -mindepth 1 -maxdepth 1 -type d -printf '%p\n' \
    | sort -V | tail -n 1
)"

if [ -z "${SOURCE_DIR}" ] || [ ! -f "${SOURCE_DIR}/meshes/sydney_regatta.dae" ]; then
  echo "Sydney Regatta asset is incomplete under ${MODEL_ROOT}" >&2
  exit 1
fi

mkdir -p "${TARGET_DIR}"
if [ -L "${TARGET_DIR}/meshes" ]; then
  rm "${TARGET_DIR}/meshes"
fi
mkdir -p "${TARGET_DIR}/meshes"
ln -sfn "${SOURCE_DIR}/materials" "${TARGET_DIR}/materials"
cp "${SHARE_DIR}/config/sydney_coast.model.sdf" "${TARGET_DIR}/model.sdf"
cp "${SHARE_DIR}/config/sydney_coast.model.config" "${TARGET_DIR}/model.config"

BUILD_VERSION="dual_exit_v1"
if [ "${UAV_USV_REFRESH_COASTLINE:-0}" = "1" ] || \
   [ ! -f "${TARGET_DIR}/.${BUILD_VERSION}" ] || \
   [ ! -f "${TARGET_DIR}/meshes/sydney_regatta.dae" ] || \
   [ ! -f "${TARGET_DIR}/meshes/sydney_regatta_shore.dae" ]; then
  rm -f "${TARGET_DIR}"/.dual_exit_v*
  python3 "${SCRIPT_DIR}/build_coastline_assets.py" \
    --source-dir "${SOURCE_DIR}" \
    --target-dir "${TARGET_DIR}"
  touch "${TARGET_DIR}/.${BUILD_VERSION}"
fi

echo "Sydney coastline prepared at ${TARGET_DIR}"
