#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
THIRD_PARTY="${ROOT}/third_party"
PATCH_DIR="${ROOT}/third_party_patches"

ARDUPILOT_COMMIT="2a3dc4b7bf2507120f7378a7b2fde73185e0c325"
ARDUPILOT_GAZEBO_COMMIT="082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5"
RAPIDJSON_COMMIT="f54b0e47a08782a6131cc3d60f94d038fa6e0a51"
LITTLEFS_COMMIT="34be692aafaaf4319a18e415b08aa4332697408b"

mkdir -p "${THIRD_PARTY}"

echo "========== ArduPilot =========="

if [ ! -d "${THIRD_PARTY}/ardupilot/.git" ]; then
    rm -rf "${THIRD_PARTY}/ardupilot"
    git clone https://github.com/ArduPilot/ardupilot.git \
        "${THIRD_PARTY}/ardupilot"
fi

git -C "${THIRD_PARTY}/ardupilot" fetch --all --tags
git -C "${THIRD_PARTY}/ardupilot" checkout "${ARDUPILOT_COMMIT}"
git -C "${THIRD_PARTY}/ardupilot" submodule update --init --recursive

touch "${THIRD_PARTY}/ardupilot/CATKIN_IGNORE"

echo "========== ArduPilot littlefs =========="

if [ ! -d "${THIRD_PARTY}/ardupilot/modules/littlefs/.git" ] && \
   [ ! -f "${THIRD_PARTY}/ardupilot/modules/littlefs/.git" ]; then
    rm -rf "${THIRD_PARTY}/ardupilot/modules/littlefs"
    git clone https://github.com/ArduPilot/littlefs.git \
        "${THIRD_PARTY}/ardupilot/modules/littlefs"
fi

git -C "${THIRD_PARTY}/ardupilot/modules/littlefs" fetch --all --tags
git -C "${THIRD_PARTY}/ardupilot/modules/littlefs" checkout "${LITTLEFS_COMMIT}"

echo "========== ardupilot_gazebo =========="

if [ ! -d "${THIRD_PARTY}/ardupilot_gazebo/.git" ]; then
    rm -rf "${THIRD_PARTY}/ardupilot_gazebo"
    git clone https://github.com/ArduPilot/ardupilot_gazebo.git \
        "${THIRD_PARTY}/ardupilot_gazebo"
fi

git -C "${THIRD_PARTY}/ardupilot_gazebo" fetch --all --tags
git -C "${THIRD_PARTY}/ardupilot_gazebo" checkout "${ARDUPILOT_GAZEBO_COMMIT}"

PATCH="${PATCH_DIR}/ardupilot_gazebo_ArduPilotPlugin.patch"

if git -C "${THIRD_PARTY}/ardupilot_gazebo" apply \
    --reverse --check "${PATCH}" >/dev/null 2>&1; then
    echo "ArduPilotPlugin patch already applied."
elif git -C "${THIRD_PARTY}/ardupilot_gazebo" apply \
    --check "${PATCH}" >/dev/null 2>&1; then
    git -C "${THIRD_PARTY}/ardupilot_gazebo" apply "${PATCH}"
    echo "ArduPilotPlugin patch applied."
else
    echo "ERROR: ArduPilotPlugin patch cannot be applied."
    exit 1
fi

echo "========== RapidJSON =========="

if [ ! -d "${THIRD_PARTY}/rapidjson/.git" ]; then
    rm -rf "${THIRD_PARTY}/rapidjson"
    git clone https://github.com/Tencent/rapidjson.git \
        "${THIRD_PARTY}/rapidjson"
fi

git -C "${THIRD_PARTY}/rapidjson" fetch --all --tags
git -C "${THIRD_PARTY}/rapidjson" checkout "${RAPIDJSON_COMMIT}"

touch "${THIRD_PARTY}/rapidjson/CATKIN_IGNORE"

echo
echo "Third-party dependency restore complete."
echo
echo "ArduPilot:"
git -C "${THIRD_PARTY}/ardupilot" rev-parse HEAD

echo "littlefs:"
git -C "${THIRD_PARTY}/ardupilot/modules/littlefs" rev-parse HEAD

echo "ardupilot_gazebo:"
git -C "${THIRD_PARTY}/ardupilot_gazebo" rev-parse HEAD

echo "RapidJSON:"
git -C "${THIRD_PARTY}/rapidjson" rev-parse HEAD
