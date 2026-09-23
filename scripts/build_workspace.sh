#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_ROOT="$(cd "$REPO_ROOT/../.." && pwd)"

JOBS="${JOBS:-4}"

echo "========================================"
echo " UAV_USV ROS1 workspace build"
echo "========================================"
echo
echo "Repository : $REPO_ROOT"
echo "Workspace  : $WS_ROOT"
echo "Jobs       : $JOBS"
echo

# ------------------------------------------------------------
# ROS Noetic
# ------------------------------------------------------------

if [ ! -f /opt/ros/noetic/setup.bash ]; then
    echo "ERROR: ROS Noetic not found."
    echo "Run:"
    echo "  bash scripts/setup_system.sh"
    exit 1
fi

set +u
source /opt/ros/noetic/setup.bash
set -u

if [ "${ROS_DISTRO:-}" != "noetic" ]; then
    echo "ERROR: expected ROS Noetic."
    exit 1
fi

# ------------------------------------------------------------
# Gazebo Harmonic environment
# ------------------------------------------------------------

GZ_WS="${GZ_HARMONIC_WS:-$HOME/gz_harmonic_ws}"

if [ -f "$GZ_WS/install/setup.bash" ]; then
    set +u
    source "$GZ_WS/install/setup.bash"
    set -u

    export GZ_CONFIG_PATH="$GZ_WS/install/gz-sim8/share/gz"
else
    echo "WARNING: Gazebo Harmonic workspace not found:"
    echo "  $GZ_WS"
    echo
    echo "If this project needs Gazebo Sim, run:"
    echo "  bash scripts/setup_gz_harmonic.sh"
fi

# ------------------------------------------------------------
# Third-party dependencies
# ------------------------------------------------------------

missing_third_party=0

for dir in \
    "$REPO_ROOT/third_party/ardupilot" \
    "$REPO_ROOT/third_party/ardupilot_gazebo" \
    "$REPO_ROOT/third_party/rapidjson"
do
    if [ ! -d "$dir" ]; then
        echo "Missing dependency: $dir"
        missing_third_party=1
    fi
done

if [ "$missing_third_party" -ne 0 ]; then
    echo
    echo "Run:"
    echo "  bash scripts/setup_third_party.sh"
    exit 1
fi

# ------------------------------------------------------------
# rosdep
# ------------------------------------------------------------

echo
echo "========================================"
echo " Resolving ROS package dependencies"
echo "========================================"

rosdep install \
    --from-paths "$REPO_ROOT/src" \
    --ignore-src \
    --rosdistro noetic \
    -r \
    -y

# ------------------------------------------------------------
# Build
# ------------------------------------------------------------

echo
echo "========================================"
echo " Building catkin workspace"
echo "========================================"

cd "$WS_ROOT"

catkin_make \
    -j"$JOBS" \
    -DCMAKE_BUILD_TYPE=Release

# ------------------------------------------------------------
# Source result
# ------------------------------------------------------------

if [ ! -f "$WS_ROOT/devel/setup.bash" ]; then
    echo
    echo "ERROR: build completed without devel/setup.bash"
    exit 1
fi

set +u
source "$WS_ROOT/devel/setup.bash"
set -u

# ------------------------------------------------------------
# Verify key ROS packages
# ------------------------------------------------------------

echo
echo "========================================"
echo " Package verification"
echo "========================================"

packages=(
    uav_usv_interfaces
    uav_usv_gazebo
    uav_usv_bringup
    uav_usv_fleet_gateway
    uav_usv_cooperative_algorithms
    uav_usv_uav_control
    uav_usv_usv_control
    uav_usv_mission
    uav_usv_perception
)

failed=0

for pkg in "${packages[@]}"; do
    if path="$(rospack find "$pkg" 2>/dev/null)"; then
        echo "[OK] $pkg -> $path"
    else
        echo "[FAIL] $pkg"
        failed=1
    fi
done

echo

if [ "$failed" -ne 0 ]; then
    echo "WARNING: workspace built, but one or more key packages were not found."
    exit 1
fi

echo "========================================"
echo " ROS1 workspace build complete"
echo "========================================"
echo
echo "For this terminal:"
echo
echo "  source $WS_ROOT/devel/setup.bash"
