#!/usr/bin/env bash
set -u

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_ROOT="$(cd "$REPO_ROOT/../.." && pwd)"
GZ_WS="${GZ_HARMONIC_WS:-$HOME/gz_harmonic_ws}"

FAILURES=0
WARNINGS=0

ok()
{
    echo "[OK]   $*"
}

warn()
{
    echo "[WARN] $*"
    WARNINGS=$((WARNINGS + 1))
}

fail()
{
    echo "[FAIL] $*"
    FAILURES=$((FAILURES + 1))
}

echo "========================================"
echo " UAV_USV environment check"
echo "========================================"
echo

# ------------------------------------------------------------
# OS
# ------------------------------------------------------------

if [ -f /etc/os-release ]; then
    . /etc/os-release

    if [ "${ID:-}" = "ubuntu" ] && [ "${VERSION_ID:-}" = "20.04" ]; then
        ok "Ubuntu 20.04"
    else
        warn "Expected Ubuntu 20.04, found ${PRETTY_NAME:-unknown}"
    fi
else
    fail "/etc/os-release not found"
fi

# ------------------------------------------------------------
# ROS
# ------------------------------------------------------------

if [ -f /opt/ros/noetic/setup.bash ]; then
    set +u
    source /opt/ros/noetic/setup.bash
    set -u

    if [ "${ROS_DISTRO:-}" = "noetic" ]; then
        ok "ROS Noetic"
    else
        fail "ROS distro is ${ROS_DISTRO:-unknown}"
    fi
else
    fail "ROS Noetic not installed"
fi

# ------------------------------------------------------------
# Tools
# ------------------------------------------------------------

for cmd in git python3 colcon vcs catkin_make rosdep; do
    if command -v "$cmd" >/dev/null 2>&1; then
        ok "$cmd -> $(command -v "$cmd")"
    else
        fail "$cmd not found"
    fi
done

# ------------------------------------------------------------
# CMake
# ------------------------------------------------------------

if command -v cmake >/dev/null 2>&1; then
    CMAKE_VERSION="$(cmake --version | head -1 | awk '{print $3}')"

    if [ "$CMAKE_VERSION" = "3.27.9" ]; then
        ok "CMake 3.27.9"
    else
        warn "CMake version is $CMAKE_VERSION; recorded version is 3.27.9"
    fi
else
    fail "cmake not found"
fi

# ------------------------------------------------------------
# Gazebo Classic
# ------------------------------------------------------------

if command -v gazebo >/dev/null 2>&1; then
    GAZEBO_VERSION="$(gazebo --version 2>/dev/null | head -1)"

    if echo "$GAZEBO_VERSION" | grep -q '11\.'; then
        ok "$GAZEBO_VERSION"
    else
        warn "$GAZEBO_VERSION"
    fi
else
    fail "Gazebo Classic not found"
fi

# ------------------------------------------------------------
# Gazebo Harmonic source workspace
# ------------------------------------------------------------

if [ -f "$GZ_WS/install/setup.bash" ]; then

    set +u
    source "$GZ_WS/install/setup.bash"
    set -u

    export GZ_CONFIG_PATH="$GZ_WS/install/gz-sim8/share/gz"

    ok "Gazebo Harmonic setup.bash"

    if [ -f "$GZ_WS/install/.colcon_install_layout" ]; then
        layout="$(cat "$GZ_WS/install/.colcon_install_layout")"

        if [ "$layout" = "isolated" ]; then
            ok "Gazebo Harmonic colcon layout: isolated"
        else
            warn "Gazebo Harmonic layout: $layout"
        fi
    else
        warn "Gazebo Harmonic install layout marker missing"
    fi

    if command -v gz >/dev/null 2>&1; then
        GZ_VERSION="$(gz sim --version 2>/dev/null | head -1)"

        if echo "$GZ_VERSION" | grep -q 'version 8\.'; then
            ok "$GZ_VERSION"
        else
            warn "Unexpected Gazebo Sim version: $GZ_VERSION"
        fi
    else
        fail "gz command unavailable after sourcing Gazebo workspace"
    fi

else
    fail "Gazebo Harmonic workspace missing: $GZ_WS"
fi

# ------------------------------------------------------------
# Third-party revisions
# ------------------------------------------------------------

check_git_revision()
{
    local name="$1"
    local path="$2"
    local expected="$3"

    if [ ! -d "$path" ]; then
        fail "$name missing: $path"
        return
    fi

    if ! git -C "$path" rev-parse HEAD >/dev/null 2>&1; then
        fail "$name is not a Git repository"
        return
    fi

    local actual
    actual="$(git -C "$path" rev-parse HEAD)"

    if [ "$actual" = "$expected" ]; then
        ok "$name revision $actual"
    else
        fail "$name revision mismatch"
        echo "       expected: $expected"
        echo "       actual:   $actual"
    fi
}

echo
echo "----- Third-party repositories -----"

check_git_revision \
    "ArduPilot" \
    "$REPO_ROOT/third_party/ardupilot" \
    "2a3dc4b7bf2507120f7378a7b2fde73185e0c325"

check_git_revision \
    "ardupilot_gazebo" \
    "$REPO_ROOT/third_party/ardupilot_gazebo" \
    "082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5"

check_git_revision \
    "RapidJSON" \
    "$REPO_ROOT/third_party/rapidjson" \
    "f54b0e47a08782a6131cc3d60f94d038fa6e0a51"

check_git_revision \
    "ArduPilot littlefs" \
    "$REPO_ROOT/third_party/ardupilot/modules/littlefs" \
    "34be692aafaaf4319a18e415b08aa4332697408b"

# ------------------------------------------------------------
# ardupilot_gazebo local patch
# ------------------------------------------------------------

PATCH="$REPO_ROOT/third_party_patches/ardupilot_gazebo_ArduPilotPlugin.patch"
AP_GZ="$REPO_ROOT/third_party/ardupilot_gazebo"

if [ -f "$PATCH" ] && [ -d "$AP_GZ/.git" ]; then

    if git -C "$AP_GZ" apply \
        --reverse \
        --check \
        "$PATCH" >/dev/null 2>&1
    then
        ok "ArduPilotPlugin local patch is applied"
    else
        fail "ArduPilotPlugin local patch is not applied correctly"
    fi

else
    fail "ArduPilot Gazebo patch or repository missing"
fi

# ------------------------------------------------------------
# ROS workspace
# ------------------------------------------------------------

echo
echo "----- ROS1 workspace -----"

if [ -f "$WS_ROOT/devel/setup.bash" ]; then

    set +u
    source "$WS_ROOT/devel/setup.bash"
    set -u

    ok "catkin devel environment exists"

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

    for pkg in "${packages[@]}"; do
        if rospack find "$pkg" >/dev/null 2>&1; then
            ok "ROS package: $pkg"
        else
            fail "ROS package not found: $pkg"
        fi
    done

else
    warn "Workspace has not been built yet: $WS_ROOT/devel/setup.bash"
fi

# ------------------------------------------------------------
# Summary
# ------------------------------------------------------------

echo
echo "========================================"
echo " Environment check summary"
echo "========================================"
echo
echo "Failures : $FAILURES"
echo "Warnings : $WARNINGS"
echo

if [ "$FAILURES" -eq 0 ]; then
    echo "Environment is ready."
    exit 0
else
    echo "Environment requires attention."
    exit 1
fi
