#!/usr/bin/env bash
set -euo pipefail

echo "========================================"
echo " UAV_USV system environment setup"
echo "========================================"

# ------------------------------------------------------------
# 1. Platform check
# ------------------------------------------------------------

if [ ! -f /etc/os-release ]; then
    echo "ERROR: /etc/os-release not found."
    exit 1
fi

. /etc/os-release

echo
echo "Detected OS:"
echo "  ${PRETTY_NAME:-unknown}"

if [ "${ID:-}" != "ubuntu" ]; then
    echo "ERROR: this setup targets Ubuntu."
    exit 1
fi

if [ "${VERSION_ID:-}" != "20.04" ]; then
    echo
    echo "WARNING:"
    echo "  Recorded development environment is Ubuntu 20.04."
    echo "  Current system is ${VERSION_ID:-unknown}."
    echo
    echo "  Continuing may produce a different environment."
fi

# ------------------------------------------------------------
# 2. Base development packages
# ------------------------------------------------------------

echo
echo "========================================"
echo " Installing base development tools"
echo "========================================"

sudo apt-get update

sudo apt-get install -y \
    build-essential \
    gcc \
    g++ \
    make \
    pkg-config \
    git \
    curl \
    wget \
    gnupg \
    lsb-release \
    ca-certificates \
    software-properties-common \
    python3 \
    python3-dev \
    python3-pip \
    python3-venv \
    python3-setuptools \
    python3-yaml

# ------------------------------------------------------------
# 3. ROS Noetic repository
# ------------------------------------------------------------

echo
echo "========================================"
echo " Configuring ROS Noetic repository"
echo "========================================"

sudo mkdir -p /usr/share/keyrings

if [ ! -f /usr/share/keyrings/ros-archive-keyring.asc ]; then
    curl -fsSL \
      https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
      | sudo tee /usr/share/keyrings/ros-archive-keyring.asc \
      >/dev/null
fi

echo \
"deb [signed-by=/usr/share/keyrings/ros-archive-keyring.asc] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
| sudo tee /etc/apt/sources.list.d/ros1.list \
>/dev/null

sudo apt-get update

# ------------------------------------------------------------
# 4. ROS Noetic
# ------------------------------------------------------------

echo
echo "========================================"
echo " Installing ROS Noetic"
echo "========================================"

if [ ! -f /opt/ros/noetic/setup.bash ]; then

    sudo apt-get install -y \
        ros-noetic-desktop-full

else
    echo "ROS Noetic already installed."
fi

sudo apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-catkin-tools

# ------------------------------------------------------------
# 5. rosdep
# ------------------------------------------------------------

echo
echo "========================================"
echo " Configuring rosdep"
echo "========================================"

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then

    sudo rosdep init || true

fi

rosdep update || {
    echo
    echo "WARNING: rosdep update failed."
    echo "Network access may be unavailable."
}

# ------------------------------------------------------------
# 6. Gazebo Classic 11
# ------------------------------------------------------------

echo
echo "========================================"
echo " Gazebo Classic"
echo "========================================"

if command -v gazebo >/dev/null 2>&1; then

    echo "Gazebo Classic already installed:"
    gazebo --version | head -1 || true

else

    sudo apt-get install -y gazebo11 libgazebo11-dev

fi

# ------------------------------------------------------------
# 7. OSRF repository
# ------------------------------------------------------------

echo
echo "========================================"
echo " Configuring OSRF Gazebo repository"
echo "========================================"

sudo mkdir -p /usr/share/keyrings

if [ ! -f /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]; then

    sudo curl -fsSL \
      https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

fi

echo \
"deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/gazebo-stable.list \
>/dev/null

sudo apt-get update

# IMPORTANT:
# Do NOT install gz-harmonic here.
#
# The recorded machine runs Ubuntu 20.04 with a source-built
# Gazebo Harmonic workspace at ~/gz_harmonic_ws.
#
# scripts/setup_gz_harmonic.sh recreates that environment.

# ------------------------------------------------------------
# 8. CMake 3.27.9
# ------------------------------------------------------------

echo
echo "========================================"
echo " Installing recorded CMake version"
echo "========================================"

python3 -m pip install --user "cmake==3.27.9"

export PATH="$HOME/.local/bin:$PATH"

echo
echo "CMake executable:"
command -v cmake

echo "CMake version:"
cmake --version | head -1

# ------------------------------------------------------------
# 9. Shell ROS environment
# ------------------------------------------------------------

BASHRC="$HOME/.bashrc"

MARK_BEGIN="# >>> UAV_USV ROS1 environment >>>"
MARK_END="# <<< UAV_USV ROS1 environment <<<"

if ! grep -Fq "$MARK_BEGIN" "$BASHRC" 2>/dev/null; then

cat >> "$BASHRC" <<'BASHRC_EOF'

# >>> UAV_USV ROS1 environment >>>
export PATH="$HOME/.local/bin:$PATH"
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
fi
# <<< UAV_USV ROS1 environment <<<
BASHRC_EOF

    echo
    echo "ROS environment added to ~/.bashrc."

else

    echo
    echo "ROS environment block already exists in ~/.bashrc."

fi

# ------------------------------------------------------------
# 10. Verification
# ------------------------------------------------------------

echo
echo "========================================"
echo " Verification"
echo "========================================"

set +u
source /opt/ros/noetic/setup.bash
set -u

echo
echo "[ROS]"
echo "ROS_DISTRO=${ROS_DISTRO:-unknown}"
rosversion -d || true

echo
echo "[catkin_make]"
command -v catkin_make || true

echo
echo "[colcon]"
command -v colcon || true

echo
echo "[vcs]"
command -v vcs || true
vcs --version || true

echo
echo "[Gazebo Classic]"
gazebo --version | head -1 || true

echo
echo "[CMake]"
cmake --version | head -1 || true

echo
echo "========================================"
echo " System setup complete"
echo "========================================"
echo
echo "Next steps:"
echo "  bash scripts/setup_gz_harmonic.sh"
echo "  bash scripts/setup_third_party.sh"
echo
