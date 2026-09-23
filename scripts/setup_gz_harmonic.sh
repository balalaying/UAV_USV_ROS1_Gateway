#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

WS="${GZ_HARMONIC_WS:-$HOME/gz_harmonic_ws}"
SRC="$WS/src"

MANIFEST="$PROJECT_ROOT/docs/environment/gz_harmonic_source.yaml"

CMAKE_VERSION="3.27.9"

echo "========================================"
echo " Gazebo Harmonic source environment"
echo "========================================"
echo
echo "Workspace: $WS"
echo

# ------------------------------------------------------------
# Basic platform check
# ------------------------------------------------------------

if [ -f /etc/os-release ]; then
    . /etc/os-release

    if [ "${VERSION_ID:-}" != "20.04" ]; then
        echo "WARNING:"
        echo "  Recorded development environment is Ubuntu 20.04."
        echo "  Current system is ${VERSION_ID:-unknown}."
        echo
    fi
fi

# ------------------------------------------------------------
# Required commands
# ------------------------------------------------------------

for cmd in git python3 vcs colcon; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
        echo "ERROR: required command not found: $cmd"
        exit 1
    fi
done

# ------------------------------------------------------------
# CMake 3.27.9
# ------------------------------------------------------------

export PATH="$HOME/.local/bin:$PATH"

CURRENT_CMAKE=""
if command -v cmake >/dev/null 2>&1; then
    CURRENT_CMAKE="$(cmake --version | head -1 | awk '{print $3}')"
fi

if [ "$CURRENT_CMAKE" != "$CMAKE_VERSION" ]; then
    echo
    echo "Installing user-level CMake $CMAKE_VERSION ..."
    python3 -m pip install --user "cmake==$CMAKE_VERSION"
fi

export PATH="$HOME/.local/bin:$PATH"

echo
echo "CMake:"
which cmake
cmake --version | head -1

# ------------------------------------------------------------
# Workspace
# ------------------------------------------------------------

mkdir -p "$SRC"

if [ ! -f "$MANIFEST" ]; then
    echo "ERROR: manifest not found:"
    echo "  $MANIFEST"
    exit 1
fi

# Clone repositories if the workspace has no Gazebo sources yet.
if [ ! -d "$SRC/gz-sim/.git" ]; then
    echo
    echo "Importing Gazebo Harmonic repositories ..."
    vcs import "$SRC" < "$MANIFEST"
else
    echo
    echo "Gazebo source tree already exists."
fi

# ------------------------------------------------------------
# Exact revisions recorded from the working machine
# ------------------------------------------------------------

checkout_revision()
{
    local repo="$1"
    local commit="$2"
    local dir="$SRC/$repo"

    if [ ! -d "$dir/.git" ]; then
        echo "ERROR: repository missing: $dir"
        exit 1
    fi

    if [ -n "$(git -C "$dir" status --porcelain)" ]; then
        echo "ERROR: repository has local modifications:"
        echo "  $dir"
        echo
        echo "Refusing to overwrite local work."
        exit 1
    fi

    echo
    echo "[$repo]"
    echo "checkout $commit"

    git -C "$dir" fetch origin
    git -C "$dir" checkout "$commit"
}

checkout_revision gz-cmake \
  eb85f5f20d9358c65a0ec1afa0c391b3254a8e26

checkout_revision gz-common \
  dcf9693fea6977cabc7bd408c94251a59019db24

checkout_revision gz-fuel-tools \
  c381002f4e0004fd9c186592836d8a9c936ab281

checkout_revision gz-gui \
  e17f4fda7697c08c84fae6f106ee6997a7b6196a

checkout_revision gz-launch \
  f97cf936af57eb75d477487b35e4bfdbc670d330

checkout_revision gz-math \
  c2f878f355d9c9fd5050ea36a09c9ea6e723d4e7

checkout_revision gz-msgs \
  292056c60b08a83b5d005e244dc3aa2c9a32a061

checkout_revision gz-physics \
  275641c6c830c75ed0c3a602ed440f42b8ef8b13

checkout_revision gz-plugin \
  25b10154cbe7a23e0bbf939fcc7cd63be39f70da

checkout_revision gz-rendering \
  f4805cc67d815b1f80805d120e2ceb54c78b4cf5

checkout_revision gz-sensors \
  504241fff44bb2bb5e86b323aac95493550bfa7d

checkout_revision gz-sim \
  446a44335a45b704b4d36dabcc5508ee34eeb3d8

checkout_revision gz-tools \
  e1ea54ce558127465e8d313084bb719eaccdf33e

checkout_revision gz-transport \
  e57e131e38b198c81391f72e297a8437ccdb1c23

checkout_revision gz-utils \
  c23010fad4a291dbd4f06023000da071469f1424

checkout_revision sdformat \
  8512585c6265faf4aed0069a6765f11bd15b7efc

# ------------------------------------------------------------
# Build
# ------------------------------------------------------------

cd "$WS"

if [ -f "$WS/install/setup.bash" ]; then
    # Existing partial / complete installation.
    set +u
    source "$WS/install/setup.bash"
    set -u
fi

echo
echo "========================================"
echo " Building Gazebo Harmonic"
echo "========================================"
echo

colcon build \
    --continue-on-error \
    --cmake-args \
    -DBUILD_TESTING=OFF

# ------------------------------------------------------------
# Verify
# ------------------------------------------------------------

if [ ! -f "$WS/install/setup.bash" ]; then
    echo
    echo "ERROR: build finished without install/setup.bash"
    exit 1
fi

set +u
source "$WS/install/setup.bash"
set -u

export GZ_CONFIG_PATH="$WS/install/gz-sim8/share/gz"

echo
echo "========================================"
echo " Verification"
echo "========================================"

echo
echo "gz executable:"
command -v gz || true

echo
echo "Gazebo Sim version:"
gz sim --version || true

echo
echo "Install layout:"
cat "$WS/install/.colcon_install_layout" 2>/dev/null || true

echo
echo "Expected: isolated"

# ------------------------------------------------------------
# Shell environment
# ------------------------------------------------------------

BASHRC="$HOME/.bashrc"

MARK_BEGIN="# >>> UAV_USV Gazebo Harmonic >>>"
MARK_END="# <<< UAV_USV Gazebo Harmonic <<<"

if ! grep -Fq "$MARK_BEGIN" "$BASHRC" 2>/dev/null; then

    cat >> "$BASHRC" <<BASHRC_EOF

$MARK_BEGIN
export PATH="\$HOME/.local/bin:\$PATH"
export GZ_CONFIG_PATH="\$HOME/gz_harmonic_ws/install/gz-sim8/share/gz"
if [ -f "\$HOME/gz_harmonic_ws/install/setup.bash" ]; then
    source "\$HOME/gz_harmonic_ws/install/setup.bash"
fi
$MARK_END
BASHRC_EOF

    echo
    echo "Added Gazebo Harmonic environment to ~/.bashrc"
else
    echo
    echo "Gazebo Harmonic ~/.bashrc block already exists."
fi

echo
echo "========================================"
echo " Gazebo Harmonic setup complete"
echo "========================================"
echo
echo "Open a new terminal or run:"
echo
echo "  source ~/.bashrc"
echo
