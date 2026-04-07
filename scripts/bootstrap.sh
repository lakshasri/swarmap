#!/usr/bin/env bash
# bootstrap.sh — one-shot workspace setup on a fresh Ubuntu 22.04 + ROS2 Humble machine
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=== Swarmap bootstrap ==="
echo "Workspace: $WS_DIR"

# Source ROS2
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
else
    echo "ERROR: ROS2 Humble not found at /opt/ros/humble" >&2
    exit 1
fi

# Install rosdep if missing
if ! command -v rosdep &>/dev/null; then
    sudo apt-get install -y python3-rosdep
    sudo rosdep init || true
fi
rosdep update

# Install package dependencies
cd "$WS_DIR"
rosdep install --from-paths src --ignore-src -r -y

# Install Node.js 20 if missing (for dashboard)
if ! command -v node &>/dev/null || [[ "$(node -e 'process.stdout.write(process.version.slice(1).split(".")[0])')" -lt 20 ]]; then
    curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
    sudo apt-get install -y nodejs
fi

# Install dashboard npm dependencies
DASHBOARD_DIR="$WS_DIR/src/swarmap_dashboard"
if [[ -f "$DASHBOARD_DIR/package.json" ]]; then
    echo "Installing npm packages..."
    npm --prefix "$DASHBOARD_DIR" install
fi

# Build workspace
colcon build --symlink-install

echo ""
echo "=== Build complete ==="
echo "Run:  source $WS_DIR/install/setup.bash"
echo "Then: ros2 launch swarmap_bringup simulation.launch.py"
