#!/usr/bin/env bash
# bootstrap.sh — one-shot workspace setup on a fresh Ubuntu 22.04 + ROS2 Humble machine
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=== Swarmap bootstrap ==="
echo "Workspace: $WS_DIR"

if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
else
    echo "ERROR: ROS2 Humble not found at /opt/ros/humble" >&2
    exit 1
fi

if ! command -v rosdep &>/dev/null; then
    sudo apt-get install -y python3-rosdep
    sudo rosdep init || true
fi
rosdep update

cd "$WS_DIR"
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

echo ""
echo "=== Build complete ==="
echo "Run:  source $WS_DIR/install/setup.bash"
echo "Then: ros2 launch swarmap_bringup simulation.launch.py num_robots:=10"
