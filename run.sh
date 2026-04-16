#!/usr/bin/env bash
#
# SWARMAP — one-command launcher
#
# Usage:
#   ./run.sh              # 8 robots, dashboard, no RViz
#   ./run.sh 12           # 12 robots
#   ./run.sh 5 --rviz     # 5 robots + RViz
#
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NUM_ROBOTS="${1:-8}"
USE_RVIZ="false"
for arg in "$@"; do [[ "$arg" == "--rviz" ]] && USE_RVIZ="true"; done

RED='\033[0;31m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

info()  { echo -e "${CYAN}[swarmap]${NC} $*"; }
ok()    { echo -e "${GREEN}[swarmap]${NC} $*"; }
fail()  { echo -e "${RED}[swarmap]${NC} $*"; exit 1; }

# ── 1. Source ROS2 ──────────────────────────────────────────────
info "Sourcing ROS2 Humble..."
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
else
    fail "ROS2 Humble not found at /opt/ros/humble/setup.bash"
fi

# ── 2. Install npm deps if needed ───────────────────────────────
DASHBOARD_DIR="$SCRIPT_DIR/src/swarmap_dashboard"
if [[ ! -d "$DASHBOARD_DIR/node_modules" ]]; then
    info "Installing dashboard npm dependencies..."
    (cd "$DASHBOARD_DIR" && npm install --silent)
fi

# ── 3. Build workspace ──────────────────────────────────────────
info "Building workspace (colcon)..."
cd "$SCRIPT_DIR"
colcon build --symlink-install 2>&1 | tail -5
source install/setup.bash
ok "Build complete."

# ── 4. Kill stale processes on our ports ────────────────────────
for port in 5173 9090; do
    fuser -k "$port/tcp" 2>/dev/null || true
done
sleep 1

# ── 5. Launch ───────────────────────────────────────────────────
echo ""
echo -e "${BOLD}╔══════════════════════════════════════════╗${NC}"
echo -e "${BOLD}║           S W A R M A P                  ║${NC}"
echo -e "${BOLD}║  Decentralised swarm exploration system  ║${NC}"
echo -e "${BOLD}╚══════════════════════════════════════════╝${NC}"
echo ""
info "Robots:    ${NUM_ROBOTS}"
info "Dashboard: http://localhost:5173"
info "Rosbridge: ws://localhost:9090"
info "RViz:      ${USE_RVIZ}"
echo ""
ok "Launching... (Ctrl+C to stop)"
echo ""

ros2 launch swarmap_bringup simulation.launch.py \
    num_robots:="$NUM_ROBOTS" \
    rviz:="$USE_RVIZ" \
    dashboard:=true
