#!/usr/bin/env bash
#
# SWARMAP — one-command launcher
#
# Usage:
#   ./run.sh                 # 8 robots, dashboard only (recommended)
#   ./run.sh 12              # 12 robots
#   ./run.sh 8 --rviz        # 8 robots + RViz
#   ./run.sh --clean         # nuke stale processes first, then 8 robots
#   ./run.sh --rebuild       # force rebuild before launch
#   ./run.sh --help          # show options
#
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NUM_ROBOTS=8
USE_RVIZ="false"
DO_CLEAN=false
DO_REBUILD=false

# ── Parse args ────────────────────────────────────────────────────
for arg in "$@"; do
    case "$arg" in
        --help|-h)
            sed -n '/^# Usage:/,/^#$/p' "$0" | sed 's/^# //; s/^#//'
            exit 0
            ;;
        --rviz)    USE_RVIZ="true" ;;
        --clean)   DO_CLEAN=true ;;
        --rebuild) DO_REBUILD=true ;;
        [0-9]*)    NUM_ROBOTS="$arg" ;;
    esac
done

# ── Colors ────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; CYAN='\033[0;36m'
BOLD='\033[1m'; NC='\033[0m'
info(){ echo -e "${CYAN}[swarmap]${NC} $*"; }
ok(){   echo -e "${GREEN}[swarmap]${NC} $*"; }
fail(){ echo -e "${RED}[swarmap]${NC} $*"; exit 1; }

# ── 1. Source ROS 2 ───────────────────────────────────────────────
[[ -f /opt/ros/humble/setup.bash ]] || fail "ROS2 Humble not found at /opt/ros/humble/setup.bash"
source /opt/ros/humble/setup.bash

# ── 2. Kill stale processes ───────────────────────────────────────
cleanup() {
    info "Killing stale swarmap processes..."
    pkill -9 -f "robot_node" 2>/dev/null || true
    pkill -9 -f "world_sim_node" 2>/dev/null || true
    pkill -9 -f "map_aggregator" 2>/dev/null || true
    pkill -9 -f "swarm_monitor" 2>/dev/null || true
    pkill -9 -f "failure_injector" 2>/dev/null || true
    pkill -9 -f "dashboard_bridge" 2>/dev/null || true
    pkill -9 -f "rosbridge_websocket" 2>/dev/null || true
    pkill -9 -f "launch swarmap" 2>/dev/null || true
    pkill -9 -f "vite" 2>/dev/null || true
    sleep 1
    fuser -k 5173/tcp 2>/dev/null || true
    fuser -k 9090/tcp 2>/dev/null || true
    sleep 2
}

if $DO_CLEAN; then
    cleanup
else
    # Quick port check only
    if ss -ltn 2>/dev/null | grep -qE ":5173|:9090"; then
        info "Ports 5173/9090 already in use — cleaning up..."
        cleanup
    fi
fi

# ── 3. npm deps ───────────────────────────────────────────────────
DASHBOARD_DIR="$SCRIPT_DIR/src/swarmap_dashboard"
if [[ ! -d "$DASHBOARD_DIR/node_modules" ]]; then
    info "Installing dashboard npm dependencies..."
    (cd "$DASHBOARD_DIR" && npm install --silent)
fi

# ── 4. Build (if needed or forced) ────────────────────────────────
cd "$SCRIPT_DIR"
NEEDS_BUILD=$DO_REBUILD
if ! $DO_REBUILD; then
    if [[ ! -f install/swarmap_core/lib/swarmap_core/robot_node ]]; then
        NEEDS_BUILD=true
    fi
fi

if $NEEDS_BUILD; then
    info "Building workspace (colcon)..."
    colcon build --symlink-install 2>&1 | tail -3
    ok "Build complete."
fi

source install/setup.bash

# ── 5. Banner ─────────────────────────────────────────────────────
cat <<BANNER
${BOLD}
╔═══════════════════════════════════════════════╗
║                S W A R M A P                  ║
║   Decentralised swarm exploration system      ║
╚═══════════════════════════════════════════════╝
${NC}
  Robots:     ${BOLD}${NUM_ROBOTS}${NC}
  Dashboard:  ${BOLD}http://localhost:5173${NC}
  Rosbridge:  ws://localhost:9090
  RViz:       ${USE_RVIZ}

  Controls available in the left panel:
    ▸ PAUSE / RESUME / STOP / RESET MAP
    ▸ + SPAWN ROBOT / KILL SELECTED
    ▸ Three map layers: MERGED / GROUND TRUTH / CONFIDENCE

${CYAN}Launching... (Ctrl+C to stop)${NC}

BANNER

# ── 6. Launch ─────────────────────────────────────────────────────
exec ros2 launch swarmap_bringup simulation.launch.py \
    num_robots:="$NUM_ROBOTS" \
    rviz:="$USE_RVIZ" \
    dashboard:=true
