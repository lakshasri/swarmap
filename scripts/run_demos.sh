#!/usr/bin/env bash
# End-to-end driver that runs the three headline demo scenarios and
# collates their outputs under results/.
set -euo pipefail
cd "$(dirname "$0")/.."

source install/setup.bash

run_demo () {
    local name=$1
    local launch=$2
    local duration=$3
    local out="results/${name}"
    mkdir -p "$out"
    echo "==> demo: $name (duration=${duration}s)"
    SWARMAP_DEMO_DURATION=$duration ros2 launch swarmap_bringup "$launch" &
    local pid=$!
    # Let the auto-stop timer fire + 30 s grace for bag flush.
    sleep "$((duration + 30))"
    kill -INT $pid 2>/dev/null || true
    wait $pid 2>/dev/null || true
    echo "==> $name complete -> $out"
}

run_demo demo_basic            demo_basic.launch.py            240
run_demo demo_fault_tolerance  demo_fault_tolerance.launch.py  300

echo "All demos complete.  Artifacts under results/"
