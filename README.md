# Swarmap — Decentralised Robot Exploration and Mapping

[![CI](https://github.com/lakshasri/Swarmap/actions/workflows/ci.yml/badge.svg)](https://github.com/lakshasri/Swarmap/actions/workflows/ci.yml)

A decentralised swarm-robotics system where autonomous robots collaborate to
explore and map unknown environments without any central coordinator.

## What it does

Swarmap simulates a fleet of mobile robots that work together as a swarm.
Each robot explores a slice of an unknown environment, shares partial maps
only with its immediate neighbours (peer-to-peer, no master), and uses a
distributed frontier auction to decide where to go next so the swarm doesn't
duplicate effort. The system is robust to failures: when nearly half the
robots die mid-mission, the survivors keep mapping.

Visualisation comes in two flavours, both backed by the same ROS2 stack:

- **RViz2** — the native ROS2 viewer (default). One window with the merged
  occupancy grid, robot poses, LiDAR rays, frontier markers.
- **Browser dashboard** (opt-in) — React + Vite, talks to the swarm via
  rosbridge WebSocket; map canvas, stats, comm graph, mission replay.

There is no Gazebo dependency.

## Stack

- **ROS2 Humble** — middleware
- **C++17** — robot, frontier auction, map merger, aggregator (`swarmap_core`)
- **Python** — pure-ROS world simulator (`world_sim_node.py`) + swarm monitor
- **RViz2** — primary visualisation
- **React + Vite + rosbridge** — optional browser dashboard (`swarmap_dashboard`)
There is no Gazebo.

## Quick start

```bash
git clone https://github.com/lakshasri/Swarmap.git
cd Swarmap
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch swarmap_bringup simulation.launch.py num_robots:=10
```

RViz opens automatically with `swarm_debug.rviz` showing the merged map,
robot poses, LiDAR scans, and frontier markers.

### With the browser dashboard

```bash
# one-time
(cd src/swarmap_dashboard && npm install)

# then
ros2 launch swarmap_bringup simulation.launch.py num_robots:=10 dashboard:=true
```

This adds rosbridge on `ws://localhost:9090` and a Vite dev server on
`http://localhost:5173`. Open the browser to that URL — MapCanvas,
StatsPanel, ControlPanel, NetworkGraph, ReplayPanel.

## Demo scenarios

```bash
# 10 robots, no failures, ~4 min, records a rosbag in results/demo_basic/
ros2 launch swarmap_bringup demo_basic.launch.py

# 20 robots, 40% progressive failures, ~5 min, records to results/demo_fault_tolerance/
ros2 launch swarmap_bringup demo_fault_tolerance.launch.py
```

## Benchmarks

Scaling and fault-tolerance benchmarks are pre-run; results live in `results/benchmark/`:

- `coverage_vs_size_*.png` — coverage improvement as fleet size grows
- `accuracy_vs_noise_*.png` — mapping accuracy under sensor noise
- `heatmap_*.png` — spatial coverage heatmap

## Architecture in one paragraph

`world_sim_node` owns a procedurally-generated occupancy grid, listens to
each robot's `/robot_i/cmd_vel`, integrates pose, and publishes
`/robot_i/odom` + a synthetic ray-cast `/robot_i/scan`. Each
`robot_node` (lifecycle, C++) consumes its scan + odom, builds a local
log-odds grid, runs frontier detection + a peer-to-peer auction with its
neighbours, and shares partial-map deltas. `map_aggregator_node` merges
every robot's map into `/swarm/global_map` (an `OccupancyGrid`) for RViz.
`failure_injector_node` shuts down random robots to test recovery.
`swarm_monitor_node.py` tracks heartbeats and publishes `/swarm/health`.

---

**Status:** RViz + browser dashboard simulation, C++ unit tests, CI wired.
