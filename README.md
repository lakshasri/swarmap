# Swarmap - Decentralised Robot Exploration and Mapping

[![CI](https://github.com/lakshasri/Swarmap/actions/workflows/ci.yml/badge.svg)](https://github.com/lakshasri/Swarmap/actions/workflows/ci.yml)

A decentralised swarm robotics system where autonomous robots collaborate to explore and map unknown environments without any central coordinator.

## What is Swarmap?

Swarmap enables a fleet of mobile robots to work together as a swarm, each exploring different parts of an unknown environment simultaneously. The robots share partial maps only with their immediate neighbours, never requiring a central server or master controller. Through distributed frontier auctions, each robot autonomously decides where to explore next, avoiding wasteful duplication of effort.

The system is designed to be **robust to failures** — even when nearly half the robots fail mid-mission, the remaining swarm adapts and continues mapping. The live browser dashboard lets you watch the map emerge in real-time and adjust swarm parameters dynamically.

## Why It Matters

- **Decentralised**: No single point of failure. Communication is peer-to-peer.
- **Autonomous**: Robots make local decisions without a master control loop.
- **Scalable**: Add more robots and coverage increases proportionally (until overcrowded).
- **Fault-tolerant**: Surviving robots restructure and keep working when neighbours fail.
- **Real-time visibility**: Browser dashboard shows live exploration progress and network topology.

## Core Technologies

- **ROS2 Humble** — inter-robot communication and middleware
- **Gazebo Harmonic** — physics simulation and sensor simulation
- **React + Canvas** — browser-based live dashboard
- **MATLAB** — algorithm prototyping and benchmark analysis
- **C++17** — high-performance robot nodes

## Quick Start

```bash
git clone https://github.com/lakshasri/Swarmap.git
cd Swarmap
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch swarmap_bringup simulation.launch.py num_robots:=10
```

Open http://localhost:5173 in your browser to see the dashboard.

## Demo Scenarios

```bash
# Scenario 1 — 10 robots, warehouse, no failures, 4 min
ros2 launch swarmap_bringup demo_basic.launch.py

# Scenario 2 — 20 robots, large office, 40% progressive failures, 5 min
ros2 launch swarmap_bringup demo_fault_tolerance.launch.py

# Batch driver that records both into results/
scripts/run_demos.sh
```

## Benchmarks

```bash
# Headless throughput / scalability sweep (ROS2 only)
scripts/benchmarks/perf_scalability.sh
scripts/benchmarks/fault_tolerance_validation.py --runs 10

# MATLAB parameter sweep and PDF report
matlab -batch "addpath(genpath('matlab/src')); RunBenchmarks('results/benchmark')"
```

See [results/README.md](results/README.md) for the output layout and
[matlab/README.md](matlab/README.md) for MATLAB quick-start.

---

**Status**: Feature-complete — demo launches, MATLAB sweeps, launch-based
integration tests, CI, and release workflow all wired up.
