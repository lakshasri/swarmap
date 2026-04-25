# Swarmap — Decentralised Robot Exploration and Mapping

A browser-based dashboard and simulation framework for decentralized multi-robot exploration and collaborative mapping of unknown environments. Built on **ROS2 Humble** for real-time robot communication and **MATLAB** for algorithmic simulation and analysis.

---

## Overview

Swarmap demonstrates emergent swarm intelligence: a fleet of autonomous ground robots collectively maps an unknown environment without any central coordinator. Each robot only shares map data with its immediate neighbours, navigates toward unexplored areas on its own, and the global map assembles itself from hundreds of small local exchanges.

The system is fault-tolerant by design — even if nearly half the robots fail mid-mission, the remaining swarm reorganises and continues mapping.

---

## Core Features

### Decentralised Map Sharing

-   Robots broadcast partial occupancy grid maps only to neighbours within a configurable communication radius
-   No central map server; the global map emerges from peer-to-peer gossip propagation
-   Map merging uses weighted confidence scoring to resolve conflicts between overlapping scans

### Autonomous Frontier Exploration

-   Each robot maintains a local frontier list (boundaries between explored and unknown space)
-   A distributed frontier auction prevents multiple robots from targeting the same unexplored cell
-   Path planning uses RRT* with obstacle inflation, replanning on sensor updates

### Live Browser Dashboard

-   Real-time WebSocket feed from ROS2 bridge to a browser frontend
-   Animated occupancy grid showing explored (white), unexplored (grey), obstacle (black), and robot-position (coloured) cells
-   Live stats panel:
    -   Percentage of area mapped
    -   Number of active vs. failed robots
    -   Map accuracy score (compared against ground truth in simulation mode)
    -   Per-robot trajectory trails
    -   Network topology graph showing current communication links

### User-Adjustable Parameters (Dashboard Controls)

Parameter

Range

Effect

Swarm size

2 – 30 robots

Spawns or removes robots at runtime

Sensor range

1 – 15 m

LiDAR scan radius per robot

Noise level

0 – 50%

Gaussian noise injected into sensor readings

Failure rate

0 – 45%

Probability a robot permanently fails each minute

Comm radius

0.5 – 10 m

Maximum distance for map-sharing handshakes

Map resolution

0.05 – 0.5 m/cell

Occupancy grid cell size

### Fault Tolerance Proof

-   Robots are randomly killed at the configured failure rate during a mission
-   Dashboard logs and replays each failure event
-   A post-run report shows mission completion percentage as a function of robot loss

---

## Tech Stack

Layer

Technology

Robot middleware

ROS2 Humble (Ubuntu 22.04)

Algorithm simulation

MATLAB R2023b + Robotics System Toolbox

MATLAB–ROS2 bridge

`ros2` MATLAB interface (built-in from R2021b+)

Frontend dashboard

React + Vite, Canvas API for map rendering

Backend bridge

`roslibjs` + `rosbridge_suite` WebSocket server

Map format

ROS2 `nav_msgs/OccupancyGrid`

Robot description

URDF diff-drive model, Gazebo Harmonic simulation

Inter-robot comms

ROS2 DDS (FastDDS) with custom QoS profiles

Visualisation

RViz2 (debug) + custom browser dashboard (primary)

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐│                    Browser Dashboard                     ││  React UI ←─── WebSocket ─── rosbridge_server (ROS2)   │└─────────────────────────────────────────────────────────┘                              │              ┌───────────────▼───────────────┐              │         ROS2 DDS Layer         │              │  /robot_N/map  /robot_N/pose   │              │  /robot_N/frontier  /swarm/... │              └───────┬───────────────┬────────┘                      │               │            ┌─────────▼──┐     ┌──────▼──────┐            │  Robot 1   │     │   Robot N   │            │ ┌────────┐ │ ... │ ┌─────────┐ │            │ │Explore │ │     │ │ Explore │ │            │ │Planner │ │     │ │ Planner │ │            │ └────────┘ │     │ └─────────┘ │            │ ┌────────┐ │     │ ┌─────────┐ │            │ │Map     │ │◄───►│ │  Map    │ │            │ │Merger  │ │     │ │ Merger  │ │            │ └────────┘ │     │ └─────────┘ │            └────────────┘     └─────────────┘                      │               │            ┌─────────▼───────────────▼────────┐            │       Gazebo Harmonic / MATLAB    │            │    Physics + Ground Truth Map     │            └───────────────────────────────────┘
```

---

## MATLAB Simulation

The MATLAB component provides two roles:

**1. Algorithm Prototyping (offline)**

-   Implement and tune frontier exploration, map merging, and auction algorithms in MATLAB before deploying to ROS2
-   Use `robotics.OccupancyGrid` and `nav.PRM` / `nav.RRTStar` from Robotics System Toolbox
-   Run Monte Carlo experiments over noise level and failure rate with `parfor`

**2. Online Co-simulation (with ROS2)**

-   MATLAB connects to a running ROS2 network via the built-in `ros2` interface
-   Publishes synthetic sensor data to inject into the live swarm
-   Subscribes to `/swarm/global_map` and plots accuracy curves against the known ground truth in real time
-   Exports mission reports as PDF / CSV

### Running the MATLAB simulation

```matlab
% Connect to ROS2ros2 node create "matlab_bridge"% Load environmentenv = load('maps/warehouse_env.mat');sim = SwarmSimulator(env, numRobots=10, sensorRange=5);% Run with 30% failure ratesim.run(failureRate=0.30, duration=300);  % 5 min missionsim.plotResults();sim.exportReport('results/run_001.pdf');
```

---

## ROS2 Package Layout

```
swarmap/├── swarmap_core/            # Core robot node (C++17)│   ├── src/│   │   ├── robot_node.cpp           # Main robot lifecycle node│   │   ├── frontier_explorer.cpp    # Frontier detection + auction│   │   ├── map_merger.cpp           # Neighbour map merge logic│   │   └── failure_injector.cpp     # Simulated fault injection│   └── include/swarmap_core/├── swarmap_msgs/            # Custom ROS2 message definitions│   └── msg/│       ├── PartialMap.msg           # Compressed occupancy grid chunk│       ├── FrontierBid.msg          # Auction bid message│       └── RobotStatus.msg          # Health + pose broadcast├── swarmap_bringup/         # Launch files│   └── launch/│       ├── simulation.launch.py     # Gazebo + N robots│       └── dashboard.launch.py      # rosbridge + React dev server├── swarmap_dashboard/       # Browser frontend (React/Vite)│   ├── src/│   │   ├── components/│   │   │   ├── MapCanvas.tsx        # Live occupancy grid renderer│   │   │   ├── StatsPanel.tsx       # Mapped %, active robots, accuracy│   │   │   ├── ControlPanel.tsx     # Sliders and buttons│   │   │   └── NetworkGraph.tsx     # Robot comm topology│   │   └── hooks/│   │       └── useRosBridge.ts      # WebSocket + ROSLIB hook└── matlab/    ├── SwarmSimulator.m    ├── FrontierExplorer.m    ├── MapMerger.m    └── maps/
```

---

## Getting Started

### Prerequisites

-   Ubuntu 22.04
-   ROS2 Humble (`ros-humble-desktop`, `ros-humble-rosbridge-server`, `ros-humble-nav2-*`)
-   Gazebo Harmonic
-   Node.js 20+
-   MATLAB R2023b with Robotics System Toolbox (for MATLAB features)

### Quick Start

```bash
# Clonegit clone https://github.com/lakshasri/Swarmap.git && cd Swarmap# Build ROS2 packagessource /opt/ros/humble/setup.bashcolcon build --symlink-installsource install/setup.bash# Launch simulation (10 robots, warehouse map)ros2 launch swarmap_bringup simulation.launch.py num_robots:=10 map:=warehouse# In a second terminal — start dashboardros2 launch swarmap_bringup dashboard.launch.py# Open http://localhost:5173 in browser
```

---

## Suggested Feature Changes

### Features to Add

#

Feature

Reason

1

**3D voxel mapping**

Extend from 2D occupancy grids to 3D voxel maps using `octomap_server2`; enables staircase / multi-floor environments

2

**Heterogeneous robot types**

Mix ground robots (diff-drive) with aerial drones (quadrotors) for faster frontier coverage of high shelves or elevated terrain

3

**Dynamic obstacle tracking**

Add moving obstacle detection and sharing between robots so the merged map stays current in changing environments

4

**Energy-aware planning**

Model robot battery; frontier auction bids factor in remaining energy so robots near depletion return to dock instead of exploring

5

**SLAM per robot** (GMapping / Cartographer)

Replace simple odometry + LiDAR with proper per-robot SLAM so pose estimates stay accurate over longer missions

6

**Replay and export**

Record a full mission and replay it at variable speed in the dashboard; export the final merged map as PGM/YAML for use in Nav2

7

**MATLAB Automated Test Suite**

Parameterised MATLAB scripts that benchmark exploration efficiency vs. swarm size, generating publishable result plots

### Features to Remove or Simplify

#

Feature

Reason

1

**Central map server (if any)**

Contradicts the core decentralisation goal; remove any `/global_map` publisher that aggregates on behalf of all robots — let the browser aggregate for display only

2

**Per-robot RViz2 windows**

Clutters the workflow; the browser dashboard replaces RViz2 as the primary visualisation; keep RViz2 only as an optional debug tool via a launch argument

3

**Hard-coded environment maps**

Replace fixed maps with a procedural map generator (Perlin noise rooms + corridors) so every simulation run is unique and parameter sweeps are more meaningful

4

**Synchronous map merging**

Remove any blocking merge calls; merging must be fully asynchronous to avoid one slow robot stalling the whole swarm

---

## Fault Tolerance Demo

Run the pre-built scenario that kills robots progressively:

```bash
ros2 launch swarmap_bringup simulation.launch.py   num_robots:=20   failure_rate:=0.40   failure_mode:=progressive   map:=large_office
```

The dashboard will show robots going dark one by one while the remaining swarm converges on unvisited frontiers. The mission completes with map coverage above 85% even with 8 of 20 robots lost.

---

## Outcomes

-   A working browser-based dashboard where users watch the swarm build a map in real time
-   Demonstrated fault tolerance: mission succeeds with up to 45% robot loss
-   A global occupancy map that emerges purely from local peer-to-peer information sharing
-   MATLAB reports quantifying exploration efficiency, map accuracy, and swarm resilience across parameter sweeps

---

## License

MIT