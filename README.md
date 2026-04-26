# Swarmap

[![CI](https://github.com/lakshasri/Swarmap/actions/workflows/ci.yml/badge.svg)](https://github.com/lakshasri/Swarmap/actions/workflows/ci.yml)

Decentralised swarm robotics system for autonomous exploration and mapping. A fleet of mobile robots collaboratively explores an unknown environment with no central coordinator — each robot makes local decisions, shares partial maps only with immediate neighbours, and uses a distributed frontier auction to avoid duplicating effort. The swarm keeps functioning even when nearly half the robots fail mid-mission.

---

## How it works

Each robot:

1. Receives a ray-cast LiDAR scan and odometry from `world_sim_node`
2. Builds a local log-odds occupancy grid from its scan
3. Discovers neighbours within `comm_radius` and exchanges compressed map deltas
4. Runs frontier detection and submits bids for unclaimed frontiers (lower bid score = higher priority, weighted by energy cost-to-go and cluster size)
5. Navigates to its claimed frontier while monitoring battery level
6. Returns to dock when `battery × safety_factor < time_to_empty`

`map_aggregator_node` merges every robot's local map into a single `/swarm/global_map` published for RViz and the browser dashboard. `swarm_monitor_node` tracks heartbeats and publishes `/swarm/health`.

---

## Architecture

```
world_sim_node (Python)
  ├── publishes /robot_i/odom, /robot_i/scan
  └── subscribes to /robot_i/cmd_vel

robot_node × N (C++17, lifecycle)
  ├── subscribes: /robot_i/odom, /robot_i/scan, /swarm/partial_map
  ├── publishes:  /robot_i/cmd_vel, /swarm/robot_status, /swarm/partial_map
  │              /swarm/frontier_bid, /swarm/neighbour_discovery
  └── lifecycle states: unconfigured → inactive → active → failed

map_aggregator_node (C++)
  ├── subscribes: /swarm/partial_map (all robots)
  └── publishes:  /swarm/global_map (OccupancyGrid)

swarm_monitor_node (Python)
  ├── subscribes: /swarm/robot_status
  └── publishes:  /swarm/health

dashboard_bridge_node (Python)
  └── bridges /swarm/* topics to rosbridge WebSocket (port 9090)
```

There is no Gazebo. The world simulator is a pure-ROS2 Python node.

---

## Stack

| Component | Technology |
|-----------|-----------|
| Middleware | ROS2 Humble |
| Robot & mapping logic | C++17 (`swarmap_core`) |
| World simulator, monitor | Python 3 |
| Primary visualisation | RViz2 |
| Browser dashboard | React + TypeScript + Vite |
| ROS ↔ browser bridge | rosbridge + roslibjs |
| Build system | colcon + ament_cmake |

---

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble ([install guide](https://docs.ros.org/en/humble/Installation.html))
- Node.js 20+ and npm
- rosdep

```bash
sudo rosdep init   # skip if already done
rosdep update
```

---

## Quick start

```bash
git clone https://github.com/lakshasri/Swarmap.git
cd Swarmap

# install ROS dependencies
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# build
colcon build --symlink-install
source install/setup.bash

# launch (8 robots + browser dashboard)
./run.sh
```

Open **http://localhost:5173** in your browser.

### run.sh flags

```
./run.sh              # 8 robots, dashboard on (default)
./run.sh 12           # 12 robots
./run.sh 8 --rviz     # add RViz2 window
./run.sh --clean      # kill stale processes first, then launch
./run.sh --rebuild    # force colcon rebuild before launch
./run.sh --help       # show usage
```

---

## Manual launch

For more control, use the ROS2 launch files directly.

### Simulation only (RViz)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch swarmap_bringup simulation.launch.py \
    num_robots:=10 \
    rviz:=true \
    dashboard:=false
```

### Simulation + browser dashboard

```bash
# one-time: install npm dependencies
(cd src/swarmap_dashboard && npm install)

ros2 launch swarmap_bringup simulation.launch.py \
    num_robots:=10 \
    dashboard:=true
```

### Dashboard only (simulation already running)

```bash
ros2 launch swarmap_bringup dashboard.launch.py
```

---

## Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `num_robots` | `5` | Number of robots to spawn |
| `sensor_range` | `5.0` | LiDAR max range (metres) |
| `comm_radius` | `8.0` | Peer-to-peer communication radius (metres) |
| `map_resolution` | `0.1` | Occupancy grid cell size (metres) |
| `rviz` | `true` | Open RViz2 with `swarm_debug.rviz` |
| `dashboard` | `false` | Start rosbridge + Vite dev server |

---

## Configuration

All tunable parameters live in `src/swarmap_bringup/config/default_params.yaml`. Key values:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_width_m` / `map_height_m` | `50.0` | World size in metres |
| `noise_level` | `0.0` | Gaussian noise on LiDAR ranges (fraction) |
| `goal_tolerance` | `0.3` | Arrival threshold in metres |
| `update_rate_hz` | `10.0` | Exploration + control tick rate |
| `frontier_min_size` | `5.0` | Minimum frontier cluster size (cells) |
| `battery_drain_rate` | `0.001` | Fraction drained per second (~16 min to empty) |
| `battery_weight` | `2.0` | How aggressively low-battery robots avoid far frontiers |
| `dock_return_safety` | `0.85` | Return if `time_to_dock > 0.85 × time_to_empty` |

---

## Key topics

| Topic | Type | Publisher |
|-------|------|-----------|
| `/robot_i/scan` | `sensor_msgs/LaserScan` | `world_sim_node` |
| `/robot_i/odom` | `nav_msgs/Odometry` | `world_sim_node` |
| `/robot_i/cmd_vel` | `geometry_msgs/Twist` | `robot_node` |
| `/swarm/partial_map` | `swarmap_msgs/PartialMap` | each `robot_node` |
| `/swarm/robot_status` | `swarmap_msgs/RobotStatus` | each `robot_node` |
| `/swarm/frontier_bid` | `swarmap_msgs/FrontierBid` | each `robot_node` |
| `/swarm/global_map` | `nav_msgs/OccupancyGrid` | `map_aggregator_node` |
| `/swarm/health` | `std_msgs/String` | `swarm_monitor_node` |

---

## Custom messages (`swarmap_msgs`)

**`RobotStatus`** — 1 Hz heartbeat from each robot  
Fields: `robot_id`, `pose`, `battery_level`, `is_active`, `current_state` (`IDLE` / `EXPLORING` / `NAVIGATING` / `RETURNING` / `FAILED`), `cells_mapped`, `neighbour_ids`

**`PartialMap`** — compressed map delta shared between neighbours  
Fields: `robot_id`, `origin_x/y`, `width`, `height`, `resolution`, `data` (−1/0/100), `confidence[]`

**`FrontierBid`** — distributed auction bid  
Fields: `robot_id`, `frontier_centroid`, `bid_score` (lower = higher priority), `battery_level`, `claim`

**`NeighbourDiscovery`** — periodic peer advertisement  
Fields: `robot_id`, `pose`, `battery_level`

---

## Browser dashboard

Accessible at **http://localhost:5173** when launched with `dashboard:=true` or `./run.sh`.

| Panel | Description |
|-------|-------------|
| **MapCanvas** | Live merged occupancy grid with robot poses and frontier markers. Toggle: merged / ground truth / confidence |
| **StatsPanel** | Per-robot battery, state, cells mapped, and swarm-wide coverage % |
| **ControlPanel** | Pause / Resume / Stop / Reset map. Spawn robots. Kill selected robot |
| **NetworkGraph** | Real-time peer-to-peer communication graph (nodes = robots, edges = active links) |
| **ReplayPanel** | Step through a recorded mission bag |

rosbridge WebSocket runs on **ws://localhost:9090**.

---

## Running tests

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon test --packages-select swarmap_core swarmap_msgs
colcon test-result --verbose
```

C++ unit tests cover: frontier detection, occupancy grid logic, map merger, and distributed auction.

---

## Contributors 
mars.ciot@pes.edu

## Project structure

```
Swarmap/
├── src/
│   ├── swarmap_core/          # C++17 robot node, map merger, frontier explorer
│   │   ├── include/           # public headers
│   │   ├── src/               # node implementations + Python nodes
│   │   └── test/              # C++ unit tests (gtest)
│   ├── swarmap_msgs/          # custom ROS2 message definitions
│   ├── swarmap_bringup/       # launch files, params, RViz config
│   └── swarmap_dashboard/     # React + TypeScript browser dashboard
├── docs/                      # troubleshooting, topic naming, demo script
├── scripts/                   # bootstrap helper
├── run.sh                     # one-command launcher
└── requirements.txt           # Python pip dependencies
```

---

**Status:** simulation (RViz + browser dashboard), distributed frontier auction, C++ unit tests, CI wired.
