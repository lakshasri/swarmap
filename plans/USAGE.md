# SWARMAP — How to Run

## Prerequisites

```bash
# ROS2 Humble
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
     ros-humble-rosbridge-suite ros-humble-nav-msgs \
     ros-humble-robot-state-publisher ros-humble-xacro

# Node.js 18+
node --version   # must be >= 18

# Dashboard dependencies (one time)
cd ~/Desktop/PROJECTS/SWARMAP/Swarmap/src/swarmap_dashboard
npm install
npm run build    # builds dist/ for preview
```

---

## Build the ROS2 workspace

Always run from the workspace root (`Swarmap/`):

```bash
cd ~/Desktop/PROJECTS/SWARMAP/Swarmap
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

> Re-run `source install/setup.bash` every time you open a new terminal.

---

## Running

You need **two terminals** — one for simulation, one for the dashboard.

### Terminal 1 — Simulation

```bash
cd ~/Desktop/PROJECTS/SWARMAP/Swarmap
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch swarmap_bringup simulation.launch.py
```

This starts:
- Ignition Gazebo with the warehouse world
- `num_robots` robots spawned and bridged
- `robot_node` per robot (lifecycle, exploration, mapping)
- `map_aggregator_node` (global map + stats)
- `swarm_monitor_node` (health + topology)
- `failure_injector_node` (fault injection)
- `param_service_node` (runtime param updates)

### Terminal 2 — Dashboard

```bash
cd ~/Desktop/PROJECTS/SWARMAP/Swarmap
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch swarmap_bringup dashboard.launch.py
```

Then open **http://localhost:5173** in a browser.

---

## Launch arguments

### simulation.launch.py

| Argument | Default | Description |
|---|---|---|
| `num_robots` | `5` | Number of robots to spawn |
| `world` | `warehouse` | World file — `warehouse` or `large_office` |
| `sensor_range` | `5.0` | LiDAR max range (metres) |
| `comm_radius` | `8.0` | Inter-robot communication radius (metres) |
| `noise_level` | `0.0` | Gaussian noise on lidar ranges (0 = none) |
| `failure_rate` | `0.0` | Robot failure probability per minute (0–0.45) |
| `map_resolution` | `0.1` | Metres per occupancy grid cell |

Example:
```bash
ros2 launch swarmap_bringup simulation.launch.py \
  num_robots:=10 world:=large_office failure_rate:=0.1
```

### dashboard.launch.py

| Argument | Default | Description |
|---|---|---|
| `num_robots` | `10` | Must match simulation |
| `dashboard_port` | `5173` | Port for the Vite preview server |

---

## What you see in the dashboard

- **Map panel** — live occupancy grid, robot positions update in real time. Pan with drag, zoom with scroll.
- **Stats panel** — coverage % ring, active/failed robot count, per-robot state chip and battery bar, coverage-over-time chart.
- **Network graph** — communication topology. Nodes are robots, edges mean they are within comm radius.
- **Control panel** — sliders to change `num_robots`, `failure_rate`, `comm_radius` at runtime via ROS2 services.
- **Replay tab** — drag in a mission JSON file, scrub the timeline, play back at variable speed.

---

## ROS2 topics to monitor

```bash
# Global occupancy grid
ros2 topic echo /swarm/global_map

# Dashboard stats (JSON string)
ros2 topic echo /dashboard/stats

# Swarm health (JSON string)
ros2 topic echo /swarm/health

# Network topology (JSON string)
ros2 topic echo /dashboard/network_topology

# Per-robot status
ros2 topic echo /robot_0/status
```

---

## Useful runtime commands

```bash
# Check all running nodes
ros2 node list

# See a robot's lifecycle state
ros2 lifecycle get /robot_0/robot_node

# Inject a failure manually
ros2 topic pub /swarm/inject_failure std_msgs/String "data: 'robot_2'" --once

# Change failure mode at runtime
ros2 param set /failure_injector_node failure_mode cascade
```

---

## Rebuilding after code changes

```bash
cd ~/Desktop/PROJECTS/SWARMAP/Swarmap
source /opt/ros/humble/setup.bash
colcon build --packages-select swarmap_core swarmap_bringup
source install/setup.bash
```

To rebuild only the dashboard:

```bash
cd src/swarmap_dashboard
npm run build
```

---

## Common errors

**`package not found` on launch**
→ You forgot to `source install/setup.bash` after building.

**rosbridge fails to start**
→ Install it: `sudo apt install ros-humble-rosbridge-suite`

**Dashboard shows "Waiting…" forever**
→ Check rosbridge is running on port 9090: `ros2 node list | grep rosbridge`

**Gazebo doesn't open / crashes**
→ Check Ignition Fortress is installed: `ign gazebo --version` should say `6.x`

**Robots don't move**
→ Lifecycle activation takes ~2s per robot (staggered 0.5s apart). Wait for all robot_nodes to print `ACTIVE`.
