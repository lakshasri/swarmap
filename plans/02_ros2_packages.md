# Phase 2 — ROS2 Package Skeleton and Custom Messages

## Goal
Create the full `src/` directory layout with empty but buildable ROS2 packages, and define all custom message types before any implementation starts.

---

## Tasks

### 2.1 — Create Workspace src/ Layout

- [ ] Create directory structure:
  ```
  src/
  ├── swarmap_msgs/
  ├── swarmap_core/
  ├── swarmap_bringup/
  └── swarmap_dashboard/
  ```
- [ ] Each package gets: `package.xml`, `CMakeLists.txt` (or `setup.py` for Python-only), and a `src/` or `scripts/` subdirectory
- [ ] Run `colcon build` — all packages must build with zero errors before any logic is written

### 2.2 — swarmap_msgs: Custom Message Definitions
**This package must be built first — all others depend on it.**

- [ ] Create `msg/PartialMap.msg`:
  ```
  # Compressed occupancy grid chunk sent between neighbours
  std_msgs/Header header
  string robot_id
  int32 origin_x        # cell offset from world origin
  int32 origin_y
  int32 width
  int32 height
  float32 resolution    # metres per cell
  int8[] data           # -1=unknown, 0=free, 100=occupied
  float32[] confidence  # per-cell confidence 0.0-1.0
  ```

- [ ] Create `msg/FrontierBid.msg`:
  ```
  # Robot's bid to claim a frontier cell in the auction
  std_msgs/Header header
  string robot_id
  geometry_msgs/Point frontier_centroid
  float32 bid_score     # lower = higher priority (cost-to-go + gain)
  bool claim            # true = claiming, false = releasing
  ```

- [ ] Create `msg/RobotStatus.msg`:
  ```
  # Periodic health broadcast (every 1 Hz)
  std_msgs/Header header
  string robot_id
  geometry_msgs/Pose pose
  float32 battery_level   # 0.0-1.0
  bool is_active
  string current_state    # EXPLORING | RETURNING | IDLE | FAILED
  int32 cells_mapped
  float32[] neighbour_ids # robot IDs currently in comm range
  ```

- [ ] Create `msg/NeighbourDiscovery.msg`:
  ```
  # Robots announce their position for neighbour detection
  std_msgs/Header header
  string robot_id
  geometry_msgs/Point position
  float32 comm_radius
  ```

- [ ] Add all messages to `CMakeLists.txt` with `rosidl_generate_interfaces`
- [ ] Build and verify: `ros2 interface show swarmap_msgs/msg/PartialMap` works

### 2.3 — swarmap_core: C++ Package Setup
**Language:** C++17  
**Build system:** ament_cmake

- [ ] Configure `CMakeLists.txt` with dependencies:
  - `rclcpp`, `rclcpp_lifecycle`
  - `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `std_msgs`
  - `swarmap_msgs`
  - `tf2`, `tf2_ros`, `tf2_geometry_msgs`
  - `nav2_msgs` (for path planning interfaces)
- [ ] Create stub `.cpp` files (empty `main()`) for:
  - `robot_node.cpp`
  - `frontier_explorer.cpp`
  - `map_merger.cpp`
  - `failure_injector.cpp`
  - `neighbour_tracker.cpp`
- [ ] Register each as an executable in `CMakeLists.txt`
- [ ] Confirm `colcon build` succeeds with stubs

### 2.4 — swarmap_bringup: Launch Files

- [ ] Create Python launch files (empty scaffolds):
  - `launch/simulation.launch.py` — spawns Gazebo + N robot nodes
  - `launch/dashboard.launch.py` — starts rosbridge + frontend
  - `launch/robots_only.launch.py` — robots without Gazebo (for hardware)
- [ ] Create parameter file `config/default_params.yaml`:
  ```yaml
  robot_node:
    ros__parameters:
      sensor_range: 5.0         # metres
      comm_radius: 8.0          # metres
      map_resolution: 0.1       # m/cell
      update_rate: 5.0          # Hz
      failure_rate: 0.0         # probability per minute
      frontier_min_size: 5      # minimum frontier cell count
  ```
- [ ] Create `worlds/warehouse.sdf` — a placeholder SDF world file
- [ ] Create `urdf/diff_drive_robot.urdf.xacro` — minimal diff-drive robot model

### 2.5 — swarmap_dashboard: Node.js Package Init

- [ ] Run `npm create vite@latest . -- --template react-ts` inside `swarmap_dashboard/`
- [ ] Install dependencies:
  ```
  roslib  roslibjs  @types/roslib
  recharts  (for stats graphs)
  zustand  (lightweight state management)
  ```
- [ ] Create empty component stubs:
  - `src/components/MapCanvas.tsx`
  - `src/components/StatsPanel.tsx`
  - `src/components/ControlPanel.tsx`
  - `src/components/NetworkGraph.tsx`
- [ ] Create `src/hooks/useRosBridge.ts` stub
- [ ] Verify: `npm run dev` starts without errors

### 2.6 — Topic and Service Naming Convention

Define the naming scheme before any node is written — consistency is critical with many robots.

| Topic Pattern | Type | Direction |
|---|---|---|
| `/robot_{id}/map` | `PartialMap` | Robot → neighbours |
| `/robot_{id}/pose` | `geometry_msgs/PoseStamped` | Robot → all |
| `/robot_{id}/status` | `RobotStatus` | Robot → all (1 Hz) |
| `/robot_{id}/frontier_bid` | `FrontierBid` | Robot → all |
| `/robot_{id}/cmd_vel` | `geometry_msgs/Twist` | Planner → robot |
| `/robot_{id}/scan` | `sensor_msgs/LaserScan` | Sensor → robot |
| `/swarm/discovery` | `NeighbourDiscovery` | Robot → all |
| `/swarm/global_map` | `nav_msgs/OccupancyGrid` | Dashboard aggregator → browser |
| `/swarm/stats` | `std_msgs/String` (JSON) | Dashboard aggregator → browser |

- [ ] Document this table in `docs/topic_naming.md`
- [ ] Add a linting script that checks all node subscriptions match this table

---

## Verification Checklist

- [ ] `colcon build --packages-select swarmap_msgs` succeeds
- [ ] `ros2 interface show swarmap_msgs/msg/PartialMap` prints the definition
- [ ] `colcon build` builds all four packages with zero errors
- [ ] `ros2 launch swarmap_bringup simulation.launch.py` starts without crashing (stubs only)
- [ ] `npm run dev` in `swarmap_dashboard/` opens a blank page at localhost:5173
