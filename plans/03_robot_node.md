# Phase 3 — Core Robot Node

## Goal
Implement a fully functional ROS2 lifecycle node for a single robot that can sense its environment, maintain a local occupancy grid, broadcast its status, and respond to velocity commands. This is the foundation all other phases build on.

---

## Tasks

### 3.1 — Robot Lifecycle Node (`robot_node.cpp`)
**Pattern:** `rclcpp_lifecycle::LifecycleNode` with states: Unconfigured → Inactive → Active → Finalized

- [ ] Inherit from `rclcpp_lifecycle::LifecycleNode`
- [ ] Implement lifecycle callbacks:
  - `on_configure` — load parameters, create publishers/subscribers, allocate map
  - `on_activate` — start timers, enable publishing
  - `on_deactivate` — stop timers, stop publishing
  - `on_cleanup` — free resources
  - `on_shutdown` — emergency stop
- [ ] Load all parameters from `config/default_params.yaml` via `declare_parameter`
- [ ] Read robot ID from launch argument (e.g. `robot_0`, `robot_1`, ...)
- [ ] Namespace all topics under `/robot_{id}/`

### 3.2 — Local Occupancy Grid
**Type:** `nav_msgs/OccupancyGrid` maintained in memory

- [ ] Allocate grid sized to `(map_width / resolution) × (map_height / resolution)` cells
- [ ] Initialise all cells to `-1` (unknown)
- [ ] Implement `updateCell(x, y, value, confidence)`:
  - Use log-odds update: `l = l + log(p/(1-p))`
  - Clamp to `[-5, 5]`, convert to `[-1, 0, 100]` for ROS
  - Store per-cell confidence as `float32`
- [ ] Implement `getCell(x, y)` with world-to-grid coordinate transform
- [ ] Timer at 2 Hz to publish local map on `/robot_{id}/map`

### 3.3 — LiDAR Sensor Integration

- [ ] Subscribe to `/robot_{id}/scan` (`sensor_msgs/LaserScan`)
- [ ] In scan callback:
  - Ray-cast each beam using Bresenham's line algorithm
  - Mark all cells along the ray as free (value 0)
  - Mark the endpoint cell as occupied (value 100) if range < `range_max`
  - Apply Gaussian noise if `noise_level > 0`:  
    `noisy_range = range + N(0, noise_level * range_max)`
- [ ] Publish updated map after each scan callback

### 3.4 — Odometry and TF

- [ ] Subscribe to `/robot_{id}/odom` (`nav_msgs/Odometry`)
- [ ] Maintain current pose `(x, y, θ)` in world frame
- [ ] Publish TF transform `odom → base_link` via `tf2_ros::TransformBroadcaster`
- [ ] Subscribe to `/tf` and maintain a `tf2_ros::Buffer` for coordinate transforms

### 3.5 — Status Broadcaster

- [ ] Publish `RobotStatus` on `/robot_{id}/status` at 1 Hz:
  - `robot_id`, `pose`, `is_active=true`, `current_state`, `cells_mapped`
  - `battery_level` — starts at 1.0, decreases at configurable drain rate
  - `neighbour_ids` — populated by the neighbour tracker (Phase 5)
- [ ] Publish `NeighbourDiscovery` on `/swarm/discovery` at 2 Hz

### 3.6 — Velocity Command Output

- [ ] Create publisher on `/robot_{id}/cmd_vel` (`geometry_msgs/Twist`)
- [ ] Implement simple `moveTo(target_x, target_y)` using proportional controller:
  - Compute bearing error → angular velocity
  - Compute distance → linear velocity
  - Stop when within `goal_tolerance` (default 0.3 m)
- [ ] Implement `stop()` — publishes zero Twist
- [ ] This interface will be called by the frontier explorer (Phase 4)

### 3.7 — State Machine

- [ ] Implement internal states with `std::variant` or an enum class:
  ```
  IDLE → NAVIGATING → EXPLORING → MERGING → RETURNING → FAILED
  ```
- [ ] State transitions:
  - `IDLE` → `EXPLORING` when first frontier received
  - `NAVIGATING` → `EXPLORING` when goal reached
  - Any state → `RETURNING` when `battery_level < 0.15`
  - Any state → `FAILED` when failure injector triggers
  - `FAILED` is terminal — node transitions to `Inactive` lifecycle state
- [ ] Publish current state string in `RobotStatus`

### 3.8 — Gazebo Robot Spawning (swarmap_bringup)

- [ ] Update `simulation.launch.py` to spawn N robots:
  ```python
  for i in range(num_robots):
      # Spawn URDF into Gazebo at unique (x, y) start position
      # Launch robot_node with namespace robot_{i}
      # Launch robot_state_publisher with namespace robot_{i}
  ```
- [ ] Start positions spread on a grid or random within a spawn zone
- [ ] Wire Gazebo topics (`/model/robot_{i}/odometry`, `/model/robot_{i}/scan`) to ROS2 topics via `ros_gz_bridge`

---

## Single-Robot Verification Checklist

Run with one robot before scaling:

- [ ] Robot spawns in Gazebo at the correct start position
- [ ] `/robot_0/scan` receives LiDAR data
- [ ] `/robot_0/map` publishes an occupancy grid that grows as the robot moves
- [ ] `/robot_0/status` publishes at 1 Hz with correct state and pose
- [ ] `moveTo(5.0, 5.0)` drives the robot to that location and stops
- [ ] State transitions log correctly to `/rosout`
- [ ] Battery drains and robot transitions to `RETURNING`
- [ ] 10-robot launch completes without namespace collisions

---

## Notes
- Use `rclcpp::QoS(10).reliability(BEST_EFFORT)` for high-frequency topics (scan, odom)
- Use `RELIABLE` QoS for map and status topics
- Log all state transitions at `INFO` level with robot ID in the message
