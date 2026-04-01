# MARS — Task Assignments

Each person owns a distinct set of files. No two people write to the same source file.

---

## Lakshasri S

Files owned: `swarmap_msgs/msg/*.msg`, `swarmap_core/src/robot_node.cpp`, `swarmap_core/src/frontier_explorer.cpp`, `swarmap_core/src/map_merger.cpp`, `swarmap_core/src/neighbour_tracker.cpp`, `swarmap_bringup/launch/simulation.launch.py`, `rviz/swarm_debug.rviz`, `swarmap_core/test/*.cpp`

- **ROS2 Package Structure** — Create all four package directories with `CMakeLists.txt` and `package.xml`; confirm `colcon build` passes with empty stubs before any logic is written
- **Custom Message Definitions** — Define and build `PartialMap.msg`, `FrontierBid.msg`, `RobotStatus.msg`, `NeighbourDiscovery.msg` in `swarmap_msgs`; these are the shared contract all other nodes depend on
- **Robot Lifecycle Node** — Implement `robot_node.cpp` using `rclcpp_lifecycle`: parameter loading, all publishers/subscribers, and the state machine (IDLE → EXPLORING → RETURNING → FAILED)
- **Occupancy Grid and Sensor Integration** — Local occupancy grid with log-odds updates, Bresenham ray-cast from LiDAR scan, Gaussian noise injection, and dirty-cell tracking for incremental publishing
- **TF2 and Odometry** — Odometry subscriber, pose maintenance, and `odom → base_link` TF broadcast via `tf2_ros::TransformBroadcaster`
- **Frontier Detection** — `frontier_explorer.cpp`: BFS frontier detection, connected-component clustering, centroid scoring, and anti-revisit penalty
- **Distributed Frontier Auction** — Peer-to-peer bid cycle inside `frontier_explorer.cpp`: publish `FrontierBid`, filter to neighbours only, resolve by score, claim and release
- **Map Merging** — `map_merger.cpp`: weighted confidence merge of incoming `PartialMap` messages into the local grid, TF2 coordinate alignment, async merge with `std::shared_mutex`
- **Neighbour Tracker** — `neighbour_tracker.cpp`: track robots within comm range via `/swarm/discovery`, expire stale entries, manage dynamic subscriptions per neighbour
- **Gazebo Robot Spawning** — `simulation.launch.py`: spawn N robots at unique positions, bridge Gazebo sensor/odometry topics to ROS2 namespaces via `ros_gz_bridge`
- **RViz2 Debug Config** — `rviz/swarm_debug.rviz`: display robot poses, LiDAR scans, occupancy grids, and frontier markers; used only for development debugging, not the final dashboard
- **C++ Unit Tests** — `gtest` tests for grid log-odds update, frontier cluster detection, map merger weighted average, and auction conflict resolution; all inside `swarmap_core/test/`

---

## Janya Mahesh

Files owned: `scripts/bootstrap.sh`, `config/default_params.yaml`, `docs/topic_naming.md`, `docs/troubleshooting.md`, `urdf/diff_drive_robot.urdf.xacro`, `worlds/warehouse.sdf`, `worlds/large_office.sdf`, `docker/Dockerfile`, `docker-compose.yml`, `results/README.md`, `.gitignore`, `docs/demo_script.md`

- **Bootstrap Script** — `scripts/bootstrap.sh`: sources ROS2, runs `rosdep install`, builds the workspace, and prints a ready confirmation; must work on a fresh Ubuntu 22.04 machine
- **Default Parameter Config** — `config/default_params.yaml`: all tunable parameters (sensor range, comm radius, failure rate, map resolution, etc.) with an inline comment explaining each one
- **Topic Naming Doc** — `docs/topic_naming.md`: table of every ROS2 topic with its message type, which node publishes it, and which nodes subscribe; serves as the shared reference for all team members
- **Robot URDF** — `urdf/diff_drive_robot.urdf.xacro`: minimal diff-drive robot description with a LiDAR joint; used by Lakshasri's spawning launch file
- **Gazebo World Files** — `worlds/warehouse.sdf` and `worlds/large_office.sdf`: static environments for the simulation; create placeholder geometry early so Lakshasri is not blocked
- **Docker Setup** — `docker/Dockerfile` (based on `osrf/ros:humble-desktop`) and `docker-compose.yml` with services for ROS core, Gazebo, rosbridge, and frontend; allows any team member to run the full stack without manual install
- **Results Folder** — Create `results/` subdirectories (`demo_basic/`, `demo_fault_tolerance/`, `benchmark/`), write `results/README.md` explaining every output file type, and add `results/` to `.gitignore`
- **Troubleshooting Guide** — `docs/troubleshooting.md`: common failure modes (rosbridge refused, `ROS_DOMAIN_ID` mismatch, Gazebo crash, MATLAB ROS2 env not sourced) with step-by-step fixes
- **Demo Script** — `docs/demo_script.md`: narrated walkthrough for all three demo scenarios in presentation order; used during the final demo, not committed as code

---

## Aman Das

Files owned: `swarmap_core/src/map_aggregator_node.cpp`, `swarmap_bringup/launch/dashboard.launch.py`, `swarmap_dashboard/src/hooks/useRosBridge.ts`, `swarmap_dashboard/src/App.tsx`, `swarmap_dashboard/src/components/MapCanvas.tsx`, `swarmap_dashboard/src/components/StatsPanel.tsx`, `swarmap_dashboard/vite.config.ts`, `config/rosbridge_params.yaml`

- **rosbridge Configuration** — `config/rosbridge_params.yaml`: WebSocket port 9090, compression enabled, max message size 10 MB; add rosbridge launch to `dashboard.launch.py`
- **useRosBridge Hook** — `useRosBridge.ts`: connects to `ws://localhost:9090`, returns `{ ros, connected, error }`; `useRosTopic<T>` subscribes and returns latest message as React state with cleanup on unmount; `callRosService` returns a Promise
- **Dashboard App Layout** — `App.tsx`: three-column layout (control panel / map canvas / stats panel), dark theme with CSS variables, responsive to 1280 px
- **MapCanvas Component** — `MapCanvas.tsx`: HTML5 Canvas renderer for the live occupancy grid; cell colours for unknown/free/occupied/frontier; robot markers with ID and trajectory trail; pan and zoom; scale bar
- **StatsPanel Component** — `StatsPanel.tsx`: coverage ring, active vs. failed robot count, map accuracy indicator, per-robot table with battery bar, coverage-over-time line chart using Recharts; subscribes to `/dashboard/stats`
- **Map Aggregator Node** — `map_aggregator_node.cpp`: subscribes to all `/robot_*/map` topics, merges them into a global `OccupancyGrid`, computes coverage %, and publishes `/dashboard/stats` (JSON), `/dashboard/map_compressed` (Base64 delta), and `/dashboard/events`
- **Map Delta Serialiser** — Part of `map_aggregator_node.cpp`: full map on first send, changed-cells-only delta on subsequent sends; Base64-encoded JSON field; keeps WebSocket traffic below 1 MB/s with 20 robots
- **Frontend Build Config** — `vite.config.ts`: WebSocket proxy to port 9090, `npm run build` output to `dist/`; `dashboard.launch.py` starts `npm run preview` as a subprocess

---

## Vaibhav Handoo

Files owned: `swarmap_core/src/swarm_monitor_node.py`, `swarmap_core/src/param_service_node.py`, `swarmap_core/src/failure_injector_node.cpp`, `swarmap_dashboard/src/components/ControlPanel.tsx`, `swarmap_dashboard/src/components/NetworkGraph.tsx`, `swarmap_dashboard/src/components/ReplayPanel.tsx`, `.github/workflows/ci.yml`

- **Swarm Monitor Node** — `swarm_monitor_node.py` (Python): subscribes to all `/robot_*/status` topics, tracks heartbeat timestamps, detects stale robots (no ping > 3 s), publishes `/swarm/health` JSON at 1 Hz and `/dashboard/network_topology` (adjacency list JSON); logs `WARN` when active count drops below 50%
- **Parameter Service Node** — `param_service_node.py` (Python): standalone node exposing `/swarm/set_param` (broadcast value to all robot parameter servers) and `/swarm/set_num_robots` (spawn or lifecycle-shutdown robots dynamically); exposed through rosbridge so the frontend calls it as a ROS2 service
- **Failure Injector Node** — `failure_injector_node.cpp` (C++, standalone binary): reads `failure_rate` and `failure_mode` (random / progressive / cascade), calls the ROS2 lifecycle `SHUTDOWN` transition on selected robots, publishes failure events to `/swarm/events` JSON; writes `results/failure_log.csv` with one row per failure event
- **ControlPanel Component** — `ControlPanel.tsx`: sliders for swarm size, sensor range, noise level, failure rate, comm radius; each slider calls `callRosService` on `mouseup` targeting `/swarm/set_param`; "Applying…" spinner while in-flight; mission control buttons (Start / Pause / Reset / Save Map); failure mode dropdown
- **NetworkGraph Component** — `NetworkGraph.tsx`: Canvas-based comm topology graph; nodes at real-world (x, y) coordinates, edges opacity scaled by distance/comm_radius, animated 300 ms fade on edge add/remove, failed-robot ✕ overlay; subscribes to `/dashboard/network_topology`
- **Mission Replay Panel** — `ReplayPanel.tsx`: file drag-and-drop to load a recorded mission JSON; timeline scrubber; play/pause and speed controls (0.5× 1× 2× 4×); drives the same MapCanvas and StatsPanel with historical data by pushing frames into shared state
- **GitHub Actions CI Pipeline** — `.github/workflows/ci.yml`: ROS2 Humble build + colcon test job on `ubuntu-22.04`; separate Node.js 20 frontend build job; both triggered on push and pull request to `main`; status badge added to README
