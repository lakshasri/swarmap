# SWARMAP: Decentralized Multi-Robot Swarm Mapping

SWARMAP is a decentralized multi-robot system where a swarm of autonomous robots collaboratively explores and maps an unknown environment without any central coordinator. Each robot maintains its own local occupancy grid, shares map updates with nearby peers, and coordinates exploration targets through a distributed auction. The system runs on ROS2 Humble with a custom Python world simulator and does not depend on Gazebo.

---

## 1. Occupancy Grid Mapping

Each robot builds a probabilistic map of its surroundings using a 2D occupancy grid. Rather than storing raw probabilities, cells store log-odds values. This representation is well suited to incremental updates because incorporating a new sensor reading reduces to a single addition per cell.

When a laser scan arrives, the robot traces a ray from its position to each scan endpoint using Bresenham's line algorithm. Every cell the ray passes through receives a free update, and the endpoint cell receives an occupied update (if the ray hit an obstacle) or a free update (if it reached maximum range). The log-odds are clamped to prevent any single cell from becoming overly certain. A confidence value is derived from each cell's probability as `|p - 0.5| * 2`, where a value of 1.0 indicates full certainty and 0.0 indicates complete uncertainty.

**In the code:** The grid is defined in `occupancy_grid.hpp` / `occupancy_grid.cpp`. Ray-casting and scan processing happen inside `robot_node.cpp` in the `onScan()` callback.

---

## 2. Frontier-Based Exploration

A frontier is the boundary between explored free space and unexplored space. The exploration strategy is to repeatedly identify frontier regions, pick the most promising one, navigate to it, and map the area it reveals.

Frontier detection scans the occupancy grid for free cells that border at least one unknown cell. These cells are grouped into clusters using BFS-based connected-component analysis. Small clusters below a minimum size threshold are discarded as noise.

Each cluster is scored to determine priority. The scoring function balances three factors:

- **Distance** -- closer frontiers are preferred to reduce travel time.
- **Cluster size** -- larger frontiers are more likely to reveal significant unmapped area.
- **Anti-revisit penalty** -- frontiers near previously visited locations are penalised using a Gaussian decay kernel, preventing the robot from looping back to areas it has already covered.

Battery level also feeds into scoring through an energy factor that makes all frontiers appear "farther" as battery drains, biasing low-battery robots toward closer targets.

**In the code:** `frontier_explorer.hpp` / `frontier_explorer.cpp` handle detection, clustering, and scoring. The robot node calls the explorer on each exploration tick.

---

## 3. Distributed Frontier Auction

When multiple robots operate in the same environment, they risk converging on the same frontier and duplicating effort. SWARMAP prevents this through a fully decentralized auction -- there is no central auctioneer.

Each robot publishes a bid for its preferred frontier, containing the frontier location and a score. Neighbouring robots receive these bids through topic subscriptions. A robot considers itself the winner for a frontier if no neighbour has placed a lower (better) bid on the same location. If two robots bid the same score, the tie is broken by robot ID. Bids expire after a short timeout so that stale claims from failed or moved robots do not block others indefinitely.

**In the code:** Bids are published as `FrontierBid` messages on per-robot topics (`/robot_i/frontier_bid`). The auction resolution logic is in the `winsAuction()` method of `frontier_explorer.cpp`.

---

## 4. Peer-to-Peer Map Sharing and Merging

Robots do not broadcast their maps globally. Instead, each robot periodically announces its position and communication radius. When two robots are within each other's range, they subscribe to each other's map topics and begin exchanging local grid updates. If a robot moves out of range or fails, the subscription times out after 3 seconds and the neighbour is dropped. This models realistic wireless communication constraints.

When a map update arrives from a neighbour, the receiving robot merges it into its own grid. The merging algorithm uses confidence-weighted averaging: if the local cell is unknown, the neighbour's data is adopted directly; if both are known, their probabilities are blended in proportion to their confidence values. This ensures that well-observed cells are not easily overwritten by noisier observations.

**In the code:** Neighbour tracking is in `neighbour_tracker.hpp` / `neighbour_tracker.cpp`. Map fusion is in `map_merger.hpp` / `map_merger.cpp`. Dynamic subscription management happens in the `onDiscovery()` callback of `robot_node.cpp`.

---

## 5. Battery Management and Dock Return

Each robot simulates a finite battery that drains at a fixed rate over time. The return-to-dock decision is predictive rather than threshold-based: the robot continuously estimates how long it will take to travel back to its starting position (the dock) and compares this against its remaining battery life. When the estimated travel time exceeds a safety margin of the remaining time, the robot abandons its current frontier and navigates home.

This predictive approach ensures that robots far from the dock begin returning earlier than robots that happen to be nearby, and that no robot strands itself with an empty battery in the field.

**In the code:** Battery logic is inside `robot_node.cpp` in the `drainBattery()` timer callback and the state transition logic of `explorationTick()`.

---

## 6. Fault Tolerance

The system continues operating even when robots fail during a mission. A dedicated failure injector node can remove robots from the swarm at a configurable rate using one of three modes: random selection, progressive (sequential order), or cascade. Failures follow a Poisson stochastic process.

When a robot is selected for failure, the injector triggers a ROS2 lifecycle transition that gracefully shuts the robot down -- timers stop, subscriptions are released, and the robot ceases publishing. Surviving robots detect the absence through heartbeat timeout and stop expecting data from the failed peer. No reconfiguration or central intervention is required; the remaining robots simply continue exploring the portions of the map that still have frontiers.

**In the code:** `failure_injector_node.cpp` handles the stochastic selection and lifecycle shutdown. The robot's `on_deactivate()` and `on_shutdown()` callbacks in `robot_node.cpp` handle cleanup.

---

## 7. World Simulation

The environment is simulated by a lightweight Python node rather than a full physics engine. At startup, the simulator procedurally generates a 2D world with border walls and randomly placed rectangular obstacles. For each robot, it integrates velocity commands using Euler integration with collision detection, and generates simulated laser scans by ray-marching against the ground truth grid. Optional Gaussian noise can be added to the scan readings to simulate sensor imperfections.

The simulator publishes odometry and laser scans at 10 Hz per robot, along with the ground truth occupancy grid at 2 Hz for accuracy comparison. It also broadcasts the TF transforms that relate each robot's local coordinate frame to the global map frame.

**In the code:** `world_sim_node.py` contains the simulator. World generation, physics integration, and ray-casting are all handled within this single node.

---

## 8. Robot State Machine

Each robot operates as a finite state machine with five states:

| State | Behaviour |
|-------|-----------|
| **IDLE** | No reachable frontiers remain. The robot has finished mapping. |
| **EXPLORING** | The robot is detecting frontiers and bidding in auctions. |
| **NAVIGATING** | The robot has won an auction and is driving toward the frontier using a proportional controller. |
| **RETURNING** | Battery is low. The robot is heading back to the dock at the origin. |
| **FAILED** | The robot has been shut down by the failure injector. |

State transitions are evaluated on every exploration tick. The robot node is implemented as a ROS2 lifecycle node, which provides additional managed states (unconfigured, inactive, active, finalized) that the failure injector uses to cleanly bring robots up and down.

**In the code:** The state machine runs in `robot_node.cpp` inside `explorationTick()` and `driveTowardGoal()`.

---

## 9. Global Map Aggregation and Visualisation

While each robot only has a partial view, a separate aggregator node subscribes to all robot maps and fuses them into a single global occupancy grid for monitoring purposes. This merged map is published to RViz2 along with frontier markers and per-robot pose visualisations.

An optional browser-based dashboard built with React connects to the ROS2 system through a WebSocket bridge. It provides an interactive map canvas, a statistics panel with coverage metrics and battery levels, a network topology graph showing which robots are neighbours, and a mission replay mode.

**In the code:** `map_aggregator_node.cpp` performs the fusion and publishes to RViz. The dashboard lives in the `swarmap_dashboard/` package and connects via `rosbridge_websocket`.

---

## Appendix: ROS2 Concepts

This section briefly covers the ROS2 concepts that underpin the project.

### Nodes
A node is the basic unit of computation in ROS2. Each process that does a distinct job -- reading sensors, planning motion, merging maps -- runs as a node. In SWARMAP, each robot is a node, and additional nodes handle simulation, aggregation, failure injection, and monitoring.

### Topics
Topics are named channels for asynchronous, many-to-many communication. A node publishes messages to a topic, and any number of nodes can subscribe. All sensor data, map updates, bids, and status broadcasts in SWARMAP flow through topics.

### Services
Services provide synchronous one-to-one request-response communication. SWARMAP uses lifecycle services to command state transitions (for example, telling a robot node to shut down).

### Messages
Messages are structured data types defined in `.msg` files. ROS2 generates serialisation code from these definitions so that the same message type can be used seamlessly across C++ and Python nodes. SWARMAP defines four custom messages: `FrontierBid`, `NeighbourDiscovery`, `PartialMap`, and `RobotStatus`.

### Lifecycle Nodes
A lifecycle node has managed states (unconfigured, inactive, active, finalized) with well-defined transitions. This pattern is used when nodes need to be brought up, configured, and shut down in a controlled sequence -- for example, cleanly stopping a robot mid-mission.

### TF2 Transforms
TF2 maintains a tree of coordinate frame relationships. The world simulator publishes transforms from the global `map` frame to each robot's `base_link` frame, so that every component in the system can convert positions between frames without manual bookkeeping.

### Quality of Service (QoS)
QoS profiles control the reliability and durability of topic connections. Best-effort QoS is used for high-frequency sensor data where dropping an occasional message is acceptable. Reliable QoS with transient-local durability is used for the global map so that a visualiser that starts late still receives the most recent map.

### Launch Files
Launch files (written in Python) start multiple nodes together with their parameters and namespaces configured. A single launch command can bring up the entire swarm -- simulator, all robots, aggregator, and visualiser -- with configurable arguments for the number of robots, sensor range, failure rate, and other settings.

### Namespaces
Each robot's topics are placed under a unique namespace (`/robot_0/`, `/robot_1/`, ...) so the same node executable can be instantiated multiple times without name collisions. System-wide topics like `/swarm/global_map` sit outside any robot namespace.

### Colcon
Colcon is the ROS2 build tool. It resolves dependencies between packages and compiles them in the correct order. SWARMAP's message definitions are compiled first, followed by the core library, then the launch configuration and dashboard packages.
