# Phase 5 — Decentralised Map Sharing and Merging

## Goal
Implement the peer-to-peer map gossip protocol. Robots share compressed partial maps only with neighbours within communication range, and merge received maps into their own local grid — no central server.

---

## Tasks

### 5.1 — Neighbour Tracker (`neighbour_tracker.cpp`)
Determines which robots are within communication range at any moment.

- [ ] Subscribe to `/swarm/discovery` (`NeighbourDiscovery`)
- [ ] Maintain a `std::unordered_map<string, NeighbourInfo>` with:
  - `robot_id`, `last_seen` timestamp, `position`, `comm_radius`
- [ ] Implement `getNeighbours()`:
  - Return IDs of robots where:  
    `distance(self_position, neighbour_position) < min(self_comm_radius, neighbour_comm_radius)`
  - Expire neighbours not heard from in `> 3 seconds`
- [ ] Publish updated neighbour list into the robot node every cycle
- [ ] Log neighbour count at `DEBUG` level — useful for debugging comm topology

### 5.2 — Map Publisher (Outgoing)
Robots publish their local map as a compressed `PartialMap` to be received by neighbours.

- [ ] Implement `compressMap(grid)`:
  - Extract only cells that have changed since last publish (dirty-flag per cell)
  - Fill `PartialMap.data[]` and `PartialMap.confidence[]` with changed cells only
  - Set `origin_x`, `origin_y`, `width`, `height` to the bounding box of dirty cells
  - Reset dirty flags after publish
- [ ] Publish compressed `PartialMap` on `/robot_{id}/map` at 2 Hz
- [ ] On first publish: send full grid (all cells), not just dirty cells

### 5.3 — Map Subscriber (Incoming)
Robots subscribe to their neighbours' map topics and merge them.

- [ ] In `NeighbourTracker`, when neighbour set changes:
  - Subscribe to `/robot_{neighbour_id}/map` for new neighbours
  - Unsubscribe from removed neighbours (to avoid stale data)
- [ ] Implement dynamic subscription management using `rclcpp::Subscription` pointers stored in a map

### 5.4 — Map Merger (`map_merger.cpp`)
Merges incoming `PartialMap` messages into the local occupancy grid.

- [ ] Implement `mergePartialMap(partial_map, local_grid)`:
  1. Transform `partial_map` origin from sender's frame to local world frame:
     - Both robots share a common world frame (world → odom → base_link)
     - Use TF2 to transform the map origin
  2. For each cell in `partial_map.data`:
     - Compute corresponding cell in `local_grid`
     - If `local_grid` cell is `UNKNOWN (-1)` → overwrite directly
     - If both cells are known → weighted average:
       ```
       new_value = (local_conf * local_val + recv_conf * recv_val) / (local_conf + recv_conf)
       new_conf  = max(local_conf, recv_conf)
       ```
  3. Mark merged cells as dirty (so they get re-published to this robot's own neighbours)
- [ ] Run merge in a separate thread with a mutex protecting the grid
- [ ] Merge callback must complete in < 10 ms — reject maps that would stall processing

### 5.5 — Map Accuracy Metric
Used by the dashboard and MATLAB to score the merged map against ground truth.

- [ ] Expose a ROS2 service `/robot_{id}/get_map_accuracy`:
  - Request: ground truth `OccupancyGrid`
  - Response: `float32 accuracy` (fraction of cells matching ground truth)
- [ ] Implement accuracy calculation:
  ```
  accuracy = (correctly_classified_known_cells) / (total_known_cells_in_ground_truth)
  ```
  - Skip cells still `UNKNOWN` in local map (robot hasn't seen them yet)
- [ ] Publish accuracy to `/robot_{id}/map_accuracy` (`std_msgs/Float32`) at 0.5 Hz

### 5.6 — Global Map Aggregator (Dashboard Only)
This node runs once for visualisation in the browser — it is **not** part of the robot logic.

- [ ] Create `aggregator_node.cpp` in `swarmap_core`
- [ ] Subscribe to all `/robot_*/map` topics (wildcard)
- [ ] Merge all received partial maps into one `nav_msgs/OccupancyGrid`
- [ ] Publish merged global map on `/swarm/global_map` at 1 Hz
- [ ] Publish coverage percentage on `/swarm/coverage` (`std_msgs/Float32`)
- [ ] This node is **read-only** — it never sends data back to robots

### 5.7 — Map Persistence
Save the evolving global map to disk for post-run analysis.

- [ ] On `SIGINT` (Ctrl+C), save `/swarm/global_map` to `results/map_{timestamp}.pgm`
- [ ] Use `nav2_map_server`'s map saver or implement directly with `pgm` writer
- [ ] Save metadata (robot count, mission duration, final coverage) to `results/map_{timestamp}.yaml`

---

## Verification Checklist

- [ ] Two robots exploring the same environment merge each other's maps correctly (inspect in RViz2)
- [ ] Removing a robot from the ROS2 network causes its neighbour to unsubscribe cleanly
- [ ] A robot entering comm range of another gets a full map on first contact
- [ ] Map accuracy metric increases monotonically as more area is explored
- [ ] The global aggregator map in RViz2 matches the union of all robot maps
- [ ] With 5 robots, a 20×20 m room reaches 90% accuracy within 2 minutes

---

## Notes
- The world frame must be consistent across all robots — set in Gazebo and broadcast via `/tf_static`
- Use `std::shared_mutex` for grid access (many concurrent readers, occasional writer)
- Publish maps on a separate timer, never inside the scan callback — avoids backpressure
- `partial_map.confidence` starts at 0.5 for a single scan and rises with repeated observations
