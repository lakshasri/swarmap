# Phase 4 — Frontier Detection and Distributed Auction

## Goal
Each robot independently identifies unexplored frontiers in its local map and participates in a distributed auction so no two robots target the same frontier simultaneously — no central coordinator.

---

## Tasks

### 4.1 — Frontier Detection (`frontier_explorer.cpp`)
A frontier is a free cell (value 0) adjacent to an unknown cell (value -1).

- [ ] Implement `detectFrontiers(grid)`:
  - Iterate over all cells in the local occupancy grid
  - A cell is a frontier if: `cell == FREE` AND any 8-connected neighbour `== UNKNOWN`
  - Return list of frontier cells as `(grid_x, grid_y)` pairs
- [ ] Implement frontier clustering using connected-components (BFS):
  - Group adjacent frontier cells into clusters
  - Discard clusters smaller than `frontier_min_size` parameter (default 5 cells)
  - Compute cluster centroid as the target point
- [ ] Implement frontier scoring for each cluster:
  ```
  score = distance_to_robot + k * (1 / cluster_size)
  ```
  - Lower score = higher priority
  - `k` is a tunable weight parameter (default 2.0)
- [ ] Sort clusters by score; the top cluster is the robot's preferred frontier

### 4.2 — Distributed Frontier Auction
**Mechanism:** Robots broadcast bids; the robot with the lowest bid score wins a frontier. No central auctioneer.

- [ ] Subscribe to `/robot_*/frontier_bid` topics for all neighbours within comm range
  - Use a wildcard subscriber pattern with a regex filter on topic names
  - Only process bids from robots in the current neighbour list (from neighbour tracker)
- [ ] Implement bid cycle (triggered every 2 seconds or when current frontier is reached):
  1. Detect frontiers in local map
  2. Select preferred frontier (lowest score)
  3. Check received bids from neighbours for the same frontier:
     - If own bid score < all neighbour bids for this frontier → claim it
     - If a neighbour has a lower score → move to next-best frontier
  4. Publish `FrontierBid` on `/robot_{id}/frontier_bid` with `claim=true`
  5. Wait 200 ms for conflict responses before committing
- [ ] Implement bid timeout: if no acknowledgment in 500 ms, proceed anyway
- [ ] Release a frontier bid (`claim=false`) when:
  - The frontier is reached
  - The frontier disappears (becomes mapped)
  - The robot transitions to `RETURNING` or `FAILED`

### 4.3 — Path Planning to Frontier

- [ ] Implement `planPath(start, goal, grid)` using A* on the local occupancy grid:
  - Inflate obstacles by `robot_radius + safety_margin` (default 0.3 m) before planning
  - Return a list of waypoints `[(x1,y1), (x2,y2), ...]`
  - Return empty path if goal is unreachable → trigger next-best frontier
- [ ] Implement path execution:
  - Follow waypoints sequentially using `moveTo` from the robot node
  - Replan if a new obstacle is detected within 1.5 m of the current path
  - Abort and replan if stuck for more than 5 seconds (no position change > 0.1 m)

### 4.4 — Frontier Sharing Between Neighbours
Robots share their detected frontiers with neighbours so they can plan without sensing the same area.

- [ ] When merging a neighbour's partial map (Phase 5), re-run frontier detection on the merged area
- [ ] Add newly discovered frontiers from merged map to the local frontier list
- [ ] Prune frontiers that are now fully explored (all cells in cluster are known)

### 4.5 — Anti-Revisit Logic

- [ ] Maintain a `visited_frontiers` set storing centroids of previously reached frontiers
- [ ] Penalise frontiers near visited centroids: add `k2 * exp(-dist / sigma)` to their score
  - Default: `k2 = 5.0`, `sigma = 2.0` metres
- [ ] This biases each robot toward genuinely new areas without global coordination

### 4.6 — Exploration Loop Integration

- [ ] Wire everything into the robot state machine (Phase 3):
  - Entry to `EXPLORING` state → run frontier detection → start bid cycle → plan path → execute
  - `NAVIGATING` state → follow current path → on arrival → transition back to `EXPLORING`
  - `EXPLORING` state with no frontiers → transition to `IDLE` (map complete)

---

## Verification Checklist

- [ ] Single robot in Gazebo explores a room corner-to-corner without manual input
- [ ] Frontier cells are correctly identified (visualise with RViz2 MarkerArray)
- [ ] Two robots in the same room claim different frontiers and do not collide paths
- [ ] When one robot's frontier disappears (mapped by neighbour), robot switches target
- [ ] Robot does not revisit a frontier it already reached (anti-revisit active)
- [ ] 5-robot swarm achieves 80%+ map coverage of a 20×20 m room within 3 minutes

---

## Notes
- Frontier detection runs at 1 Hz — do not run it in the scan callback (too slow)
- The A* planner operates on a copy of the grid, not the live grid, to avoid race conditions
- Set a `max_frontier_distance` parameter (default 20 m) to skip unreachable far frontiers
- Log auction outcomes at `DEBUG` level to avoid flooding `/rosout`
