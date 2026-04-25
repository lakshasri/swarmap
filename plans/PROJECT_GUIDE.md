# SWARMAP — Project Guide

A decentralised multi-robot exploration system. N simulated robots collaboratively
map an unknown world, avoid each other via a distributed auction for frontiers,
and share their discoveries peer-to-peer.

---

## 1. What it does (features)

| Feature | Where it lives |
|---|---|
| **Simulated 2D world** with axis-aligned rectangular obstacles | `swarmap_core/src/world_sim_node.py` |
| **Lifecycle-managed robot node** (configure → activate → shutdown) per robot | `swarmap_core/src/robot_node.cpp` |
| **Log-odds occupancy grid** per robot with probabilistic updates | `swarmap_core/src/occupancy_grid.cpp` |
| **Simulated LiDAR** (180 rays, range 5 m, optional Gaussian noise) | `_publish_scan` in `world_sim_node.py` |
| **Frontier detection** via 8-neighbour flood-fill & clustering | `swarmap_core/src/frontier_explorer.cpp` |
| **Distributed auction** — robots bid on frontiers, lowest score wins | `winsAuction` in `frontier_explorer.cpp` |
| **Peer-to-peer map merging** (log-odds blend weighted by confidence) | `swarmap_core/src/map_merger.cpp` |
| **Neighbour discovery** bounded by `comm_radius` | `swarmap_core/src/neighbour_tracker.cpp` |
| **A\* path planner** with obstacle clearance and stride-based downsampling | `swarmap_core/src/path_planner.cpp` |
| **Smooth path follower** with 1 m lookahead | `driveAlongPath` in `robot_node.cpp` |
| **5 dock stations** (4 quadrants + centre) with 7 s recharge | `nearestDock`, `on_configure` in `robot_node.cpp` |
| **Battery model** with predictive "return-to-dock" logic | `drainBattery` in `robot_node.cpp` |
| **Failure injection** (random / progressive / cascade modes) | `swarmap_core/src/failure_injector_node.cpp` |
| **Global map aggregator** publishes merged grid + RViz markers | `swarmap_core/src/map_aggregator_node.cpp` |
| **Browser dashboard** (React + Vite) with live map, spawn/kill, sim controls | `swarmap_dashboard/` |
| **RViz 3D view** with robot bodies, docks, paths, ground-truth overlay | `swarmap_bringup/rviz/swarm_debug.rviz` |
| **One-command launcher** (`./run.sh`) | `run.sh` |

---

## 2. System architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                         SWARMAP PROCESS GRAPH                         │
└──────────────────────────────────────────────────────────────────────┘

  world_sim_node.py   ────────►  /robot_i/scan, /robot_i/odom
  (ground truth,                  /world/ground_truth
   physics, LiDAR)
        ▲
        │ /robot_i/cmd_vel
        │
  ┌─────┴──────┐      /robot_i/map          ┌─────────────────────────┐
  │ robot_0..N │ ─────────────────────────► │ map_aggregator_node.cpp │
  │ (lifecycle │      /robot_i/status       │ global map merge,       │
  │  node)     │ ─────────────────────────► │ /swarm/global_map,      │
  │  - OccupancyGrid                        │ /dashboard/stats,       │
  │  - FrontierExplorer                     │ RViz markers            │
  │  - PathPlanner (A*)                     └─────────────────────────┘
  │  - MapMerger                                       ▲
  │  - NeighbourTracker                                │
  └────────────┘                                        │
        ▲  ▲  ▲                                         │
        │  │  │  /swarm/discovery, frontier_bid         │
        │  │  │  (P2P neighbour comms)                  │
        │  │  │                                         │
   ┌────┴──┴──┴───────────────────────────┐             │
   │  swarm_monitor_node.py                │             │
   │  health tracking, topology graph      │             │
   └───────────────────────────────────────┘             │
                        ▲                                │
                        │                                │
   ┌────────────────────┴────────────────┐               │
   │  dashboard_bridge_node.py           │ ←─────────────┤
   │  translates UI requests to ROS      │ (reset, stop, │
   │  (spawn, kill, pause, reset, …)     │  inject_fail) │
   └───────────────────────┬─────────────┘               │
                           │ rosbridge ws:9090           │
                           ▼                             │
   ┌─────────────────────────────────────────────────────┴──────────┐
   │          swarmap_dashboard  (React + Vite, localhost:5173)      │
   │   - MapCanvas (MERGED / GROUND TRUTH / CONFIDENCE)              │
   │   - ControlPanel (Pause / Resume / Stop / Reset / Spawn / Kill) │
   │   - StatsPanel, NetworkGraph                                    │
   └─────────────────────────────────────────────────────────────────┘
```

---

## 3. How it's implemented — the important bits

### 3.1 Perception — log-odds occupancy grid

Each robot carries its own [OccupancyGrid](../src/swarmap_core/include/swarmap_core/occupancy_grid.hpp):

- Cell stores `log_odds`, `confidence`, `dirty`.
- `updateCell(gx, gy, Δlog_odds)` clamps to ±5.
- Ray-cast from robot pose → endpoint. Free cells along the ray get `LOG_ODDS_FREE`, endpoint gets `LOG_ODDS_OCCUPIED` if the ray returned shorter than `range_max`.
- `getCellRos()` maps log-odds back to {`-1` unknown, `0` free, `100` occupied} using thresholds 0.35 / 0.75.
- **Thread-safe**: `std::shared_mutex` lets scan (write) and publish (read) interleave.

### 3.2 Planning — A\* with stride downsampling

[`PathPlanner::plan()`](../src/swarmap_core/src/path_planner.cpp) does 8-connected A\* on the robot's own occupancy grid. Three techniques make it fast enough at 500×500:

1. **Stride** (default 5): the planner treats every 5th cell as a node, giving an effective 100×100 grid → ~1 ms plans.
2. **Line-of-sight check**: for every stride step, every cell along the line is verified free (no tunneling through thin walls).
3. **Clearance** (default 1 cell): paths keep a 0.1 m buffer from walls.

Unknown cells are treated as traversable (`treat_unknown_as_free = true`) so robots plan paths *through* unexplored space to reach frontiers — that's how exploration actually happens.

### 3.3 Following — smooth proportional control with lookahead

- Robot aims at the waypoint ~1 m ahead of its current position, not the closest one.
- Waypoints within 0.6 m are popped as the robot sweeps past.
- Angular gain 1.4 (clamped to ±1 rad/s), linear speed 0.25–0.6 m/s depending on heading error and nearest obstacle.
- If a path cell becomes occupied between ticks, replan; if the new plan fails, abandon this frontier and pick another.

### 3.4 Coordination — distributed frontier auction

Every 200 ms, each robot detects local frontiers, scores them with

```
score = energy_factor · distance + 2 / (cluster_size + 1) + anti_revisit_penalty
```

then publishes its bid on `/{robot}/frontier_bid`. A robot "wins" a frontier iff no
neighbour within 1 m of the same centroid has a lower bid (ε = 0.01). Ties broken
lexicographically on robot id. Bids expire after 2 s (self-healing if a robot dies
mid-claim).

### 3.5 Map merging — P2P gossip

On first discovery, each robot subscribes to its neighbours' `/map` and
`/frontier_bid` topics. Neighbour maps come in as [`PartialMap`](../src/swarmap_msgs/msg/PartialMap.msg)
messages (full grid + per-cell confidence). [`MapMerger::merge()`](../src/swarmap_core/src/map_merger.cpp)
blends in log-odds space weighted by confidence. Net effect: every robot rapidly
converges on the union of everything the swarm has seen, so **every robot acts
on a near-global map** without any central server.

### 3.6 Docks & recharge

Five docks at quadrant centres + map centre. `drainBattery()` predicts
`time_to_empty` and `time_to_return`; if the latter exceeds a safety-scaled
fraction of the former, the robot switches to `RETURNING`. On arrival within
0.9 m of a dock it enters `DOCKING` state, drain is paused, and after 7 s the
battery is refilled to 100 %.

### 3.7 Dashboard & bridge

- The **React dashboard** (`swarmap_dashboard/src/`) is a Vite app that speaks ROS over WebSocket via `roslib`. It subscribes to `/dashboard/stats`, `/swarm/global_map`, `/world/ground_truth`, and publishes user intents like `/swarm/spawn_robot_request`.
- **`dashboard_bridge_node.py`** is the go-between: it listens for those request topics and calls into real ROS actions (lifecycle shutdown, subprocess spawn, `pkill`, pause/resume broadcasts, etc.).

---

## 4. What worked — design wins

| Choice | Why it's good |
|---|---|
| **Lifecycle nodes for robots** | Clean configure → activate → shutdown transitions, reusable for failure injection |
| **Grid resolution stayed at 0.1 m** | Map looks detailed; expensive A\* avoided via stride parameter instead of coarsening the grid |
| **Frontier-auction for coordination** | Fully decentralised, self-heals on robot death, needs no consensus protocol |
| **Peer-to-peer log-odds map merging** | Confidence-weighted blend; stale neighbour data naturally decays |
| **A\* with stride + clearance** | 100× speedup without losing path safety; catches thin walls via line-of-sight check |
| **1 m lookahead in path follower** | Smooth motion, no servo jitter between adjacent cells |
| **Predictive dock-return** | Robots never strand themselves with dead batteries |
| **Rosbridge dashboard** | No custom transport, any browser can connect; hot-reload during development |
| **Three-layer map view (merged / truth / confidence)** | Instantly shows how well the swarm knows the world vs. reality |
| **Carved spawn corridor + dock clearance** | Guaranteed good initial conditions regardless of obstacle RNG |

---

## 5. What didn't work — things we tried and backed out

### 5.1 Bug2 reactive navigation (deleted)

First attempt was a classic **Bug2 state machine**:
- `GOAL_SEEK` mode: drive straight at the frontier using heading-proportional control.
- `WALL_FOLLOW` mode: when something is within 1.2 m front, hug the right wall.
- Switch back once the front clears and we've moved ≥2 m from the hit point.

**Why it failed:**
- **Mode oscillation** — LiDAR noise pushed the front range across the 1.2 m threshold constantly, flipping modes every tick.
- **Concave-corner deadlock** — when both front *and* right were blocked, the robot spun in place, never moved, stuck-timeout tripped after 8 s, abandon-goal, repeat forever.
- **Topologically blind** — Bug2 can't reason about "the frontier is on the other side of this wall, go around"; it just follows whatever wall it hits, often in the wrong direction.

Replaced with A\* path planning, which is the correct tool for this job.

### 5.2 Robot body collision in `world_sim` (partially backed out)

Tried enforcing a 2-cell robot body radius in the physics check (each move tested a 5×5 cell neighbourhood). Sounded right physically, but:

- Stranded robots in corridors narrower than 0.5 m (they couldn't squeeze through gaps that A\* had planned through).
- Initial spawn positions occasionally put robot bodies half-inside walls; every move was rejected, robot never moved.

Backed out to point-mass collision in `world_sim` and trust **A\* clearance=1** to keep robots off walls. Much more stable.

### 5.3 Small pillar obstacles

Earlier world builder scattered 8 small 4-10 cell square pillars around the map. They made the world look busier, but:
- Robots would oscillate when planning around pillars that appeared in their local map edges.
- Many pillars blocked the tight paths between large obstacles, making some frontiers truly unreachable.

Removed. The large rectangular rooms give more than enough topology.

### 5.4 Failure-injection & noise sliders in the dashboard

Had `NOISE` and `FAILURE RATE` sliders plus a `FAILURE MODE` dropdown. They became UI clutter once the basics worked — removed per feedback. Failure injection still lives on ROS topics if needed (`/swarm/inject_failure_now`).

### 5.5 "Pure grid world" teleport motion

Considered making robots teleport cell-by-cell to fully eliminate physics edge cases. Rejected because:
- Jumpy motion looks bad on the dashboard.
- Continuous Twist→integration physics was not the actual bug; Bug2 was.

Kept continuous motion, fixed planning instead.

### 5.6 Over-tuned anti-revisit penalty

At one point sigma = 5 m, weight = 15. Made *every* frontier near any visited
spot look repulsive → robots all picked identical far-away goals and clustered up.
Tuned back to sigma = 3 m, weight = 6 — gentle bias without herding.

### 5.7 High angular gain (2.5) with aggressive turn-slowdown

Original follower had angular gain 2.5 and `linear.x ∝ (1 − |err|/1.5)`. Meant
robots came to a near-stop on even mild heading errors → stuttering. Dropped gain
to 1.4 and raised the linear floor to 0.25 m/s so the robot always makes progress.

---

## 6. Known pros & cons

### Pros

- **Fully decentralised** — no master/coordinator; robots only talk to radio-range neighbours.
- **Self-healing** — killing a robot mid-mission simply redistributes its frontiers via the auction. Failure-injector node can simulate progressive / cascade failures.
- **Live P2P map merging** — robots act on what the swarm collectively knows, not just what they've seen.
- **One-command launch** (`./run.sh`) for demo convenience; browser dashboard at `localhost:5173` requires no RViz.
- **ROS 2 Humble standard** — drop-in with rosbridge, RViz, etc.

### Cons

- **No real SLAM** — robots trust the world's absolute odometry; there's no loop closure or pose graph. Fine for a simulation, wouldn't fly on real hardware.
- **A\* on a 500×500 grid is expensive** even with stride=5; if the swarm scales past ~30 robots on a single machine, expect CPU pressure.
- **No dynamic obstacles** — other robots don't appear in each other's occupancy grids, so two robots can collide at high speed (world_sim has no inter-robot collision detection either).
- **Navigation is globally-informed but locally-reactive** — robots don't renegotiate plans; if two paths cross, whoever gets there first wins. Collision avoidance relies on the scan-based speed slowdown and isn't formal.
- **Frontier-exhausted fallback** — when no frontiers exist locally, the robot goes IDLE. There's no "go help a busy neighbour" behaviour.
- **Docks are static** — can't reposition or add more at runtime. A dock inside an obstacle breaks the robot; the world builder carves clearance but doesn't validate robot_node's dock constants.
- **Stale ROS DDS messages** — if you restart the sim without killing everything, pause-state messages from a previous session can freeze the new world_sim. `./run.sh --clean` works around this.

---

## 7. How to run it

```bash
cd ~/Desktop/PROJECTS/SWARMAP/Swarmap

./run.sh                 # 8 robots + dashboard
./run.sh 12              # 12 robots
./run.sh 8 --rviz        # 8 robots + dashboard + RViz
./run.sh --clean         # kill any stale processes, then launch
./run.sh --rebuild       # force a colcon rebuild before launching
```

Dashboard: **http://localhost:5173/**

Left panel:
- **SIMULATION**: PAUSE · RESUME · STOP · RESET MAP
- **SPAWN / KILL**: click +SPAWN ROBOT, or select a robot in the list and KILL it
- **SWARM**: live count of active robots

Map view toggles (top-right):
- **MERGED** — what the swarm knows (walls robots have seen, unknown walls in blue, unknown space in black).
- **GROUND TRUTH** — the actual world from `world_sim_node`. Useful for judging map quality.
- **CONFIDENCE** — heatmap of how certain each cell is. Dark blue = low, yellow = high.

---

## 8. File map

```
Swarmap/
├── run.sh                                  ← one-command launcher
├── plans/
│   ├── PROJECT_GUIDE.md                    ← you are here
│   └── ... (older design notes)
├── src/
│   ├── swarmap_msgs/                       ← custom ROS messages
│   ├── swarmap_core/
│   │   ├── include/swarmap_core/
│   │   │   ├── occupancy_grid.hpp
│   │   │   ├── frontier_explorer.hpp
│   │   │   ├── map_merger.hpp
│   │   │   ├── neighbour_tracker.hpp
│   │   │   └── path_planner.hpp            ← A* interface
│   │   └── src/
│   │       ├── robot_node.cpp              ← per-robot lifecycle node
│   │       ├── world_sim_node.py           ← simulator
│   │       ├── map_aggregator_node.cpp     ← global map + RViz markers
│   │       ├── swarm_monitor_node.py       ← health + topology
│   │       ├── dashboard_bridge_node.py    ← dashboard↔ROS
│   │       ├── failure_injector_node.cpp
│   │       ├── occupancy_grid.cpp
│   │       ├── frontier_explorer.cpp
│   │       ├── map_merger.cpp
│   │       ├── neighbour_tracker.cpp
│   │       └── path_planner.cpp            ← A* implementation
│   ├── swarmap_bringup/
│   │   ├── launch/simulation.launch.py
│   │   ├── rviz/swarm_debug.rviz
│   │   └── config/default_params.yaml
│   └── swarmap_dashboard/                  ← React + Vite app
│       ├── src/
│       │   ├── App.tsx
│       │   ├── hooks/useRosBridge.ts
│       │   └── components/
│       │       ├── MapCanvas.tsx
│       │       ├── ControlPanel.tsx
│       │       ├── StatsPanel.tsx
│       │       └── NetworkGraph.tsx
│       └── package.json
└── README.md
```
