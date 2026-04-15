# Swarmap — Slide Deck Brief

> Feed this document to an LLM and ask it to produce a thorough slide deck.
> It contains everything needed: problem, architecture, algorithms, stack,
> results, and a suggested slide order. The LLM can pick visuals and
> tighten copy; the facts and structure are already here.

---

## 0. How to use this brief

- Target audience: mixed technical review panel (professors + engineers).
- Desired length: 18–24 slides, ~20 min talk + 5 min Q&A.
- Tone: technical but accessible; prefer diagrams over prose; 3–5 bullets per slide, each ≤ 12 words.
- Every numbered section below maps to one slide (or a small group). The "Slide beats" line tells the LLM what MUST appear on that slide.

---

## 1. Title

**Swarmap — Decentralised Robot Exploration and Mapping**
Subtitle: *A fault-tolerant multi-robot SLAM swarm with a live browser dashboard.*

Team: Lakshasri S, Janya Mahesh, Aman Das, Vaibhav Handoo.
Stack line: ROS2 Humble · Gazebo Harmonic · C++17 · React · MATLAB.

Slide beats: project name, one-line pitch, team, stack icons.

---

## 2. The Problem

- Unknown indoor environments (warehouse, collapsed building, large office) must be mapped fast.
- A single robot is slow and a single point of failure.
- Centralised multi-robot SLAM needs a master server — brittle, bandwidth-heavy, doesn't scale.
- Real deployments lose robots to hardware failure, battery death, comms dropout.

Slide beats: pain points in a 2x2 (slow / fragile / centralised / unscalable).

---

## 3. Our Answer

A **decentralised swarm** where every robot is equal:
- Each robot builds its own local occupancy grid from LiDAR.
- Robots share *partial* maps only with neighbours within comm range.
- A **distributed frontier auction** assigns exploration targets — no central planner.
- The swarm survives ~40% mid-mission failures and still hits >85% coverage.
- A live browser dashboard visualises the merged map, comm graph, and per-robot stats.

Slide beats: 5 differentiators on one slide. Emphasise "no central server".

---

## 4. Live Demo Storyboard (1 slide + optional video)

Scenario A — `demo_basic.launch.py`: 10 robots, warehouse world, no failures, ~4 min, ~95% coverage.
Scenario B — `demo_fault_tolerance.launch.py`: 20 robots, large office, 40% progressive failures over 5 min, >85% coverage survived.

Slide beats: split screen — Gazebo view + dashboard map + coverage curve.

---

## 5. System Architecture (diagram slide)

Three layers:

1. **Simulation layer** — Gazebo Harmonic runs physics + LiDAR sensors; `ros_gz_bridge` exposes sensor / odometry topics into ROS2 namespaces (`/robot_0/...`, `/robot_1/...`).
2. **Swarm layer (ROS2)** — per-robot nodes communicate peer-to-peer via DDS topics. No master. Supporting nodes: map aggregator (read-only), swarm monitor, parameter service, failure injector.
3. **Dashboard layer** — `rosbridge_server` WebSocket (port 9090) ↔ React/Vite frontend (port 5173). MapCanvas, StatsPanel, ControlPanel, NetworkGraph, ReplayPanel.

Slide beats: boxes + arrows; label topics `/robot_*/map`, `/swarm/frontier_bid`, `/dashboard/map_compressed`.

---

## 6. The Robot Node (per-robot state machine)

Implemented as an `rclcpp_lifecycle` node: `src/swarmap_core/src/robot_node.cpp`.

States: **IDLE → EXPLORING → RETURNING → FAILED**.

Each tick the robot:
1. Ingests LiDAR scan → ray-casts into its local occupancy grid (Bresenham + log-odds, Gaussian noise injected).
2. Broadcasts its pose TF (`odom → base_link`) and dirty-cell deltas on `/robot_i/map` as `PartialMap`.
3. Runs neighbour discovery; subscribes dynamically to each neighbour's map.
4. Calls frontier explorer → bids on a frontier → navigates there.
5. Battery-aware: predicts time-to-dock vs. time-to-empty; returns early if unsafe.

Slide beats: state-machine diagram + tick-loop pseudocode.

---

## 7. Occupancy Grid + Sensor Model

`src/swarmap_core/src/occupancy_grid.cpp`.
- Log-odds representation, clamped to avoid saturation.
- Cell states: UNKNOWN (-1), FREE, OCCUPIED.
- Dirty-cell bitmap → incremental publishing (only changed cells go on the wire).
- Gaussian noise injected on range reads so the algorithms are exercised under realistic uncertainty.

Slide beats: small diagram of one scan updating cells; highlight "only deltas published".

---

## 8. Frontier Detection + Clustering

`src/swarmap_core/src/frontier_explorer.cpp`.
- Frontier cell = FREE cell adjacent to at least one UNKNOWN cell.
- BFS connected-component clustering → centroids.
- Score = Euclidean distance × (1 + `battery_weight` × (1 − battery_level)) × anti-revisit penalty.
- Clusters below `min_size` dropped to ignore sensor noise specks.

Slide beats: visual — grid with frontier cells highlighted + clusters colour-coded.

---

## 9. Distributed Frontier Auction (the key novel bit)

Per frontier pick:
1. Robot publishes `FrontierBid{robot_id, centroid_xy, score, battery_level, timestamp}` on `/swarm/frontier_bid`.
2. Listens for ~T seconds, filters to bids on the *same* frontier (within tolerance radius) and only from current neighbours.
3. Wins iff its score is the lowest; ties broken by lexicographic `robot_id`.
4. Expired bids (age > timeout) are ignored; `claim=false` releases a bid early.

No central auctioneer. Two robots can converge on the same frontier only during the window; loser picks the next-best.

Slide beats: sequence diagram of 3 robots bidding on one frontier.

---

## 10. Map Merging

`src/swarmap_core/src/map_merger.cpp`.
- On each `PartialMap` from a neighbour, align via TF2 → weighted confidence merge into the local grid.
- Weight by per-cell hit count so high-confidence observations dominate.
- Unknown cells in the incoming map never overwrite known local cells.
- Async merge under `std::shared_mutex` so reads (publishing, frontier detection) don't stall.

Slide beats: before/after grid visuals + the merge rule in one line.

---

## 11. Fault Tolerance

`src/swarmap_core/src/failure_injector_node.cpp` (C++ standalone binary).
Modes: **random**, **progressive**, **cascade**.

- Random: each tick, each robot has `failure_rate` probability of shutdown.
- Progressive: linearly ramps failure rate across the run.
- Cascade: killing robot X triggers neighbour failures within radius.

Failures transition the target's lifecycle node to `SHUTDOWN`; events land on `/swarm/events` and `results/failure_log.csv`.

Swarm recovery:
- Neighbour tracker drops the dead ID → frontier auction no longer counts its bids.
- Surviving robots pick up orphaned frontiers.
- Map merger keeps the dead robot's last contribution; it doesn't vanish from the global map.

Slide beats: "what fails, what survives" table; coverage-vs-time curve with failure events annotated.

---

## 12. Network Monitoring

- `swarm_monitor_node.py` subscribes to every `/robot_*/status`, tracks heartbeats, publishes `/swarm/health` and `/dashboard/network_topology` (adjacency list) at 1 Hz.
- Warns when active count drops below 50%.
- Drives the dashboard's `NetworkGraph` component.

Slide beats: screenshot of the topology graph — nodes, edges fading with distance, ✕ overlay on failed robots.

---

## 13. Dashboard (frontend)

`src/swarmap_dashboard/` — React + Vite + Canvas; connects via `useRosBridge` hook to `ws://localhost:9090`.

Components:
- **MapCanvas** — live occupancy grid, robot markers + trails, pan/zoom, scale bar, confidence heatmap overlay.
- **StatsPanel** — coverage ring, active/failed counts, per-robot battery bars, coverage-over-time (Recharts).
- **ControlPanel** — sliders for swarm size, sensor range, comm radius, failure rate; mission Start/Pause/Reset/Save Map; spawn/kill.
- **NetworkGraph** — comm topology graph.
- **ReplayPanel** — drag-and-drop mission JSON, scrubber, 0.5×–4× speed.

Slide beats: dashboard screenshot with component labels.

---

## 14. Map Aggregator + Wire Efficiency

`src/swarmap_core/src/map_aggregator_node.cpp`.
- Subscribes to every `/robot_*/map` topic, merges into one global `OccupancyGrid`.
- Publishes three topics to the frontend:
  - `/dashboard/map_compressed` — full grid on first send, Base64 delta thereafter.
  - `/dashboard/stats` — coverage %, robot table (JSON).
  - `/dashboard/events` — failure / join / claim events.
- Keeps WebSocket traffic below ~1 MB/s with 20 robots.

Slide beats: throughput chart showing full-vs-delta bandwidth.

---

## 15. MATLAB Co-Simulation

`matlab/`: `SwarmSimulator.m`, `FrontierExplorer.m`, `MapMerger.m`, `MapGenerator.m`, `ROS2Bridge.m`, `RunBenchmarks.m`.

- Pure-MATLAB offline sim for fast parameter sweeps (no Gazebo boot cost).
- `ROS2Bridge.m` lets MATLAB join a live ROS2 graph for online co-simulation.
- `RunBenchmarks.m` sweeps swarm size × failure rate and exports PDF plots into `results/benchmark/`.
- Independent `tests/Test*.m` suite validates the MATLAB reimplementation against the C++ ground truth.

Slide beats: one benchmark plot (coverage vs. swarm size, lines per failure rate).

---

## 16. Custom ROS2 Messages (`swarmap_msgs`)

| Message | Purpose |
|---|---|
| `PartialMap.msg` | Incremental map deltas shared between neighbours |
| `FrontierBid.msg` | Auction bid: id, centroid, score, battery, timestamp |
| `RobotStatus.msg` | Heartbeat: pose, battery, lifecycle state |
| `NeighbourDiscovery.msg` | Periodic "I am here" broadcast for comm-graph formation |

Slide beats: table + one-line rationale ("shared contract for every node").

---

## 17. Testing Strategy

- **Unit tests (gtest)** in `swarmap_core/test/`: 38 tests across occupancy grid, frontier detection, map merger, auction logic. All passing.
- **Launch-based integration tests** in `swarmap_bringup/test/`: two-robot auction, map merge, failure recovery; run against a headless Gazebo in CI when `SWARMAP_RUN_INTEGRATION=1`.
- **MATLAB tests** — `matlab/tests/` cross-validate algorithms.
- **CI** — `.github/workflows/ci.yml`: colcon build + tests + Node build on every push/PR; opt-in integration stage.

Slide beats: pyramid diagram (unit → integration → scenario) with counts.

---

## 18. Benchmarks

Scripts in `scripts/benchmarks/`:
- `perf_throughput.py` — how fast can one robot produce map deltas.
- `perf_scalability.sh` — coverage vs. swarm size headless sweep.
- `fault_tolerance_validation.py --runs N` — statistical coverage measurement under progressive failures.
- `plot_coverage.py` — renders matplotlib plots from CSVs.

Slide beats: three result plots in a row — throughput, scalability, fault tolerance.

---

## 19. Results vs. Goals

| Goal | Target | Status |
|---|---|---|
| 10+ robot exploration in Gazebo | 10+ | ✅ tested to 20 |
| No central map server | Peer-to-peer only | ✅ |
| Live browser dashboard | Map + stats + controls | ✅ |
| Survive 40% failure with >85% coverage | 85% | ✅ (fault-tolerance demo) |
| MATLAB benchmark plots | Accuracy vs. swarm size | ✅ |
| Integration tests pass | All | ✅ (38/38 unit, launch tests in CI) |

Slide beats: this table verbatim; green ticks.

---

## 20. What's Novel Here

- **Energy-aware frontier auction** — bid score multiplies distance by `(1 + battery_weight × (1 − battery))`, so tired robots bid less aggressively on far frontiers.
- **Predictive dock return** — compares time-to-dock vs. time-to-empty rather than using a fixed low-battery threshold.
- **Confidence heatmap overlay** — map cells coloured by *how many* robots have observed them (1 → blue, 3+ → green).
- **Runtime spawn/kill** — `robot_spawner_node.py` spawns or lifecycle-shuts down robots at runtime via ROS topics, managing Gazebo + bridge subprocesses.
- **Delta-only map streaming** — keeps the dashboard fluid even with 20 robots.

Slide beats: 5 icons, one sentence each.

---

## 21. Repo Layout (developer slide)

```
Swarmap/
├── src/
│   ├── swarmap_msgs/      # custom messages
│   ├── swarmap_core/      # robot + swarm C++/Python nodes, gtest suite
│   ├── swarmap_bringup/   # launches, URDF, worlds, integration tests
│   └── swarmap_dashboard/ # React frontend + aggregator launch
├── matlab/                # offline sim + benchmarks
├── scripts/               # bootstrap, demos, benchmarks
├── docker/                # Dockerfile + compose for one-shot boot
├── config/                # tunable params YAML, rosbridge config
├── docs/                  # topic naming, troubleshooting, demo script
└── plans/                 # phase plans, task ownership, this brief
```

Slide beats: tree on the left, one-line description of each dir on the right.

---

## 22. Team + Ownership

- **Lakshasri S** — core robot node, frontier auction, map merger, neighbour tracker, sim launch, gtest suite, RViz config.
- **Janya Mahesh** — bootstrap, default params, URDF, Gazebo worlds, Docker, topic-naming doc, troubleshooting, demo script.
- **Aman Das** — rosbridge config, `useRosBridge` hook, App layout, MapCanvas, StatsPanel, map aggregator, Vite config.
- **Vaibhav Handoo** — swarm monitor, parameter service, failure injector, ControlPanel, NetworkGraph, ReplayPanel, CI pipeline.

No two people write to the same source file (strict file-ownership rule).

Slide beats: 2×2 grid with each person's owned surfaces.

---

## 23. Limitations + Future Work

- Localisation relies on simulated odometry + TF; real hardware would need a SLAM front-end (Cartographer / slam_toolbox).
- Current comm model is a distance threshold; no packet loss, no bandwidth cap.
- Frontier auction assumes synchronised clocks; drift > bid timeout could cause starvation.
- No semantic map layer yet — all cells are free/occupied/unknown.
- Dashboard replay is client-side JSON; for hour-long runs, rosbag2 → server-side stream would scale better.

Slide beats: 5 honest bullets — not features, real limits.

---

## 24. Close / Q&A

- Repo: `github.com/lakshasri/Swarmap` (CI badge, release workflow, integration stage).
- One line to boot the whole stack: `ros2 launch swarmap_bringup simulation.launch.py num_robots:=10`.
- Two lines for the demos: `scripts/run_demos.sh`.

Slide beats: QR to repo, one-liner, "Questions?" footer.

---

## Appendix A — Key numbers for slides

- **Code**: ~7k LoC C++, ~1.5k LoC Python, ~2k LoC TypeScript, ~1k LoC MATLAB (approximate, recount if asked).
- **Unit tests**: 38, all passing.
- **Max tested swarm size**: 20 robots in Gazebo.
- **Failure-survival bar**: 40% progressive failures, >85% coverage.
- **Wire budget**: ≤ 1 MB/s dashboard traffic at 20 robots.
- **Rosbridge port**: 9090 · Dashboard port: 5173.
- **ROS2 distro**: Humble · Gazebo: Harmonic · Node: 20 · C++: 17.

---

## Appendix B — Suggested diagrams to request from the LLM

1. System architecture (Gazebo ↔ ROS2 topics ↔ rosbridge ↔ React).
2. Robot lifecycle state machine.
3. One-scan occupancy-grid update (ray-cast).
4. Frontier auction sequence diagram (3 robots, 1 frontier).
5. Comm-graph snapshot with failed node ✕.
6. Coverage-vs-time curve with failure events annotated.
7. Dashboard screenshot (labelled components).
8. Repo tree visual.
9. Team ownership 2×2 grid.

Ask the LLM to generate these as Mermaid / PlantUML / SVG where possible so they're editable.

---

## Appendix C — Do / Don't for the deck

Do:
- Show real screenshots of Gazebo + dashboard side by side.
- Keep code on slides to ≤ 8 lines; show *interfaces*, not implementations.
- Use consistent colour for each robot across all slides.

Don't:
- Don't put the full `FrontierBid.msg` definition on a slide — name the fields in prose.
- Don't open with tech stack; open with the problem.
- Don't hand-wave fault tolerance; show the coverage curve with the failure markers.
