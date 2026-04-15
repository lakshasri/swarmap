# Phase 9 — MATLAB Simulation

## Goal
Use MATLAB to prototype exploration and merging algorithms offline, then connect MATLAB to the live ROS2 network for co-simulation and to generate publishable benchmark results.

---

## Tasks

### 9.1 — Environment Map Generator (`MapGenerator.m`)
Creates test environments so simulations are not limited to one hard-coded world.

- [ ] Implement `MapGenerator` class with method `generate(type, width, height, resolution)`:
  - `type = 'warehouse'` — rectangular rooms with shelving rows
  - `type = 'office'` — rooms + corridors with doors
  - `type = 'random'` — Perlin noise thresholded into free/occupied
  - `type = 'maze'` — recursive maze generation
- [ ] Return:
  - `grid` — binary occupancy matrix (0=free, 1=occupied)
  - `pgm_path` — saved `.pgm` file for use in Gazebo and Nav2
- [ ] Seed-based generation: same seed → same map (reproducibility)
- [ ] Save SDF snippet: `MapGenerator.toGazeboSDF(grid, output_path)`

### 9.2 — Offline Swarm Simulator (`SwarmSimulator.m`)
Pure MATLAB simulation — no ROS2 required.

- [ ] Implement `SwarmSimulator` class:
  ```matlab
  sim = SwarmSimulator(env_grid, ...
      'NumRobots', 10, ...
      'SensorRange', 5.0, ...
      'CommRadius', 8.0, ...
      'FailureRate', 0.30, ...
      'NoiseLevel', 0.05, ...
      'MapResolution', 0.1);
  ```
- [ ] Each simulated robot is a struct with: `id`, `pose`, `local_map`, `state`, `battery`
- [ ] Simulation loop at 5 Hz (200 ms steps):
  1. Each robot senses its surroundings (ray-cast with noise)
  2. Robots within comm range exchange partial maps
  3. Each robot runs frontier detection + auction bid
  4. Each robot moves one step toward its claimed frontier
  5. Apply failure injection
  6. Log state to output struct
- [ ] Implement the same occupancy grid log-odds update as the C++ node
- [ ] No GUI in the simulation loop — collect data only

### 9.3 — Frontier Explorer (`FrontierExplorer.m`)

- [ ] Port the C++ frontier detection algorithm to MATLAB:
  - `frontiers = detectFrontiers(grid)` using `bwboundaries` or manual BFS
  - `clusters = clusterFrontiers(frontiers, minSize)` using `dbscan` or custom connected-components
  - `score = scoreFrontier(cluster, robot_pose)` — distance + size weighting
- [ ] Verify outputs match C++ node outputs on the same grid (cross-validation test)

### 9.4 — Map Merger (`MapMerger.m`)

- [ ] Port the weighted confidence merge algorithm to MATLAB:
  ```matlab
  function merged = mergeGrids(grid_a, conf_a, grid_b, conf_b)
      known_a = grid_a ~= -1;
      known_b = grid_b ~= -1;
      both = known_a & known_b;
      merged = grid_a;
      merged(both) = (conf_a(both).*grid_a(both) + conf_b(both).*grid_b(both)) ...
                    ./ (conf_a(both) + conf_b(both));
      merged(~known_a & known_b) = grid_b(~known_a & known_b);
  end
  ```
- [ ] Verify against C++ implementation with the same input pairs

### 9.5 — Benchmark Suite (`RunBenchmarks.m`)
Parameterised sweeps to quantify system performance.

- [ ] Implement `RunBenchmarks` script with `parfor` over parameter combinations:
  ```matlab
  num_robots_range   = [2, 5, 10, 15, 20, 30];
  failure_rate_range = [0.0, 0.1, 0.2, 0.3, 0.4, 0.45];
  noise_range        = [0.0, 0.05, 0.1, 0.2];
  num_trials         = 5;  % per combination
  ```
- [ ] For each combination and trial:
  - Run `SwarmSimulator` for 300 simulated seconds
  - Record: final coverage %, time to 80% coverage, map accuracy, total failures
- [ ] Aggregate results: mean ± std per parameter combination
- [ ] Generate plots:
  - Coverage % vs. swarm size (line plot, one line per failure rate)
  - Time to 80% coverage vs. failure rate (bar chart)
  - Map accuracy vs. noise level (scatter + regression line)
  - Heat map: [num_robots × failure_rate] → final coverage

### 9.6 — Results Exporter (`ExportResults.m`)

- [ ] Save all results to `matlab/results/benchmark_{timestamp}.mat`
- [ ] Export CSV: `matlab/results/benchmark_{timestamp}.csv` (for Python/Excel analysis)
- [ ] Export PDF report: `matlab/results/report_{timestamp}.pdf` using MATLAB's `exportgraphics`
  - Cover page: project title, date, parameter ranges tested
  - One page per benchmark plot
  - Summary table: best configuration for each metric

### 9.7 — Online ROS2 Co-Simulation (`ROS2Bridge.m`)
Connect MATLAB to a live ROS2 network during a Gazebo simulation.

- [ ] Implement `ROS2Bridge` class:
  ```matlab
  bridge = ROS2Bridge('DomainID', 0);
  bridge.connect();
  ```
- [ ] Subscribe to `/swarm/global_map` and `/swarm/health`:
  ```matlab
  map_sub = ros2subscriber(bridge.node, '/swarm/global_map', 'nav_msgs/OccupancyGrid');
  ```
- [ ] Implement live accuracy calculation:
  - Load ground truth map from `.pgm` file
  - Every 5 seconds: receive latest global map, compute accuracy, append to time series
  - Plot live: `drawnow` update on a MATLAB figure
- [ ] Publish synthetic sensor noise injection to a running robot:
  - Create a ROS2 publisher on `/robot_0/scan_override`
  - Publish modified `LaserScan` messages with controlled noise
  - Used to test map merger robustness to bad sensor data
- [ ] At mission end: call `bridge.exportReport(filename)` to save accuracy time series

### 9.8 — Gazebo World Generation from MATLAB Map

- [ ] Implement `MapGenerator.toGazeboSDF(grid, resolution, output_path)`:
  - Convert occupied cells to `<box>` collision geometry in SDF
  - Group adjacent occupied cells into rectangles (reduce primitive count)
  - Output valid `.sdf` file loadable by Gazebo Harmonic
- [ ] Run: generate `warehouse.sdf`, `office.sdf`, `maze.sdf` and commit them to `worlds/`

---

## Verification Checklist

- [ ] `MapGenerator` produces a valid `.pgm` and `.sdf` for each map type
- [ ] `SwarmSimulator` with 10 robots and `FailureRate=0` reaches 90% coverage in < 300 s
- [ ] `FrontierExplorer.m` and C++ frontier detector produce identical clusters on the same grid
- [ ] `RunBenchmarks` completes a 6×6×3 parameter sweep with 5 trials each using `parfor` in < 30 minutes
- [ ] Benchmark plots are saved to PDF without manual intervention
- [ ] MATLAB connects to live ROS2 in Gazebo and plots coverage accuracy in real time
- [ ] Generated `.sdf` files load in Gazebo without errors

---

## Notes
- Use `parfor` for the benchmark sweep only — the ROS2 bridge is single-threaded
- MATLAB figures must use `set(gcf,'Visible','off')` when running headless (on a server)
- The MATLAB ROS2 interface requires a real ROS2 installation on the same machine — document this clearly
- Keep `SwarmSimulator` and `ROS2Bridge` independent classes — the simulator should run without any ROS2 connection
