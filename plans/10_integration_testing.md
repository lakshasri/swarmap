# Phase 10 — Integration Testing and Benchmarking

## Goal
Verify that all components work correctly together end-to-end, establish reproducible benchmarks, and prevent regressions as the codebase grows.

---

## Tasks

### 10.1 — Unit Tests: ROS2 Nodes (C++)
Use `ament_cmake_gtest` for C++ unit tests.

- [ ] **Occupancy Grid Tests** (`test_grid.cpp`):
  - Log-odds update produces correct values at boundary conditions
  - `getCell` / `updateCell` round-trips are consistent
  - Ray-cast marks correct cells free and endpoint occupied

- [ ] **Frontier Detection Tests** (`test_frontier.cpp`):
  - Empty grid → no frontiers detected
  - Single free cell surrounded by unknowns → one frontier
  - Explored room with open door → frontier at doorway only
  - Cluster below `min_size` threshold is discarded

- [ ] **Map Merger Tests** (`test_merger.cpp`):
  - Merging two identical grids → same grid, confidence doubled
  - Merging known cell with unknown → known cell wins
  - Merging two conflicting knowns → weighted average correct
  - Offset partial map aligned correctly using TF2 transform

- [ ] **Bid Auction Tests** (`test_auction.cpp`):
  - Single robot always wins its own bid
  - Robot with lower score wins over robot with higher score
  - Tie-breaking by robot ID (lexicographic) is consistent
  - Released bid allows other robot to claim frontier

### 10.2 — Unit Tests: MATLAB
Use MATLAB's built-in `matlab.unittest.TestCase`.

- [ ] `TestFrontierExplorer.m` — same cases as C++ frontier tests
- [ ] `TestMapMerger.m` — same cases as C++ merger tests
- [ ] `TestSwarmSimulator.m` — coverage reaches expected range for given parameters
- [ ] Run: `runtests('matlab/tests')` from MATLAB command window

### 10.3 — Integration Tests: Launch + Node Communication
Use `launch_testing` Python framework.

- [ ] **2-Robot Handshake Test** (`test_two_robots.py`):
  - Launch 2 robots in Gazebo
  - Place them within comm range
  - Assert `/robot_0/map` and `/robot_1/map` are published within 5 s
  - Assert both `RobotStatus` messages show each other in `neighbour_ids`
  - Assert map cells increase over 30 seconds

- [ ] **Frontier Auction Test** (`test_frontier_auction.py`):
  - Launch 3 robots facing a common frontier
  - Assert only one robot claims the frontier (one bid with `claim=true`, others `claim=false`)
  - Assert claiming robot navigates toward the frontier centroid

- [ ] **Map Merge Integration Test** (`test_map_merge.py`):
  - Launch 2 robots, each explore half a room
  - After 60 s, assert each robot's local map contains cells from the other's area
  - Assert combined coverage > coverage of either robot alone

- [ ] **Failure Recovery Test** (`test_failure_recovery.py`):
  - Launch 5 robots
  - Kill robot_2 after 30 s via lifecycle service
  - Assert robot_2's frontier bid is released within 5 s
  - Assert at least one neighbour moves toward robot_2's last frontier within 15 s

### 10.4 — Performance Benchmarks

Run headless (no GUI) to measure system performance limits.

- [ ] **Throughput test** — 30 robots, warehouse map, 0% failure:
  - Measure: CPU and RAM usage per robot node
  - Measure: map publish latency (time from scan to `/robot_N/map` publish)
  - Pass criteria: all robots publish maps within 500 ms of receiving scan
  - Pass criteria: total CPU < 80% on a 4-core machine

- [ ] **WebSocket throughput test**:
  - Connect dashboard to rosbridge with 20 robots
  - Measure: WebSocket message size and frequency
  - Measure: dashboard frame rate under load
  - Pass criteria: stats update at ≥ 1.5 Hz, map at ≥ 0.5 Hz

- [ ] **Scalability test** — repeat with N = [5, 10, 15, 20, 25, 30]:
  - Plot: coverage completion time vs. N
  - Plot: CPU usage vs. N
  - Record the saturation point (N where adding more robots stops helping)

### 10.5 — Fault Tolerance Validation

- [ ] Run the fault tolerance demo scenario 10 times with different random seeds
- [ ] Record: final coverage % for each run
- [ ] Assert: mean coverage ≥ 85%, minimum coverage ≥ 75%
- [ ] Record: mission completion rate (runs where coverage > 85%)
- [ ] Assert: mission success rate ≥ 80%
- [ ] Generate a box-plot of coverage distribution across runs

### 10.6 — CI Pipeline Setup (GitHub Actions)

- [ ] Create `.github/workflows/ci.yml`:
  ```yaml
  name: Swarmap CI
  on: [push, pull_request]
  jobs:
    build-and-test:
      runs-on: ubuntu-22.04
      container: osrf/ros:humble-desktop
      steps:
        - uses: actions/checkout@v3
        - name: Install deps
          run: rosdep install --from-paths src --ignore-src -y
        - name: Build
          run: colcon build --symlink-install
        - name: Unit tests
          run: colcon test --packages-select swarmap_core swarmap_msgs
        - name: Test results
          run: colcon test-result --verbose
    frontend-test:
      runs-on: ubuntu-22.04
      steps:
        - uses: actions/checkout@v3
        - uses: actions/setup-node@v3
          with: { node-version: '20' }
        - run: cd src/swarmap_dashboard && npm ci && npm run build
  ```
- [ ] Add status badge to README

---

## Verification Checklist

- [ ] All C++ unit tests pass: `colcon test --packages-select swarmap_core`
- [ ] All MATLAB unit tests pass: `runtests('matlab/tests')`
- [ ] 2-robot handshake integration test passes in < 60 s
- [ ] Frontier auction test passes (only one robot claims each frontier)
- [ ] Failure recovery test passes (neighbour takes over within 15 s)
- [ ] 20-robot throughput test: CPU < 80%, map latency < 500 ms
- [ ] 10× fault tolerance runs: mean coverage ≥ 85%
- [ ] CI pipeline runs green on every push to `main`

---

## Notes
- Run integration tests with `--reuse-terminal` to see interleaved output from all nodes
- Use `ros2 bag record` during integration tests to save a `.bag` file for debugging failures
- The MATLAB unit tests require a MATLAB licence — they cannot run in GitHub Actions; run them locally before PRs
- Integration tests must be idempotent: each test starts Gazebo fresh and cleans up on exit
