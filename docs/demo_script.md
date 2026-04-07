# Swarmap — Demo Script

Use this script during the final presentation. Each scenario has an estimated runtime and a list of talking points. Run them in order.

---

## Pre-demo checklist (5 min before)

- [ ] `source install/setup.bash` in every terminal
- [ ] Browser open at `http://localhost:5173` (dashboard)
- [ ] RViz2 open with `rviz/swarm_debug.rviz`
- [ ] `ROS_DOMAIN_ID=0` set everywhere
- [ ] Gazebo window ready but not yet launched

---

## Scenario 1 — Basic multi-robot exploration (~8 min)

**Goal:** Show 10 robots cooperatively mapping the warehouse world.

### Steps

1. **Launch simulation**
   ```bash
   ros2 launch swarmap_bringup simulation.launch.py \
     num_robots:=10 world:=warehouse noise_level:=0.0
   ```

2. **Point out:** robots spawn in a grid pattern and immediately begin broadcasting discovery pings. Switch to RViz — show TF tree and laser scans.

3. **Point out dashboard:** MapCanvas starts filling in (grey → black/white); StatsPanel shows 10 active robots, coverage climbing from 0%.

4. **Explain frontier auction:** each robot picks the highest-scoring unclaimed frontier. No two robots target the same cell.

5. **Wait for ~80% coverage** (~5 min). Show final merged map vs ground truth in RViz.

**Talking points:**
- Distributed algorithm — no central coordinator
- Map merging via weighted log-odds confidence
- Anti-revisit penalty prevents robots revisiting explored areas

---

## Scenario 2 — Fault tolerance (~6 min)

**Goal:** Demonstrate graceful degradation when robots fail.

### Steps

1. **Relaunch with failure injection:**
   ```bash
   ros2 launch swarmap_bringup simulation.launch.py \
     num_robots:=10 world:=warehouse failure_rate:=0.08 failure_mode:=progressive
   ```

2. **Open the ControlPanel:** set failure rate slider to 0.08, mode = progressive.

3. **Watch the NetworkGraph:** edges disappear as robots fail; failed robots show ✕ overlay.

4. **StatsPanel:** active robot count drops; coverage growth slows but does not stop. Surviving robots continue claiming freed frontiers.

5. **Check failure log:**
   ```bash
   cat results/demo_fault_tolerance/failure_log.csv
   ```

**Talking points:**
- Heartbeat-based detection (no ping > 3 s → declared failed)
- Surviving robots transparently absorb the load
- Failure log records every event for post-analysis

---

## Scenario 3 — Scalability benchmark (~4 min)

**Goal:** Show coverage speed improves with robot count.

### Steps

1. Show `results/benchmark/sweep_results.csv` (pre-generated):
   | N robots | Time to 90% coverage |
   |----------|---------------------|
   | 5        | ~420 s              |
   | 10       | ~210 s              |
   | 15       | ~145 s              |
   | 20       | ~105 s              |

2. Show the `plots/` charts — near-linear speed-up up to ~15 robots, then communication overhead flattens the curve.

3. **Live demo** (optional — only if time allows):
   ```bash
   ros2 launch swarmap_bringup simulation.launch.py num_robots:=20 world:=large_office
   ```

**Talking points:**
- Communication overhead grows as O(N²) — motivates comm_radius cap
- Map delta compression keeps WebSocket bandwidth under 1 MB/s at 20 robots

---

## Q&A prompts

- "How do robots avoid collisions?" — no explicit collision avoidance; the frontier scorer penalises nearby frontiers and the diff-drive stops at walls via LiDAR range_max.
- "Why not use Nav2?" — intentional choice; demonstrates the core algorithm without middleware complexity.
- "What does 'confidence' mean in PartialMap?" — absolute value of log-odds probability mapped to [0, 1]; higher = more observations agree.
