# Swarmap — Demo Script

Use this script during the final presentation. Run scenarios in order.
Visualisation is **RViz2**, which the launch file opens automatically.

---

## Pre-demo checklist (5 min before)

- [ ] `source /opt/ros/humble/setup.bash` and `source install/setup.bash`
      in every terminal.
- [ ] `ROS_DOMAIN_ID=0` set everywhere.
- [ ] Workspace built clean (`colcon build --symlink-install`).
- [ ] MATLAB open with `matlab/src/` on the path (for the scaling demo).
- [ ] Test the smoke launch once: `ros2 launch swarmap_bringup
      simulation.launch.py num_robots:=3` — close it after 30 s.

---

## Scenario 1 — Basic multi-robot exploration (~8 min)

**Goal:** Show 10 robots cooperatively mapping the procedural world.

### Steps

1. Launch simulation:
   ```bash
   ros2 launch swarmap_bringup simulation.launch.py \
       num_robots:=10 noise_level:=0.0
   ```
2. RViz opens. Talking points:
   - Robots spawn evenly along the world, each in its own
     `/robot_i` namespace.
   - LiDAR scans render as point clouds on top of the merged map.
   - As exploration proceeds, the merged map (`/swarm/global_map`) fills in.
3. Point at a frontier marker and explain the auction: the closest
   un-claimed centroid wins, ties break by `robot_id`.
4. Open a side terminal and show coverage growing:
   ```bash
   ros2 topic echo /swarm/health
   ```

---

## Scenario 2 — Fault tolerance under fire (~10 min)

**Goal:** Show that the swarm survives 40% mid-mission failures.

### Steps

1. Stop the previous launch (Ctrl-C).
2. Launch the failure scenario:
   ```bash
   ros2 launch swarmap_bringup demo_fault_tolerance.launch.py
   ```
3. RViz comes up again. Talking points:
   - The injector node will start killing random robots ~30 s in.
   - Watch the surviving robots pick up orphaned frontiers (they
     re-bid and win because dead bidders have expired).
   - The merged map keeps filling; the dead robot's last contribution
     stays visible.
4. In a side terminal:
   ```bash
   ros2 topic echo /swarm/events
   ```
   to narrate each shutdown live.

---

## Scenario 3 — MATLAB scaling + fault-tolerance benchmark (~5 min)

**Goal:** Show *quantitatively* that the swarm scales and tolerates failure.

### Steps

1. In MATLAB:
   ```matlab
   addpath(genpath('matlab/src'))
   RunBenchmarks('results/benchmark')
   ```
2. The script sweeps:
   - swarm size: 1 → 50 robots
   - failure rate: 0% → 60%
   and writes a PDF + CSV into `results/benchmark/`.
3. Open the resulting PDF and walk through:
   - Coverage vs. swarm size (sub-linear — diminishing returns past ~30).
   - Coverage vs. failure rate (graceful degradation, knee around 50%).

---

## Wrap-up

- Hand-off: the unit-test suite (`colcon test --packages-select swarmap_core`)
  proves the algorithms are correct in isolation.
- The MATLAB benchmark proves they scale.
- The two RViz demos prove the live system works end to end.
