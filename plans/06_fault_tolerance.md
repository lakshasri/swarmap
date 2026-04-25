# Phase 6 — Fault Tolerance and Swarm Recovery

## Goal
Implement configurable robot failure injection and verify that the swarm continues mapping effectively even when up to 45% of robots fail mid-mission.

---

## Tasks

### 6.1 — Failure Injector (`failure_injector.cpp`)
A standalone node that externally kills robots based on configured probability.

- [ ] Read parameters:
  - `failure_rate` — probability a robot fails per minute (0.0 – 0.45)
  - `failure_mode` — `random | progressive | cascade`
  - `min_survivors` — never kill below this count (safety floor, default 1)
- [ ] Subscribe to `/swarm/discovery` to track active robot IDs
- [ ] Implement failure tick at 1 Hz:
  - For each active robot: if `rand() < failure_rate / 60.0` → kill it
  - `random` mode: any active robot can fail
  - `progressive` mode: failure rate increases 5% every 30 seconds
  - `cascade` mode: if robot fails, neighbours have 2× failure rate for next 10 seconds
- [ ] Killing a robot:
  - Publish `RobotStatus` with `is_active=false, current_state="FAILED"` on behalf of the robot
  - Call ROS2 lifecycle service `/robot_{id}/change_state` with `SHUTDOWN` transition
  - Log failure event to `/swarm/events` topic with timestamp and robot ID
- [ ] Publish failure event on `/swarm/events` (`std_msgs/String` JSON):
  ```json
  {"type": "robot_failure", "robot_id": "robot_3", "timestamp": 45.2, "active_count": 7}
  ```

### 6.2 — Swarm Recovery Behaviour
When robots fail, the remaining swarm must adapt — no re-planning coordinator required.

- [ ] When a robot detects a neighbour has failed (status `FAILED` or timeout > 5 s):
  - Remove it from neighbour list
  - Release any frontier bids the failed robot held (check `/robot_{failed_id}/frontier_bid`)
  - Re-run frontier detection immediately (don't wait for the next 1 Hz cycle)
- [ ] Implement frontier bid takeover:
  - If a frontier bid's `robot_id` is in the failed set AND the frontier is not yet explored:
    - Any neighbour can publish a new bid for that frontier
    - Standard auction rules apply (lowest score wins)
- [ ] Implement coverage gap detection:
  - If large unexplored area is adjacent to a failed robot's last known position:
    - Neighbouring robots increase their frontier score weight for that area
    - Weight decays after 30 seconds

### 6.3 — Liveness Monitor
A lightweight watchdog that tracks swarm health and triggers alerts.

- [ ] Create `liveness_monitor` node (Python is fine for this)
- [ ] Subscribe to all `/robot_*/status` topics
- [ ] Maintain per-robot heartbeat timestamps
- [ ] Detect stale robots (no status in 3 s) → mark as presumed-failed
- [ ] Publish to `/swarm/health` (`std_msgs/String` JSON) at 1 Hz:
  ```json
  {
    "active_robots": 8,
    "failed_robots": 2,
    "presumed_failed": ["robot_4"],
    "coverage_pct": 67.3,
    "mission_elapsed_s": 123
  }
  ```
- [ ] Log to `/rosout` at `WARN` level when `active_robots < total_robots / 2`

### 6.4 — Failure Event Log

- [ ] Create `failure_log.csv` writer in the liveness monitor:
  ```
  timestamp_s, robot_id, failure_mode, active_count_after, coverage_pct_at_failure
  ```
- [ ] Write one row per failure event to `results/failure_log_{timestamp}.csv`
- [ ] At mission end: compute and append summary row:
  ```
  total_failures, final_coverage_pct, mission_success (coverage > 85%)
  ```

### 6.5 — Fault Tolerance Proof Scenario
A scripted scenario designed to demonstrate the system's resilience.

- [ ] Create launch file `launch/fault_tolerance_demo.launch.py`:
  - 20 robots, `warehouse_large.sdf` environment
  - `failure_mode = progressive`, `failure_rate = 0.40`
  - Mission duration: 5 minutes
  - Dashboard auto-records the session
- [ ] Define success criteria in the launch file:
  - Final coverage ≥ 85% despite ≤ 8 survivors
  - No robot collision during recovery
  - Failure events visible in dashboard timeline

---

## Verification Checklist

- [ ] With `failure_rate=0.0`, all robots complete without failures
- [ ] With `failure_rate=0.40`, roughly 8 of 20 robots fail over a 5-minute run
- [ ] When a robot is killed mid-navigation, its frontier bid is released within 3 s
- [ ] Neighbours of a failed robot increase coverage of that robot's area
- [ ] `failure_log.csv` is written correctly with one row per failure
- [ ] Final coverage ≥ 85% in the fault tolerance demo scenario
- [ ] `cascade` mode causes visually observable cluster of failures in dashboard

---

## Notes
- Do NOT use `kill -9` or `pkill` to simulate failures — use the ROS2 lifecycle `SHUTDOWN` transition for clean teardown that allows the node to release bids
- The `min_survivors` guard prevents the injector from terminating all robots and leaving an incomplete map
- Test `cascade` mode carefully — it can cause rapid total swarm collapse if `failure_rate` is set too high
