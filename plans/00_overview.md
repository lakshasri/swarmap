# Swarmap — Master Task Plan

## Project Goal
Build a browser-based swarm robotics dashboard where autonomous robots explore and collaboratively map an unknown environment using ROS2 Humble for middleware and MATLAB for algorithmic simulation.

---

## Phase Structure

| Phase | Plan File | Focus | Depends On |
|---|---|---|---|
| 0 | `00_overview.md` | This index | — |
| 1 | `01_environment_setup.md` | Dev environment, ROS2, Gazebo, MATLAB | — |
| 2 | `02_ros2_packages.md` | Package skeleton, custom messages, build system | Phase 1 |
| 3 | `03_robot_node.md` | Core robot lifecycle node | Phase 2 |
| 4 | `04_frontier_exploration.md` | Frontier detection + distributed auction | Phase 3 |
| 5 | `05_map_merging.md` | Neighbour map sharing + occupancy grid merge | Phase 3 |
| 6 | `06_fault_tolerance.md` | Failure injection + swarm recovery | Phases 4, 5 |
| 7 | `07_dashboard_backend.md` | rosbridge WebSocket server + aggregation node | Phase 3 |
| 8 | `08_dashboard_frontend.md` | React dashboard, map canvas, controls | Phase 7 |
| 9 | `09_matlab_simulation.md` | MATLAB offline sim + online ROS2 co-sim | Phase 3 |
| 10 | `10_integration_testing.md` | End-to-end tests, benchmarks, CI | Phases 6, 8, 9 |
| 11 | `11_demo_scenarios.md` | Pre-built demo runs, fault tolerance proof | Phase 10 |

---

## Parallel Work Tracks

Phases 4, 5, 6 can be worked in parallel once Phase 3 is done.
Phases 7 and 8 can be worked in parallel with Phases 4–6.
Phase 9 (MATLAB) can start alongside Phase 2.

```
Phase 1 ──► Phase 2 ──► Phase 3 ──┬──► Phase 4 ──┐
                                   ├──► Phase 5 ──┤──► Phase 6 ──► Phase 10 ──► Phase 11
                                   ├──► Phase 7 ──► Phase 8 ──┘
                                   └──► Phase 9 ──────────────────┘
```

---

## Definition of Done (Project Level)

- [ ] 10+ robots explore and map a warehouse-sized environment in Gazebo
- [ ] No central map server — peer-to-peer only
- [ ] Browser dashboard shows live map, stats, and controls
- [ ] Mission completes with >85% coverage even when 40% of robots fail
- [ ] MATLAB generates accuracy vs. swarm-size benchmark plots
- [ ] All ROS2 nodes pass integration tests
