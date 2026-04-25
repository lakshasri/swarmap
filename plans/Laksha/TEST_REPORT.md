# Swarmap — Test Report

**Date:** 2026-04-01
**Branch:** `core_nav`
**Author:** Lakshasri S
**Package tested:** `swarmap_core`
**ROS2 distro:** Humble
**Build tool:** colcon
**Test framework:** Google Test (gtest) via `ament_cmake_gtest`

---

## Summary

| Metric | Value |
|---|---|
| Total test suites | 4 |
| Total test cases | 34 |
| Passed | **34** |
| Failed | 0 |
| Errors | 0 |
| Skipped | 0 |
| Result | **PASS** |

---

## Suite Breakdown

### 1. `test_grid` — OccupancyGrid (12/12 passed)

Tests the core occupancy grid data structure: log-odds updates, coordinate conversion, dirty flags, and cell counting.

| Test Case | Result |
|---|---|
| `InitialisedUnknown` — all cells start as -1 | PASS |
| `DimensionsCorrect` — width/height/resolution match constructor args | PASS |
| `FreeCellUpdate` — repeated free updates produce CELL_FREE | PASS |
| `OccupiedCellUpdate` — repeated occupied updates produce CELL_OCCUPIED | PASS |
| `LogOddsClampedAtMax` — 100× occupied updates don't overflow | PASS |
| `WorldToGridRoundTrip` — world→grid→world stays within cell tolerance | PASS |
| `OutOfBoundsReturnsFalse` — worldToGrid returns false for out-of-range coords | PASS |
| `DirtyFlagSetOnUpdate` — updateCell marks cell dirty | PASS |
| `DirtyFlagClearedAfterClear` — clearDirty removes the flag | PASS |
| `MarkAllDirty` — markAllDirty flags every cell | PASS |
| `MappedCountStartsZero` — fresh grid has 0 mapped cells | PASS |
| `MappedCountIncreasesOnUpdate` — count grows after updateCell calls | PASS |

---

### 2. `test_frontier` — FrontierExplorer (6/6 passed)

Tests frontier cell detection, BFS clustering, score ordering, and the anti-revisit penalty.

| Test Case | Result |
|---|---|
| `EmptyGridNoFrontiers` — fully unknown grid produces no clusters | PASS |
| `SingleFreeRegionHasFrontiers` — free cells adjacent to unknown are detected | PASS |
| `ClusterBelowMinSizeDiscarded` — clusters smaller than `min_size` are dropped | PASS |
| `FullyExploredRoomNoFrontiers` — all-free grid has no frontiers | PASS |
| `ClustersReturnedInScoreOrder` — closer cluster scores lower than farther one | PASS |
| `AntiRevisitPenaltyIncreasesScore` — marking centroid visited raises its score | PASS |

---

### 3. `test_merger` — MapMerger (7/7 passed)

Tests the weighted-confidence map merge algorithm and the accuracy metric.

| Test Case | Result |
|---|---|
| `UnknownLocalAcceptsIncoming` — unknown local cell is updated from incoming | PASS |
| `IncomingUnknownSkipped` — incoming -1 cells do not overwrite known local cells | PASS |
| `TwoFreeGridsStayFree` — merging two FREE grids keeps all cells FREE | PASS |
| `TwoOccupiedGridsStayOccupied` — merging two OCCUPIED grids keeps all cells OCCUPIED | PASS |
| `OffsetAppliedCorrectly` — partial map lands at the right local grid coordinates | PASS |
| `AccuracyPerfectMatch` — identical local and ground truth grids score 1.0 | PASS |
| `AccuracyZeroWhenNothingMapped` — unmapped local grid scores 0.0 | PASS |

> **Note:** An off-by-one error in the initial test (`OffsetAppliedCorrectly` checked cell (12,12)
> when the patch only reaches (11,11)) was caught and corrected before the final run.

---

### 4. `test_auction` — FrontierExplorer auction (9/9 passed)

Tests the distributed frontier bid auction: win/lose logic, tie-breaking, expiry, and bid release.

| Test Case | Result |
|---|---|
| `SingleRobotAlwaysWins` — no competitors means always winning | PASS |
| `LowerScoreWins` — robot with lower score beats higher | PASS |
| `HigherScoreLoses` — robot with higher score loses to lower | PASS |
| `TieBreakHigherIdLoses` — equal scores broken by lexicographic robot ID (higher loses) | PASS |
| `TieBreakLowerIdWins` — equal scores broken by robot ID (lower wins) | PASS |
| `BidForDifferentFrontierIgnored` — bid outside tolerance radius is not a competitor | PASS |
| `ReleasedBidAllowsWin` — `claim=false` removes the bid, allowing rival to win | PASS |
| `ExpiredBidNoLongerBlocks` — bids older than timeout no longer block | PASS |
| `FreshBidStillBlocks` — recent bid within timeout still blocks rivals | PASS |

---

## Build Log

```
Starting >>> swarmap_msgs
Finished <<< swarmap_msgs [14.4s]

Starting >>> swarmap_core
Finished <<< swarmap_core [4.50s]

Summary: 2 packages finished — 0 errors, 0 warnings
```

**One build fix applied:**
- `src/swarmap_core/src/occupancy_grid.cpp` — added missing `#include <algorithm>` required for `std::clamp` (C++17)

---

## What Is Not Yet Tested

These components are not unit-testable in isolation yet — they require Gazebo or a live ROS2 network:

| Component | Reason | Planned test type |
|---|---|---|
| `robot_node.cpp` full lifecycle | Requires Gazebo sim + sensor topics | Launch test (Phase 10) |
| Gazebo bridge + spawning | Requires Gazebo Harmonic installed | Integration test |
| Neighbour discovery end-to-end | Requires 2+ live robot nodes | Integration test |
| `simulation.launch.py` | Requires Gazebo + URDF/SDF assets | Manual smoke test |

These will be covered in **Phase 10 — Integration Testing** once Janya's world files and URDF are in place.

---

## How to Reproduce

```bash
cd /path/to/Swarmap
source /opt/ros/humble/setup.bash
colcon build --packages-select swarmap_msgs swarmap_core
source install/setup.bash
colcon test --packages-select swarmap_core
colcon test-result --verbose
```
