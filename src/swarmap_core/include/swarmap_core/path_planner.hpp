#pragma once

#include <vector>
#include <utility>
#include "swarmap_core/occupancy_grid.hpp"

namespace swarmap {

class PathPlanner {
public:
    // A* search on the occupancy grid. Returns cell-indexed waypoints from
    // start to goal (inclusive of goal, exclusive of start). Empty if no path.
    //
    // treat_unknown_as_free: if true, cells with log_odds ≈ 0 are traversable.
    // This is useful for exploration: robots need to move through unknown
    // space to discover new areas.
    //
    // clearance: inflate obstacles by N cells so paths don't hug walls.
    // stride: search on a coarser virtual grid (N cells per step). Huge speedup
    // for large grids — A* on 500x500 with stride=5 is effectively 100x100.
    std::vector<std::pair<int,int>> plan(const OccupancyGrid &grid,
                                          int sx, int sy,
                                          int gx, int gy,
                                          bool treat_unknown_as_free = true,
                                          int clearance = 1,
                                          int stride = 1,
                                          int max_iterations = 30000) const;
};

}
