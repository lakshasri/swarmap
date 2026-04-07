#pragma once

#include "swarmap_core/occupancy_grid.hpp"
#include "swarmap_msgs/msg/partial_map.hpp"

namespace swarmap {

class MapMerger {
public:
    // Merge an incoming PartialMap into the local grid.
    // offset_x/y are the sender's grid origin expressed in the local grid's
    // coordinate frame (computed externally via TF2).
    void merge(OccupancyGrid &local_grid,
               const swarmap_msgs::msg::PartialMap &incoming,
               int offset_x, int offset_y);

    // Compute map accuracy against a ground-truth grid [0.0, 1.0]
    // Only counts cells that are known (non -1) in the local grid.
    static float accuracy(const OccupancyGrid &local_grid,
                          const OccupancyGrid &ground_truth);
};

} // namespace swarmap