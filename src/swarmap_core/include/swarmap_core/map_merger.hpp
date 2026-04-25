#pragma once

#include "swarmap_core/occupancy_grid.hpp"
#include "swarmap_msgs/msg/partial_map.hpp"

namespace swarmap {

class MapMerger {
public:
    
    
    
    void merge(OccupancyGrid &local_grid,
               const swarmap_msgs::msg::PartialMap &incoming,
               int offset_x, int offset_y);

    
    
    static float accuracy(const OccupancyGrid &local_grid,
                          const OccupancyGrid &ground_truth);
};

}
