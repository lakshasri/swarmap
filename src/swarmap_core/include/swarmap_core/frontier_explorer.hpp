#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include "swarmap_core/occupancy_grid.hpp"
#include "swarmap_msgs/msg/frontier_bid.hpp"

namespace swarmap {

struct FrontierCluster {
    std::vector<std::pair<int,int>> cells;  
    float centroid_wx = 0.0f;  
    float centroid_wy = 0.0f;
    float score       = 0.0f;  
};

struct BidRecord {
    std::string robot_id;
    float       bid_score;
    float       cx, cy;   
    bool        claim;
    double      timestamp;
};

class FrontierExplorer {
public:
    explicit FrontierExplorer(float frontier_min_size, float anti_revisit_sigma,
                              float anti_revisit_weight, float battery_weight = 2.0f);


    std::vector<FrontierCluster> detect(const OccupancyGrid &grid,
                                        float robot_wx, float robot_wy,
                                        float battery = 1.0f);

    
    void recordBid(const swarmap_msgs::msg::FrontierBid &bid);

    
    void expireBids(double now_s, double timeout_s = 2.0);

    
    
    bool winsAuction(const std::string &own_id, float own_score,
                     float cx, float cy, float tolerance = 1.0f) const;

    
    void markVisited(float wx, float wy);

private:
    float min_cluster_size_;
    float anti_revisit_sigma_;
    float anti_revisit_weight_;
    float battery_weight_;

    std::vector<std::pair<float,float>> visited_centroids_;
    std::unordered_map<std::string, BidRecord> neighbour_bids_;  

    float antiRevisitPenalty(float wx, float wy) const;

    
    std::vector<FrontierCluster> clusterFrontiers(
        const std::vector<std::pair<int,int>> &cells,
        const OccupancyGrid &grid) const;
};

}
