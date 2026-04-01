#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include "swarmap_core/occupancy_grid.hpp"
#include "swarmap_msgs/msg/frontier_bid.hpp"

namespace swarmap {

struct FrontierCluster {
    std::vector<std::pair<int,int>> cells;  // grid (x,y) pairs
    float centroid_wx = 0.0f;  // world-frame centroid
    float centroid_wy = 0.0f;
    float score       = 0.0f;  // lower = higher priority
};

struct BidRecord {
    std::string robot_id;
    float       bid_score;
    float       cx, cy;   // frontier centroid
    bool        claim;
    double      timestamp;
};

class FrontierExplorer {
public:
    explicit FrontierExplorer(float frontier_min_size, float anti_revisit_sigma,
                              float anti_revisit_weight);

    // Detect and score frontier clusters from the local grid
    std::vector<FrontierCluster> detect(const OccupancyGrid &grid,
                                        float robot_wx, float robot_wy);

    // Record a bid received from a neighbour
    void recordBid(const swarmap_msgs::msg::FrontierBid &bid);

    // Expire bids older than timeout_s seconds
    void expireBids(double now_s, double timeout_s = 2.0);

    // Check if robot_id wins the bid for the given frontier centroid
    // Returns true if own_score is strictly less than all competing bids for this frontier
    bool winsAuction(const std::string &own_id, float own_score,
                     float cx, float cy, float tolerance = 1.0f) const;

    // Record a visited frontier centroid for anti-revisit penalty
    void markVisited(float wx, float wy);

private:
    float min_cluster_size_;
    float anti_revisit_sigma_;
    float anti_revisit_weight_;

    std::vector<std::pair<float,float>> visited_centroids_;
    std::unordered_map<std::string, BidRecord> neighbour_bids_;  // keyed by robot_id

    float antiRevisitPenalty(float wx, float wy) const;

    // BFS connected-component labelling on frontier cells
    std::vector<FrontierCluster> clusterFrontiers(
        const std::vector<std::pair<int,int>> &cells,
        const OccupancyGrid &grid) const;
};

} // namespace swarmap
