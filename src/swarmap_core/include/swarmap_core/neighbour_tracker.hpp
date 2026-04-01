#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace swarmap {

struct NeighbourInfo {
    std::string robot_id;
    float       x, y;
    float       comm_radius;
    double      last_seen_s;   // seconds since epoch
};

class NeighbourTracker {
public:
    // Process a discovery ping from another robot
    void updateNeighbour(const std::string &id, float x, float y,
                         float comm_radius, double now_s);

    // Return IDs of robots currently in range of self_x/self_y
    std::vector<std::string> getNeighbours(float self_x, float self_y,
                                            float self_comm_radius,
                                            double now_s,
                                            double expiry_s = 3.0) const;

    // Check if a specific robot is a current neighbour
    bool isNeighbour(const std::string &id) const;

private:
    std::unordered_map<std::string, NeighbourInfo> peers_;
};

} // namespace swarmap
