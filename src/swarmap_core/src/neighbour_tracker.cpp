#include "swarmap_core/neighbour_tracker.hpp"
#include <cmath>

namespace swarmap {

void NeighbourTracker::updateNeighbour(const std::string &id,
                                        float x, float y,
                                        float comm_radius, double now_s)
{
    auto &info      = peers_[id];
    info.robot_id   = id;
    info.x          = x;
    info.y          = y;
    info.comm_radius = comm_radius;
    info.last_seen_s = now_s;
}

std::vector<std::string> NeighbourTracker::getNeighbours(
    float self_x, float self_y, float self_comm_radius,
    double now_s, double expiry_s) const
{
    std::vector<std::string> result;
    for (auto &[id, info] : peers_) {
        
        if (now_s - info.last_seen_s > expiry_s) continue;

        float dist = std::hypot(self_x - info.x, self_y - info.y);
        float max_range = std::min(self_comm_radius, info.comm_radius);
        if (dist <= max_range)
            result.push_back(id);
    }
    return result;
}

bool NeighbourTracker::isNeighbour(const std::string &id) const
{
    return peers_.count(id) > 0;
}

}
