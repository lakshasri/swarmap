#include "swarmap_core/frontier_explorer.hpp"
#include <queue>
#include <cmath>
#include <algorithm>

namespace swarmap {

FrontierExplorer::FrontierExplorer(float frontier_min_size,
                                   float anti_revisit_sigma,
                                   float anti_revisit_weight,
                                   float battery_weight)
    : min_cluster_size_(frontier_min_size),
      anti_revisit_sigma_(anti_revisit_sigma),
      anti_revisit_weight_(anti_revisit_weight),
      battery_weight_(battery_weight)
{}

std::vector<FrontierCluster> FrontierExplorer::detect(
    const OccupancyGrid &grid,
    float robot_wx, float robot_wy,
    float battery)
{
    
    std::vector<std::pair<int,int>> frontier_cells;
    frontier_cells.reserve(512);

    const int W = grid.width(), H = grid.height();
    const int dx8[] = {-1,-1,-1, 0, 0, 1, 1, 1};
    const int dy8[] = {-1, 0, 1,-1, 1,-1, 0, 1};

    for (int gy = 0; gy < H; ++gy) {
        for (int gx = 0; gx < W; ++gx) {
            if (grid.getCellRos(gx, gy) != CELL_FREE) continue;
            for (int k = 0; k < 8; ++k) {
                int nx = gx + dx8[k], ny = gy + dy8[k];
                if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
                if (grid.getCellRos(nx, ny) == CELL_UNKNOWN) {
                    frontier_cells.emplace_back(gx, gy);
                    break;
                }
            }
        }
    }

    
    auto clusters = clusterFrontiers(frontier_cells, grid);

    
    int robot_gx, robot_gy;
    grid.worldToGrid(robot_wx, robot_wy, robot_gx, robot_gy);

    // energy_factor > 1 when battery is low, making all frontiers look farther away.
    // A robot at 20% battery sees frontiers as 1 + 2.0*(1-0.2) = 2.6x more expensive.
    float energy_factor = 1.0f + battery_weight_ * (1.0f - std::clamp(battery, 0.0f, 1.0f));

    for (auto &cl : clusters) {
        float dwx = cl.centroid_wx - robot_wx;
        float dwy = cl.centroid_wy - robot_wy;
        float dist = std::hypot(dwx, dwy);
        float size_term = 1.0f / static_cast<float>(cl.cells.size() + 1);
        cl.score = energy_factor * dist + 2.0f * size_term
                   + antiRevisitPenalty(cl.centroid_wx, cl.centroid_wy);
    }

    std::sort(clusters.begin(), clusters.end(),
              [](const FrontierCluster &a, const FrontierCluster &b){
                  return a.score < b.score;
              });

    return clusters;
}

std::vector<FrontierCluster> FrontierExplorer::clusterFrontiers(
    const std::vector<std::pair<int,int>> &cells,
    const OccupancyGrid &grid) const
{
    const int W = grid.width(), H = grid.height();
    
    std::vector<std::vector<bool>> is_frontier(W, std::vector<bool>(H, false));
    for (auto &[gx, gy] : cells) is_frontier[gx][gy] = true;

    std::vector<std::vector<bool>> visited(W, std::vector<bool>(H, false));
    std::vector<FrontierCluster> result;

    const int dx4[] = {-1, 1, 0, 0};
    const int dy4[] = {0, 0, -1, 1};

    for (auto &[sx, sy] : cells) {
        if (visited[sx][sy]) continue;

        FrontierCluster cl;
        std::queue<std::pair<int,int>> q;
        q.push({sx, sy});
        visited[sx][sy] = true;

        while (!q.empty()) {
            auto [cx, cy] = q.front(); q.pop();
            cl.cells.emplace_back(cx, cy);

            for (int k = 0; k < 4; ++k) {
                int nx = cx + dx4[k], ny = cy + dy4[k];
                if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
                if (!visited[nx][ny] && is_frontier[nx][ny]) {
                    visited[nx][ny] = true;
                    q.push({nx, ny});
                }
            }
        }

        if (static_cast<float>(cl.cells.size()) < min_cluster_size_) continue;

        
        float sum_wx = 0.0f, sum_wy = 0.0f;
        for (auto &[gx, gy] : cl.cells) {
            float wx, wy;
            grid.gridToWorld(gx, gy, wx, wy);
            sum_wx += wx; sum_wy += wy;
        }
        float n = static_cast<float>(cl.cells.size());
        cl.centroid_wx = sum_wx / n;
        cl.centroid_wy = sum_wy / n;
        result.push_back(std::move(cl));
    }

    return result;
}

void FrontierExplorer::recordBid(const swarmap_msgs::msg::FrontierBid &bid)
{
    if (!bid.claim) {
        neighbour_bids_.erase(bid.robot_id);
        return;
    }
    BidRecord r;
    r.robot_id  = bid.robot_id;
    r.bid_score = bid.bid_score;
    r.cx        = bid.frontier_centroid.x;
    r.cy        = bid.frontier_centroid.y;
    r.claim     = bid.claim;
    r.timestamp = bid.header.stamp.sec + bid.header.stamp.nanosec * 1e-9;
    neighbour_bids_[bid.robot_id] = r;
}

void FrontierExplorer::expireBids(double now_s, double timeout_s)
{
    for (auto it = neighbour_bids_.begin(); it != neighbour_bids_.end(); ) {
        if (now_s - it->second.timestamp > timeout_s)
            it = neighbour_bids_.erase(it);
        else
            ++it;
    }
}

bool FrontierExplorer::winsAuction(const std::string &own_id, float own_score,
                                    float cx, float cy, float tolerance) const
{
    for (auto &[id, rec] : neighbour_bids_) {
        if (id == own_id) continue;
        
        float d = std::hypot(rec.cx - cx, rec.cy - cy);
        if (d > tolerance) continue;
        
        if (rec.bid_score < own_score) return false;
        if (rec.bid_score == own_score && id < own_id) return false;  
    }
    return true;
}

void FrontierExplorer::markVisited(float wx, float wy)
{
    visited_centroids_.emplace_back(wx, wy);
}

float FrontierExplorer::antiRevisitPenalty(float wx, float wy) const
{
    float penalty = 0.0f;
    for (auto &[vx, vy] : visited_centroids_) {
        float d = std::hypot(wx - vx, wy - vy);
        penalty += anti_revisit_weight_ * std::exp(-d / anti_revisit_sigma_);
    }
    return penalty;
}

}
