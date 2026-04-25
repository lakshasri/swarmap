#include "swarmap_core/path_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

namespace swarmap {

namespace {

constexpr float SQRT2 = 1.41421356f;

struct Node {
    int   gx, gy;
    float f;
};
struct NodeCmp {
    bool operator()(const Node &a, const Node &b) const { return a.f > b.f; }
};

// Octile distance — correct heuristic for 8-connected grids
float heuristic(int ax, int ay, int bx, int by) {
    int dx = std::abs(ax - bx);
    int dy = std::abs(ay - by);
    return (dx + dy) + (SQRT2 - 2.0f) * std::min(dx, dy);
}

// Cell is blocked if any cell within `clearance` of (gx,gy) is occupied.
bool blocked(const OccupancyGrid &g, int gx, int gy,
             bool treat_unknown_as_free, int clearance)
{
    for (int dy = -clearance; dy <= clearance; ++dy) {
        for (int dx = -clearance; dx <= clearance; ++dx) {
            int nx = gx + dx, ny = gy + dy;
            int8_t v = g.getCellRos(nx, ny);
            if (v == CELL_OCCUPIED) return true;
            if (!treat_unknown_as_free && v == CELL_UNKNOWN) return true;
        }
    }
    return false;
}

}  // namespace

std::vector<std::pair<int,int>> PathPlanner::plan(const OccupancyGrid &grid,
                                                   int sx, int sy,
                                                   int gx, int gy,
                                                   bool treat_unknown_as_free,
                                                   int clearance,
                                                   int stride,
                                                   int max_iterations) const
{
    const int W = grid.width(), H = grid.height();
    if (stride < 1) stride = 1;
    if (sx < 0 || sx >= W || sy < 0 || sy >= H) return {};
    if (gx < 0 || gx >= W || gy < 0 || gy >= H) return {};

    // Snap start/goal to the stride grid so they line up with visited cells
    sx = (sx / stride) * stride;
    sy = (sy / stride) * stride;
    gx = (gx / stride) * stride;
    gy = (gy / stride) * stride;

    if (blocked(grid, gx, gy, treat_unknown_as_free, clearance)) return {};

    auto key = [W](int x, int y) { return static_cast<long long>(y) * W + x; };

    std::unordered_map<long long, float> gscore;
    std::unordered_map<long long, long long> came_from;
    std::priority_queue<Node, std::vector<Node>, NodeCmp> open;

    gscore[key(sx, sy)] = 0.0f;
    open.push({sx, sy, heuristic(sx, sy, gx, gy)});

    const int dx8[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy8[] = {-1, -1, -1, 0, 0, 1, 1, 1};

    int iter = 0;
    while (!open.empty() && iter < max_iterations) {
        ++iter;
        Node cur = open.top(); open.pop();
        if (std::abs(cur.gx - gx) <= stride && std::abs(cur.gy - gy) <= stride) {
            // close enough — accept
            gx = cur.gx; gy = cur.gy;
            break;
        }

        float cur_g = gscore[key(cur.gx, cur.gy)];

        for (int k = 0; k < 8; ++k) {
            int nx = cur.gx + dx8[k] * stride;
            int ny = cur.gy + dy8[k] * stride;
            if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
            if (blocked(grid, nx, ny, treat_unknown_as_free, clearance)) continue;

            // Check EVERY cell along the step to prevent tunneling through walls
            if (stride > 1) {
                bool line_blocked = false;
                int steps = stride;
                for (int s = 1; s < steps; ++s) {
                    int mx = cur.gx + (dx8[k] * s);
                    int my = cur.gy + (dy8[k] * s);
                    if (blocked(grid, mx, my, treat_unknown_as_free, clearance)) {
                        line_blocked = true; break;
                    }
                }
                if (line_blocked) continue;
            }

            float step_cost = ((dx8[k] != 0 && dy8[k] != 0) ? SQRT2 : 1.0f) * stride;
            float tentative = cur_g + step_cost;

            long long nk = key(nx, ny);
            auto it = gscore.find(nk);
            if (it == gscore.end() || tentative < it->second) {
                gscore[nk] = tentative;
                came_from[nk] = key(cur.gx, cur.gy);
                open.push({nx, ny, tentative + heuristic(nx, ny, gx, gy)});
            }
        }
    }

    // Reconstruct path if goal reached
    long long goal_k = key(gx, gy);
    if (gscore.find(goal_k) == gscore.end()) return {};

    std::vector<std::pair<int,int>> path;
    long long cur_k = goal_k;
    long long start_k = key(sx, sy);
    while (cur_k != start_k) {
        int x = static_cast<int>(cur_k % W);
        int y = static_cast<int>(cur_k / W);
        path.emplace_back(x, y);
        auto it = came_from.find(cur_k);
        if (it == came_from.end()) return {};  // broken
        cur_k = it->second;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

}
