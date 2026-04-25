#include "swarmap_core/occupancy_grid.hpp"
#include <algorithm>
#include <stdexcept>
#include <cmath>

namespace swarmap {

OccupancyGrid::OccupancyGrid(int width, int height, float resolution,
                              float origin_x, float origin_y)
    : width_(width), height_(height), resolution_(resolution),
      origin_x_(origin_x), origin_y_(origin_y),
      cells_(static_cast<size_t>(width * height))
{}

void OccupancyGrid::updateCell(int gx, int gy, float log_odds_update)
{
    if (!inBounds(gx, gy)) return;
    Cell &c = cells_[idx(gx, gy)];
    c.log_odds = std::clamp(c.log_odds + log_odds_update, LOG_ODDS_MIN, LOG_ODDS_MAX);
    float p = 1.0f / (1.0f + std::exp(-c.log_odds));
    c.confidence = std::abs(p - 0.5f) * 2.0f;
    c.dirty = true;
}

int8_t OccupancyGrid::getCellRos(int gx, int gy) const
{
    if (!inBounds(gx, gy)) return CELL_UNKNOWN;
    float lo = cells_[idx(gx, gy)].log_odds;
    // FIX #1: use threshold instead of exact float equality
    if (std::abs(lo) < 0.01f) return CELL_UNKNOWN;
    float p = 1.0f / (1.0f + std::exp(-lo));
    // Tighter thresholds to reduce noise → wall hallucinations.
    // A single hit (log-odds +1.386) is NOT enough to mark occupied; needs
    // multiple consistent hits (log-odds > ~1.9, i.e. p > 0.87).
    if (p > 0.87f) return CELL_OCCUPIED;
    if (p < 0.25f) return CELL_FREE;
    return CELL_UNKNOWN;
}

float OccupancyGrid::getCellConfidence(int gx, int gy) const
{
    if (!inBounds(gx, gy)) return 0.0f;
    return cells_[idx(gx, gy)].confidence;
}

bool OccupancyGrid::isCellDirty(int gx, int gy) const
{
    if (!inBounds(gx, gy)) return false;
    return cells_[idx(gx, gy)].dirty;
}

void OccupancyGrid::clearDirty(int gx, int gy)
{
    if (!inBounds(gx, gy)) return;
    cells_[idx(gx, gy)].dirty = false;
}

// FIX #2: use floor() instead of truncation for correct negative coords
bool OccupancyGrid::worldToGrid(float wx, float wy, int &gx, int &gy) const
{
    gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
    gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
    return inBounds(gx, gy);
}

void OccupancyGrid::gridToWorld(int gx, int gy, float &wx, float &wy) const
{
    wx = origin_x_ + (gx + 0.5f) * resolution_;
    wy = origin_y_ + (gy + 0.5f) * resolution_;
}

void OccupancyGrid::markAllDirty()
{
    for (auto &c : cells_) c.dirty = true;
}

int OccupancyGrid::mappedCellCount() const
{
    int count = 0;
    for (const auto &c : cells_)
        if (std::abs(c.log_odds) >= 0.01f) ++count;
    return count;
}

}
