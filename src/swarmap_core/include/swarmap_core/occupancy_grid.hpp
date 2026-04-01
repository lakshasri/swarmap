#pragma once

#include <vector>
#include <cstdint>
#include <cmath>
#include <shared_mutex>

namespace swarmap {

// Log-odds clamping bounds
static constexpr float LOG_ODDS_MIN = -5.0f;
static constexpr float LOG_ODDS_MAX =  5.0f;
static constexpr float LOG_ODDS_FREE      = std::log(0.2f / 0.8f);   // p=0.2
static constexpr float LOG_ODDS_OCCUPIED  = std::log(0.8f / 0.2f);   // p=0.8

// ROS OccupancyGrid values
static constexpr int8_t CELL_UNKNOWN  = -1;
static constexpr int8_t CELL_FREE     =  0;
static constexpr int8_t CELL_OCCUPIED = 100;

struct Cell {
    float   log_odds   = 0.0f;   // running log-odds accumulator
    float   confidence = 0.0f;   // 0-1 confidence score
    bool    dirty      = false;  // needs re-publishing
};

class OccupancyGrid {
public:
    OccupancyGrid(int width, int height, float resolution,
                  float origin_x = 0.0f, float origin_y = 0.0f);

    // Update a single cell with a sensor reading (p_hit or p_free)
    void updateCell(int gx, int gy, float log_odds_update);

    // Read cell value as ROS int8 (-1 / 0 / 100)
    int8_t getCellRos(int gx, int gy) const;
    float  getCellConfidence(int gx, int gy) const;
    bool   isCellDirty(int gx, int gy) const;
    void   clearDirty(int gx, int gy);

    // World ↔ grid coordinate conversion
    bool worldToGrid(float wx, float wy, int &gx, int &gy) const;
    void gridToWorld(int gx, int gy, float &wx, float &wy) const;

    // Mark all cells dirty (used for first full publish)
    void markAllDirty();

    // Count non-unknown cells
    int mappedCellCount() const;

    int   width()      const { return width_; }
    int   height()     const { return height_; }
    float resolution() const { return resolution_; }
    float originX()    const { return origin_x_; }
    float originY()    const { return origin_y_; }

    // Thread-safe read access
    mutable std::shared_mutex mutex;

private:
    int   width_, height_;
    float resolution_;
    float origin_x_, origin_y_;
    std::vector<Cell> cells_;

    int idx(int gx, int gy) const { return gy * width_ + gx; }
    bool inBounds(int gx, int gy) const {
        return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
    }
};

} // namespace swarmap
