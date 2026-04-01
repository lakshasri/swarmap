#include "swarmap_core/map_merger.hpp"
#include <cmath>

namespace swarmap {

void MapMerger::merge(OccupancyGrid &local_grid,
                      const swarmap_msgs::msg::PartialMap &incoming,
                      int offset_x, int offset_y)
{
    // incoming.data is row-major over (incoming.width × incoming.height)
    // offset_x/y translate the sender's grid origin into local grid cells
    for (int row = 0; row < incoming.height; ++row) {
        for (int col = 0; col < incoming.width; ++col) {
            int src_idx = row * incoming.width + col;
            int8_t src_val = incoming.data[src_idx];
            if (src_val == -1) continue;   // sender doesn't know — skip

            int local_gx = offset_x + incoming.origin_x + col;
            int local_gy = offset_y + incoming.origin_y + row;

            float recv_conf = (src_idx < static_cast<int>(incoming.confidence.size()))
                              ? incoming.confidence[src_idx] : 0.5f;

            int8_t local_val = local_grid.getCellRos(local_gx, local_gy);

            if (local_val == -1) {
                // Local cell unknown — accept incoming directly via a soft log-odds push
                float log_odds_update = (src_val == 100)
                    ? LOG_ODDS_OCCUPIED * recv_conf
                    : LOG_ODDS_FREE     * recv_conf;
                local_grid.updateCell(local_gx, local_gy, log_odds_update);
            } else {
                // Both known — weighted average by confidence
                float local_conf = local_grid.getCellConfidence(local_gx, local_gy);
                float total_conf = local_conf + recv_conf;
                if (total_conf < 1e-6f) continue;

                float blended_p = (local_conf  * (local_val  / 100.0f) +
                                   recv_conf   * (src_val    / 100.0f)) / total_conf;

                // Convert blended probability back to log-odds update
                blended_p = std::clamp(blended_p, 0.01f, 0.99f);
                float lo  = std::log(blended_p / (1.0f - blended_p));
                // Overwrite by applying a corrective update relative to current state
                float current_lo = std::log(
                    std::max(0.01f, static_cast<float>(local_val) / 100.0f) /
                    std::max(0.01f, 1.0f - static_cast<float>(local_val) / 100.0f));
                local_grid.updateCell(local_gx, local_gy, lo - current_lo);
            }
        }
    }
}

float MapMerger::accuracy(const OccupancyGrid &local_grid,
                           const OccupancyGrid &ground_truth)
{
    int known = 0, correct = 0;
    for (int gy = 0; gy < ground_truth.height(); ++gy) {
        for (int gx = 0; gx < ground_truth.width(); ++gx) {
            int8_t gt_val = ground_truth.getCellRos(gx, gy);
            if (gt_val == -1) continue;   // ground truth doesn't care about this cell

            int8_t local_val = local_grid.getCellRos(gx, gy);
            if (local_val == -1) continue; // robot hasn't seen this yet

            ++known;
            if (local_val == gt_val) ++correct;
        }
    }
    return (known == 0) ? 0.0f : static_cast<float>(correct) / static_cast<float>(known);
}

} // namespace swarmap
