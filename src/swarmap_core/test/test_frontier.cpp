#include <gtest/gtest.h>
#include "swarmap_core/frontier_explorer.hpp"
#include "swarmap_core/occupancy_grid.hpp"

using namespace swarmap;

// Helper — fill a rectangular region of the grid with a given log-odds bias
static void fillRect(OccupancyGrid &g, int x0, int y0, int x1, int y1, float lo)
{
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x)
            for (int i = 0; i < 5; ++i)   // repeat to push past the threshold
                g.updateCell(x, y, lo);
}

// ─────────────────────────────────────────────────────────────────────────────
// Basic detection
// ─────────────────────────────────────────────────────────────────────────────
TEST(FrontierTest, EmptyGridNoFrontiers)
{
    OccupancyGrid g(20, 20, 0.1f);
    FrontierExplorer ex(3.0f, 2.0f, 5.0f);
    auto clusters = ex.detect(g, 1.0f, 1.0f);
    EXPECT_TRUE(clusters.empty());
}

TEST(FrontierTest, SingleFreeRegionHasFrontiers)
{
    // Mark a 5×5 central region as free — borders touch unknown cells
    OccupancyGrid g(20, 20, 0.1f);
    fillRect(g, 7, 7, 13, 13, LOG_ODDS_FREE);

    FrontierExplorer ex(1.0f, 2.0f, 5.0f);   // min_size=1 to catch small clusters
    auto clusters = ex.detect(g, 1.0f, 1.0f);
    EXPECT_FALSE(clusters.empty());
}

TEST(FrontierTest, ClusterBelowMinSizeDiscarded)
{
    OccupancyGrid g(20, 20, 0.1f);
    // Mark only 2 cells free at an edge — cluster size = 2, min = 5
    for (int i = 0; i < 5; ++i) {
        g.updateCell(1, 10, LOG_ODDS_FREE);
        g.updateCell(2, 10, LOG_ODDS_FREE);
    }
    FrontierExplorer ex(5.0f, 2.0f, 5.0f);   // min_size = 5
    auto clusters = ex.detect(g, 0.0f, 0.0f);
    EXPECT_TRUE(clusters.empty());
}

TEST(FrontierTest, FullyExploredRoomNoFrontiers)
{
    // All cells known — no free cell adjacent to unknown
    OccupancyGrid g(10, 10, 0.1f);
    fillRect(g, 0, 0, 9, 9, LOG_ODDS_FREE);
    FrontierExplorer ex(1.0f, 2.0f, 5.0f);
    auto clusters = ex.detect(g, 0.5f, 0.5f);
    EXPECT_TRUE(clusters.empty());
}

// ─────────────────────────────────────────────────────────────────────────────
// Scoring and ordering
// ─────────────────────────────────────────────────────────────────────────────
TEST(FrontierTest, ClustersReturnedInScoreOrder)
{
    OccupancyGrid g(40, 40, 0.1f);
    // Two separate free patches at different distances from robot
    fillRect(g, 1, 1, 4, 4, LOG_ODDS_FREE);   // near robot
    fillRect(g, 30, 30, 35, 35, LOG_ODDS_FREE); // far from robot

    FrontierExplorer ex(1.0f, 2.0f, 5.0f);
    auto clusters = ex.detect(g, 0.15f, 0.15f);  // robot near first patch

    ASSERT_GE(clusters.size(), 2u);
    // First cluster must have lower (better) score than second
    EXPECT_LE(clusters[0].score, clusters[1].score);
}

// ─────────────────────────────────────────────────────────────────────────────
// Anti-revisit penalty
// ─────────────────────────────────────────────────────────────────────────────
TEST(FrontierTest, AntiRevisitPenaltyIncreasesScore)
{
    OccupancyGrid g(20, 20, 0.1f);
    fillRect(g, 2, 2, 6, 6, LOG_ODDS_FREE);

    FrontierExplorer ex(1.0f, 2.0f, 5.0f);

    auto before = ex.detect(g, 0.0f, 0.0f);
    ASSERT_FALSE(before.empty());
    float score_before = before[0].score;

    // Mark the centroid as visited
    ex.markVisited(before[0].centroid_wx, before[0].centroid_wy);

    auto after = ex.detect(g, 0.0f, 0.0f);
    ASSERT_FALSE(after.empty());
    float score_after = after[0].score;

    EXPECT_GT(score_after, score_before);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
