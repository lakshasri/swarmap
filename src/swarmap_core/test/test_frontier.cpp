#include <gtest/gtest.h>
#include "swarmap_core/frontier_explorer.hpp"
#include "swarmap_core/occupancy_grid.hpp"

using namespace swarmap;

static void fillRect(OccupancyGrid &g, int x0, int y0, int x1, int y1, float lo)
{
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x)
            for (int i = 0; i < 5; ++i)   
                g.updateCell(x, y, lo);
}

TEST(FrontierTest, EmptyGridNoFrontiers)
{
    OccupancyGrid g(20, 20, 0.1f);
    FrontierExplorer ex(3.0f, 2.0f, 5.0f);
    auto clusters = ex.detect(g, 1.0f, 1.0f);
    EXPECT_TRUE(clusters.empty());
}

TEST(FrontierTest, SingleFreeRegionHasFrontiers)
{
    
    OccupancyGrid g(20, 20, 0.1f);
    fillRect(g, 7, 7, 13, 13, LOG_ODDS_FREE);

    FrontierExplorer ex(1.0f, 2.0f, 5.0f);
    auto clusters = ex.detect(g, 0.0f, 0.0f);  // robot at corner, >0.6m from frontier centroid
    EXPECT_FALSE(clusters.empty());
}

TEST(FrontierTest, ClusterBelowMinSizeDiscarded)
{
    OccupancyGrid g(20, 20, 0.1f);
    
    for (int i = 0; i < 5; ++i) {
        g.updateCell(1, 10, LOG_ODDS_FREE);
        g.updateCell(2, 10, LOG_ODDS_FREE);
    }
    FrontierExplorer ex(5.0f, 2.0f, 5.0f);   
    auto clusters = ex.detect(g, 0.0f, 0.0f);
    EXPECT_TRUE(clusters.empty());
}

TEST(FrontierTest, FullyExploredRoomNoFrontiers)
{
    
    OccupancyGrid g(10, 10, 0.1f);
    fillRect(g, 0, 0, 9, 9, LOG_ODDS_FREE);
    FrontierExplorer ex(1.0f, 2.0f, 5.0f);
    auto clusters = ex.detect(g, 0.5f, 0.5f);
    EXPECT_TRUE(clusters.empty());
}

TEST(FrontierTest, ClustersReturnedInScoreOrder)
{
    OccupancyGrid g(40, 40, 0.1f);
    
    fillRect(g, 1, 1, 4, 4, LOG_ODDS_FREE);   
    fillRect(g, 30, 30, 35, 35, LOG_ODDS_FREE); 

    FrontierExplorer ex(1.0f, 2.0f, 5.0f);
    auto clusters = ex.detect(g, 2.0f, 2.0f);  // grid centre, >0.6m from both cluster centroids

    ASSERT_GE(clusters.size(), 2u);
    
    EXPECT_LE(clusters[0].score, clusters[1].score);
}

TEST(FrontierTest, AntiRevisitPenaltyIncreasesScore)
{
    OccupancyGrid g(20, 20, 0.1f);
    fillRect(g, 2, 2, 6, 6, LOG_ODDS_FREE);

    FrontierExplorer ex(1.0f, 2.0f, 5.0f);

    auto before = ex.detect(g, 0.0f, 0.0f);
    ASSERT_FALSE(before.empty());
    float score_before = before[0].score;

    
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
