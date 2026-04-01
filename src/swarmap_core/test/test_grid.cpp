#include <gtest/gtest.h>
#include "swarmap_core/occupancy_grid.hpp"

using namespace swarmap;

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────
TEST(OccupancyGridTest, InitialisedUnknown)
{
    OccupancyGrid g(10, 10, 0.1f, 0.0f, 0.0f);
    for (int y = 0; y < 10; ++y)
        for (int x = 0; x < 10; ++x)
            EXPECT_EQ(g.getCellRos(x, y), CELL_UNKNOWN);
}

TEST(OccupancyGridTest, DimensionsCorrect)
{
    OccupancyGrid g(20, 15, 0.1f);
    EXPECT_EQ(g.width(),  20);
    EXPECT_EQ(g.height(), 15);
    EXPECT_FLOAT_EQ(g.resolution(), 0.1f);
}

// ─────────────────────────────────────────────────────────────────────────────
// Log-odds update
// ─────────────────────────────────────────────────────────────────────────────
TEST(OccupancyGridTest, FreeCellUpdate)
{
    OccupancyGrid g(10, 10, 0.1f);
    // Apply several free updates — cell should become FREE
    for (int i = 0; i < 5; ++i)
        g.updateCell(5, 5, LOG_ODDS_FREE);
    EXPECT_EQ(g.getCellRos(5, 5), CELL_FREE);
}

TEST(OccupancyGridTest, OccupiedCellUpdate)
{
    OccupancyGrid g(10, 10, 0.1f);
    for (int i = 0; i < 5; ++i)
        g.updateCell(3, 3, LOG_ODDS_OCCUPIED);
    EXPECT_EQ(g.getCellRos(3, 3), CELL_OCCUPIED);
}

TEST(OccupancyGridTest, LogOddsClampedAtMax)
{
    OccupancyGrid g(10, 10, 0.1f);
    // Apply many occupied updates — should not overflow
    for (int i = 0; i < 100; ++i)
        g.updateCell(0, 0, LOG_ODDS_OCCUPIED);
    EXPECT_EQ(g.getCellRos(0, 0), CELL_OCCUPIED);
    EXPECT_GE(g.getCellConfidence(0, 0), 0.9f);
}

// ─────────────────────────────────────────────────────────────────────────────
// World ↔ grid coordinate conversion
// ─────────────────────────────────────────────────────────────────────────────
TEST(OccupancyGridTest, WorldToGridRoundTrip)
{
    OccupancyGrid g(100, 100, 0.1f, -5.0f, -5.0f);
    int gx, gy;
    ASSERT_TRUE(g.worldToGrid(0.0f, 0.0f, gx, gy));
    float wx, wy;
    g.gridToWorld(gx, gy, wx, wy);
    EXPECT_NEAR(wx, 0.05f, 0.05f);   // cell centre tolerance
    EXPECT_NEAR(wy, 0.05f, 0.05f);
}

TEST(OccupancyGridTest, OutOfBoundsReturnsFalse)
{
    OccupancyGrid g(10, 10, 0.1f, 0.0f, 0.0f);
    int gx, gy;
    EXPECT_FALSE(g.worldToGrid(-1.0f, 0.0f, gx, gy));
    EXPECT_FALSE(g.worldToGrid(0.0f, 999.0f, gx, gy));
}

// ─────────────────────────────────────────────────────────────────────────────
// Dirty flags
// ─────────────────────────────────────────────────────────────────────────────
TEST(OccupancyGridTest, DirtyFlagSetOnUpdate)
{
    OccupancyGrid g(10, 10, 0.1f);
    g.updateCell(2, 2, LOG_ODDS_FREE);
    EXPECT_TRUE(g.isCellDirty(2, 2));
}

TEST(OccupancyGridTest, DirtyFlagClearedAfterClear)
{
    OccupancyGrid g(10, 10, 0.1f);
    g.updateCell(2, 2, LOG_ODDS_FREE);
    g.clearDirty(2, 2);
    EXPECT_FALSE(g.isCellDirty(2, 2));
}

TEST(OccupancyGridTest, MarkAllDirty)
{
    OccupancyGrid g(5, 5, 0.1f);
    g.markAllDirty();
    for (int y = 0; y < 5; ++y)
        for (int x = 0; x < 5; ++x)
            EXPECT_TRUE(g.isCellDirty(x, y));
}

// ─────────────────────────────────────────────────────────────────────────────
// Mapped cell count
// ─────────────────────────────────────────────────────────────────────────────
TEST(OccupancyGridTest, MappedCountStartsZero)
{
    OccupancyGrid g(10, 10, 0.1f);
    EXPECT_EQ(g.mappedCellCount(), 0);
}

TEST(OccupancyGridTest, MappedCountIncreasesOnUpdate)
{
    OccupancyGrid g(10, 10, 0.1f);
    g.updateCell(0, 0, LOG_ODDS_FREE);
    g.updateCell(1, 1, LOG_ODDS_OCCUPIED);
    EXPECT_EQ(g.mappedCellCount(), 2);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
