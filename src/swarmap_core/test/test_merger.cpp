#include <gtest/gtest.h>
#include "swarmap_core/map_merger.hpp"
#include "swarmap_core/occupancy_grid.hpp"
#include "swarmap_msgs/msg/partial_map.hpp"

using namespace swarmap;

static swarmap_msgs::msg::PartialMap makePartial(
    int w, int h, int8_t val, float conf,
    int origin_x = 0, int origin_y = 0, float res = 0.1f)
{
    swarmap_msgs::msg::PartialMap m;
    m.width    = w;
    m.height   = h;
    m.origin_x = origin_x;
    m.origin_y = origin_y;
    m.resolution = res;
    m.data.assign(w * h, val);
    m.confidence.assign(w * h, conf);
    return m;
}

TEST(MapMergerTest, UnknownLocalAcceptsIncoming)
{
    OccupancyGrid local(10, 10, 0.1f);
    MapMerger merger;

    
    auto incoming = makePartial(10, 10, CELL_FREE, 0.8f);
    merger.merge(local, incoming, 0, 0);

    
    
    
    EXPECT_NE(local.getCellRos(3, 3), CELL_UNKNOWN);
}

TEST(MapMergerTest, IncomingUnknownSkipped)
{
    OccupancyGrid local(10, 10, 0.1f);
    
    for (int i = 0; i < 5; ++i) local.updateCell(5, 5, LOG_ODDS_FREE);

    MapMerger merger;
    
    auto incoming = makePartial(10, 10, CELL_UNKNOWN, 0.0f);
    merger.merge(local, incoming, 0, 0);

    
    EXPECT_EQ(local.getCellRos(5, 5), CELL_FREE);
}

TEST(MapMergerTest, TwoFreeGridsStayFree)
{
    OccupancyGrid local(10, 10, 0.1f);
    for (int i = 0; i < 5; ++i)
        for (int y = 0; y < 10; ++y)
            for (int x = 0; x < 10; ++x)
                local.updateCell(x, y, LOG_ODDS_FREE);

    MapMerger merger;
    auto incoming = makePartial(10, 10, CELL_FREE, 0.8f);
    merger.merge(local, incoming, 0, 0);

    for (int y = 0; y < 10; ++y)
        for (int x = 0; x < 10; ++x)
            EXPECT_EQ(local.getCellRos(x, y), CELL_FREE);
}

TEST(MapMergerTest, TwoOccupiedGridsStayOccupied)
{
    OccupancyGrid local(10, 10, 0.1f);
    for (int i = 0; i < 5; ++i)
        for (int y = 0; y < 10; ++y)
            for (int x = 0; x < 10; ++x)
                local.updateCell(x, y, LOG_ODDS_OCCUPIED);

    MapMerger merger;
    auto incoming = makePartial(10, 10, CELL_OCCUPIED, 0.8f);
    merger.merge(local, incoming, 0, 0);

    for (int y = 0; y < 10; ++y)
        for (int x = 0; x < 10; ++x)
            EXPECT_EQ(local.getCellRos(x, y), CELL_OCCUPIED);
}

TEST(MapMergerTest, OffsetAppliedCorrectly)
{
    OccupancyGrid local(20, 20, 0.1f);
    MapMerger merger;

    
    
    
    
    auto incoming = makePartial(5, 5, CELL_FREE, 0.8f, 2, 2);
    merger.merge(local, incoming, 5, 5);

    
    EXPECT_NE(local.getCellRos(7, 7), CELL_UNKNOWN);
    
    EXPECT_NE(local.getCellRos(11, 11), CELL_UNKNOWN);
    
    EXPECT_EQ(local.getCellRos(0, 0), CELL_UNKNOWN);
}

TEST(MapMergerTest, AccuracyPerfectMatch)
{
    OccupancyGrid local(10, 10, 0.1f), gt(10, 10, 0.1f);
    for (int i = 0; i < 5; ++i)
        for (int y = 0; y < 10; ++y)
            for (int x = 0; x < 10; ++x) {
                local.updateCell(x, y, LOG_ODDS_FREE);
                gt.updateCell(x, y, LOG_ODDS_FREE);
            }
    EXPECT_FLOAT_EQ(MapMerger::accuracy(local, gt), 1.0f);
}

TEST(MapMergerTest, AccuracyZeroWhenNothingMapped)
{
    OccupancyGrid local(10, 10, 0.1f), gt(10, 10, 0.1f);
    for (int i = 0; i < 5; ++i)
        for (int y = 0; y < 10; ++y)
            for (int x = 0; x < 10; ++x)
                gt.updateCell(x, y, LOG_ODDS_FREE);
    EXPECT_FLOAT_EQ(MapMerger::accuracy(local, gt), 0.0f);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
