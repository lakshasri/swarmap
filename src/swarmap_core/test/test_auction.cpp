#include <gtest/gtest.h>
#include "swarmap_core/frontier_explorer.hpp"
#include "swarmap_msgs/msg/frontier_bid.hpp"

using namespace swarmap;

// Helper — build a FrontierBid message
static swarmap_msgs::msg::FrontierBid makeBid(
    const std::string &robot_id, float score,
    float cx, float cy, bool claim = true, double stamp_s = 100.0)
{
    swarmap_msgs::msg::FrontierBid b;
    b.robot_id = robot_id;
    b.bid_score = score;
    b.frontier_centroid.x = cx;
    b.frontier_centroid.y = cy;
    b.claim = claim;
    b.header.stamp.sec     = static_cast<int32_t>(stamp_s);
    b.header.stamp.nanosec = 0;
    return b;
}

// ─────────────────────────────────────────────────────────────────────────────
// Single robot — always wins
// ─────────────────────────────────────────────────────────────────────────────
TEST(AuctionTest, SingleRobotAlwaysWins)
{
    FrontierExplorer ex(1.0f, 2.0f, 5.0f);
    EXPECT_TRUE(ex.winsAuction("robot_0", 3.5f, 1.0f, 1.0f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Lower score wins
// ─────────────────────────────────────────────────────────────────────────────
TEST(AuctionTest, LowerScoreWins)
{
    FrontierExplorer ex(1.0f, 2.0f, 5.0f);
    // Neighbour has higher score → we win
    ex.recordBid(makeBid("robot_1", 5.0f, 1.0f, 1.0f));
    EXPECT_TRUE(ex.winsAuction("robot_0", 3.0f, 1.0f, 1.0f));
}

TEST(AuctionTest, HigherScoreLoses)
{
    FrontierExplorer ex(1.0f, 2.0f, 5.0f);
    // Neighbour has lower score → we lose
    ex.recordBid(makeBid("robot_1", 2.0f, 1.0f, 1.0f));
    EXPECT_FALSE(ex.winsAuction("robot_0", 3.0f, 1.0f, 1.0f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Tie-breaking by robot ID (lexicographic)
// ─────────────────────────────────────────────────────────────────────────────
TEST(AuctionTest, TieBreakHigherIdLoses)
{
    FrontierExplorer ex(1.0f, 2.0f, 5.0f);
    // "robot_1" < "robot_9" lexicographically, so robot_9 loses a tie
    ex.recordBid(makeBid("robot_1", 3.0f, 1.0f, 1.0f));
    EXPECT_FALSE(ex.winsAuction("robot_9", 3.0f, 1.0f, 1.0f));
}

TEST(AuctionTest, TieBreakLowerIdWins)
{
    FrontierExplorer ex(1.0f, 2.0f, 5.0f);
    ex.recordBid(makeBid("robot_5", 3.0f, 1.0f, 1.0f));
    EXPECT_TRUE(ex.winsAuction("robot_1", 3.0f, 1.0f, 1.0f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Bid targeting a different frontier is ignored
// ─────────────────────────────────────────────────────────────────────────────
TEST(AuctionTest, BidForDifferentFrontierIgnored)
{
    FrontierExplorer ex(1.0f, 2.0f, 5.0f);
    // Neighbour bids on a frontier far away
    ex.recordBid(makeBid("robot_1", 1.0f, 20.0f, 20.0f));
    // We bid on a nearby frontier — should still win
    EXPECT_TRUE(ex.winsAuction("robot_0", 3.0f, 1.0f, 1.0f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Released bid (claim=false) removes competitor
// ─────────────────────────────────────────────────────────────────────────────
TEST(AuctionTest, ReleasedBidAllowsWin)
{
    FrontierExplorer ex(1.0f, 2.0f, 5.0f);
    ex.recordBid(makeBid("robot_1", 1.0f, 1.0f, 1.0f));  // robot_1 claims
    EXPECT_FALSE(ex.winsAuction("robot_0", 3.0f, 1.0f, 1.0f));

    ex.recordBid(makeBid("robot_1", 0.0f, 1.0f, 1.0f, /*claim=*/false));  // releases
    EXPECT_TRUE(ex.winsAuction("robot_0", 3.0f, 1.0f, 1.0f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Bid expiry
// ─────────────────────────────────────────────────────────────────────────────
TEST(AuctionTest, ExpiredBidNoLongerBlocks)
{
    FrontierExplorer ex(1.0f, 2.0f, 5.0f);
    ex.recordBid(makeBid("robot_1", 1.0f, 1.0f, 1.0f, true, 0.0));  // old bid at t=0
    ex.expireBids(10.0, 2.0);   // now = 10s, timeout = 2s → bid is stale
    EXPECT_TRUE(ex.winsAuction("robot_0", 3.0f, 1.0f, 1.0f));
}

TEST(AuctionTest, FreshBidStillBlocks)
{
    FrontierExplorer ex(1.0f, 2.0f, 5.0f);
    ex.recordBid(makeBid("robot_1", 1.0f, 1.0f, 1.0f, true, 9.5));  // recent bid
    ex.expireBids(10.0, 2.0);   // not yet expired
    EXPECT_FALSE(ex.winsAuction("robot_0", 3.0f, 1.0f, 1.0f));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
