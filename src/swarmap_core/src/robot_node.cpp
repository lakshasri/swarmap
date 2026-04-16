#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "swarmap_msgs/msg/robot_status.hpp"
#include "swarmap_msgs/msg/neighbour_discovery.hpp"
#include "swarmap_msgs/msg/partial_map.hpp"
#include "swarmap_msgs/msg/frontier_bid.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "swarmap_core/occupancy_grid.hpp"
#include "swarmap_core/frontier_explorer.hpp"
#include "swarmap_core/map_merger.hpp"
#include "swarmap_core/neighbour_tracker.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// FIX #8: safe angle normalization (no while-loops)
static float normalizeAngle(float a) {
    return std::atan2(std::sin(a), std::cos(a));
}

namespace swarmap {

enum class RobotState { IDLE, EXPLORING, NAVIGATING, RETURNING, DOCKING, FAILED };

static std::string stateStr(RobotState s) {
    switch (s) {
        case RobotState::IDLE:       return "IDLE";
        case RobotState::EXPLORING:  return "EXPLORING";
        case RobotState::NAVIGATING: return "NAVIGATING";
        case RobotState::RETURNING:  return "RETURNING";
        case RobotState::DOCKING:    return "DOCKING";
        case RobotState::FAILED:     return "FAILED";
    }
    return "UNKNOWN";
}

// Navigation mode for Bug2-style algorithm
enum class NavMode { GOAL_SEEK, WALL_FOLLOW };

class RobotNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit RobotNode(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions())
        : rclcpp_lifecycle::LifecycleNode("robot_node", opts)
    {
        declare_parameter("robot_id",          "robot_0");
        declare_parameter("sensor_range",      5.0);
        declare_parameter("comm_radius",       8.0);
        declare_parameter("map_resolution",    0.1);
        declare_parameter("map_width_m",       50.0);
        declare_parameter("map_height_m",      50.0);
        declare_parameter("noise_level",       0.0);
        declare_parameter("battery_drain_rate",   0.001);
        declare_parameter("battery_weight",       2.0);
        declare_parameter("dock_return_safety",   0.85);
        declare_parameter("goal_tolerance",       0.3);
        declare_parameter("frontier_min_size",    5.0f);
        declare_parameter("update_rate_hz",       5.0);
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        robot_id_    = get_parameter("robot_id").as_string();
        sensor_range_= get_parameter("sensor_range").as_double();
        comm_radius_ = get_parameter("comm_radius").as_double();
        noise_level_ = get_parameter("noise_level").as_double();
        goal_tol_    = get_parameter("goal_tolerance").as_double();
        battery_     = 1.0f;
        state_       = RobotState::IDLE;
        pose_x_ = pose_y_ = pose_theta_ = 0.0;

        float res    = get_parameter("map_resolution").as_double();
        int   w_cells = static_cast<int>(std::floor(get_parameter("map_width_m").as_double()  / res));
        int   h_cells = static_cast<int>(std::floor(get_parameter("map_height_m").as_double() / res));
        grid_ = std::make_unique<OccupancyGrid>(w_cells, h_cells, res, 0.0f, 0.0f);
        grid_->markAllDirty();

        // 5 docks: center + 4 quadrants
        float mw = static_cast<float>(get_parameter("map_width_m").as_double());
        float mh = static_cast<float>(get_parameter("map_height_m").as_double());
        docks_ = {
            {mw * 0.50f, mh * 0.50f},
            {mw * 0.25f, mh * 0.25f},
            {mw * 0.75f, mh * 0.25f},
            {mw * 0.25f, mh * 0.75f},
            {mw * 0.75f, mh * 0.75f},
        };

        explorer_ = std::make_unique<FrontierExplorer>(
            get_parameter("frontier_min_size").as_double(),
            3.0f, 6.0f,  // sigma=3m, weight=6 — moderate revisit penalty
            static_cast<float>(get_parameter("battery_weight").as_double()));
        merger_   = std::make_unique<MapMerger>();
        tracker_  = std::make_unique<NeighbourTracker>();
        tf_buffer_= std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto qos_reliable = rclcpp::QoS(10).reliable();
        auto qos_best     = rclcpp::QoS(10).best_effort();
        std::string ns = "/" + robot_id_;

        pub_map_    = create_publisher<swarmap_msgs::msg::PartialMap>(ns + "/map", qos_reliable);
        pub_status_ = create_publisher<swarmap_msgs::msg::RobotStatus>(ns + "/status", qos_reliable);
        pub_cmd_    = create_publisher<geometry_msgs::msg::Twist>(ns + "/cmd_vel", qos_best);
        pub_disc_   = create_publisher<swarmap_msgs::msg::NeighbourDiscovery>("/swarm/discovery", qos_reliable);
        pub_bid_    = create_publisher<swarmap_msgs::msg::FrontierBid>(ns + "/frontier_bid", qos_reliable);
        pub_acc_    = create_publisher<std_msgs::msg::Float32>(ns + "/map_accuracy", qos_reliable);

        sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
            ns + "/scan", qos_best,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg){ onScan(msg); });
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            ns + "/odom", qos_best,
            [this](nav_msgs::msg::Odometry::SharedPtr msg){ onOdom(msg); });
        sub_disc_ = create_subscription<swarmap_msgs::msg::NeighbourDiscovery>(
            "/swarm/discovery", qos_reliable,
            [this](swarmap_msgs::msg::NeighbourDiscovery::SharedPtr msg){ onDiscovery(msg); });

        sub_reset_ = create_subscription<std_msgs::msg::String>(
            "/swarm/reset_map", rclcpp::QoS(10).reliable(),
            [this](std_msgs::msg::String::SharedPtr){
                std::unique_lock lock(grid_->mutex);
                grid_ = std::make_unique<OccupancyGrid>(
                    grid_->width(), grid_->height(), grid_->resolution(), 0.0f, 0.0f);
                grid_->markAllDirty();
                explorer_->clearVisited();
                has_goal_ = false;
                nav_mode_ = NavMode::GOAL_SEEK;
                state_ = RobotState::EXPLORING;
                RCLCPP_INFO(get_logger(), "[%s] Map RESET", robot_id_.c_str());
            });

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(get_logger(), "[%s] Configured", robot_id_.c_str());
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        double rate = get_parameter("update_rate_hz").as_double();
        timer_map_    = create_wall_timer(500ms,  [this](){ publishMap(); });
        timer_status_ = create_wall_timer(1000ms, [this](){ publishStatus(); });
        timer_disc_   = create_wall_timer(500ms,  [this](){ publishDiscovery(); });
        timer_explore_= create_wall_timer(std::chrono::milliseconds(
                             static_cast<int>(1000.0 / rate)),
                             [this](){ explorationTick(); });
        timer_battery_= create_wall_timer(1000ms, [this](){ drainBattery(); });
        RCLCPP_INFO(get_logger(), "[%s] Active", robot_id_.c_str());
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        timer_map_.reset(); timer_status_.reset(); timer_disc_.reset();
        timer_explore_.reset(); timer_battery_.reset();
        stop();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
    {
        grid_.reset(); explorer_.reset(); merger_.reset(); tracker_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
    {
        stop();
        return CallbackReturn::SUCCESS;
    }

private:
    std::string robot_id_;
    RobotState  state_ = RobotState::IDLE;

    double pose_x_ = 0.0, pose_y_ = 0.0, pose_theta_ = 0.0;
    float  battery_ = 1.0f;
    double sensor_range_ = 5.0;
    double comm_radius_  = 8.0;
    double noise_level_  = 0.0;
    float  goal_tol_     = 0.3f;  // FIX #11: cached parameter

    float  goal_x_ = 0.0f, goal_y_ = 0.0f;
    bool   has_goal_ = false;
    bool   returning_to_dock_ = false;

    // Bug2 navigation state
    NavMode nav_mode_ = NavMode::GOAL_SEEK;
    float   hit_x_ = 0.0f, hit_y_ = 0.0f;  // where we first hit the wall
    int     wall_follow_steps_ = 0;
    static constexpr int MAX_WALL_FOLLOW = 250;  // max steps (~50s at 5Hz) before giving up

    // Stuck detection
    double pose_x_last_ = 0.0, pose_y_last_ = 0.0;
    double last_moved_time_ = 0.0;
    static constexpr double STUCK_TIMEOUT_S  = 8.0;
    // FIX #10: use cumulative distance, not instantaneous
    double cumulative_dist_ = 0.0;

    // Scan summary for obstacle avoidance
    float nearest_front_ = 999.0f;
    float nearest_left_  = 999.0f;
    float nearest_right_ = 999.0f;

    // Dock system
    struct Dock { float x; float y; };
    std::vector<Dock> docks_;
    double dock_arrive_time_ = 0.0;
    static constexpr double DOCK_RECHARGE_S = 7.0;

    std::unique_ptr<OccupancyGrid>    grid_;
    std::unique_ptr<FrontierExplorer> explorer_;
    std::unique_ptr<MapMerger>        merger_;
    std::unique_ptr<NeighbourTracker> tracker_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer>               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;

    rclcpp::Publisher<swarmap_msgs::msg::PartialMap>::SharedPtr      pub_map_;
    rclcpp::Publisher<swarmap_msgs::msg::RobotStatus>::SharedPtr     pub_status_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr          pub_cmd_;
    rclcpp::Publisher<swarmap_msgs::msg::NeighbourDiscovery>::SharedPtr pub_disc_;
    rclcpp::Publisher<swarmap_msgs::msg::FrontierBid>::SharedPtr     pub_bid_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             pub_acc_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    sub_scan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        sub_odom_;
    rclcpp::Subscription<swarmap_msgs::msg::NeighbourDiscovery>::SharedPtr sub_disc_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr          sub_reset_;

    // FIX #9/#16: track subscriptions and avoid duplicates
    std::unordered_map<std::string,
        rclcpp::Subscription<swarmap_msgs::msg::PartialMap>::SharedPtr>  neighbour_map_subs_;
    std::unordered_map<std::string,
        rclcpp::Subscription<swarmap_msgs::msg::FrontierBid>::SharedPtr> neighbour_bid_subs_;

    rclcpp::TimerBase::SharedPtr timer_map_, timer_status_, timer_disc_,
                                  timer_explore_, timer_battery_;

    std::default_random_engine rng_{std::random_device{}()};

    // ── Scan processing ──────────────────────────────────────────────
    void onScan(const sensor_msgs::msg::LaserScan::SharedPtr &msg)
    {
        if (state_ == RobotState::FAILED) return;

        // Summarise scan for obstacle avoidance
        float nf = msg->range_max, nl = msg->range_max, nr = msg->range_max;
        float angle = msg->angle_min;
        for (float raw_range : msg->ranges) {
            float r = std::clamp(raw_range, msg->range_min, msg->range_max);
            if (std::isnan(r) || std::isinf(r)) { angle += msg->angle_increment; continue; }
            float a = normalizeAngle(angle);
            float aa = std::abs(a);
            if (aa < 0.79f)               nf = std::min(nf, r);
            else if (a > 0 && aa < 1.57f) nl = std::min(nl, r);
            else if (a < 0 && aa < 1.57f) nr = std::min(nr, r);
            angle += msg->angle_increment;
        }
        nearest_front_ = nf;
        nearest_left_  = nl;
        nearest_right_ = nr;

        // FIX #4: copy scan data, lock briefly for grid update
        struct Ray { float ex, ey; bool hit; };
        std::vector<Ray> rays;
        rays.reserve(msg->ranges.size());

        std::normal_distribution<float> noise_dist(0.0f,
            static_cast<float>(noise_level_ * sensor_range_));
        float a2 = msg->angle_min;
        float px = static_cast<float>(pose_x_);
        float py = static_cast<float>(pose_y_);
        float pth = static_cast<float>(pose_theta_);

        for (float raw_range : msg->ranges) {
            float r = raw_range;
            if (noise_level_ > 0.0) r += noise_dist(rng_);
            r = std::clamp(r, msg->range_min, msg->range_max);
            float ex = px + r * std::cos(pth + a2);
            float ey = py + r * std::sin(pth + a2);
            rays.push_back({ex, ey, r < msg->range_max - 0.05f});
            a2 += msg->angle_increment;
        }

        // Now lock and update grid
        std::unique_lock lock(grid_->mutex);
        for (auto &ray : rays) {
            rayCast(px, py, ray.ex, ray.ey, ray.hit);
        }
    }

    void rayCast(float sx, float sy, float ex, float ey, bool mark_endpoint)
    {
        int gx0, gy0, gx1, gy1;
        if (!grid_->worldToGrid(sx, sy, gx0, gy0)) return;
        if (!grid_->worldToGrid(ex, ey, gx1, gy1)) return;

        int dx = std::abs(gx1 - gx0), sx_step = (gx0 < gx1) ? 1 : -1;
        int dy = std::abs(gy1 - gy0), sy_step = (gy0 < gy1) ? 1 : -1;
        int err = dx - dy;
        int cx = gx0, cy = gy0;

        for (int step = 0; step < dx + dy + 2; ++step) {
            if (cx == gx1 && cy == gy1) {
                grid_->updateCell(cx, cy, mark_endpoint ? LOG_ODDS_OCCUPIED : LOG_ODDS_FREE);
                break;
            }
            grid_->updateCell(cx, cy, LOG_ODDS_FREE);
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; cx += sx_step; }
            if (e2 <  dx) { err += dx; cy += sy_step; }
        }
    }

    // ── Odometry ─────────────────────────────────────────────────────
    void onOdom(const nav_msgs::msg::Odometry::SharedPtr &msg)
    {
        double old_x = pose_x_, old_y = pose_y_;
        pose_x_ = msg->pose.pose.position.x;
        pose_y_ = msg->pose.pose.position.y;
        auto &q = msg->pose.pose.orientation;
        pose_theta_ = std::atan2(2.0*(q.w*q.z + q.x*q.y),
                                 1.0 - 2.0*(q.y*q.y + q.z*q.z));
        broadcastTF(msg->header.stamp);

        // FIX #10: cumulative distance for stuck detection
        double now = get_clock()->now().seconds();
        cumulative_dist_ += std::hypot(pose_x_ - old_x, pose_y_ - old_y);
        if (cumulative_dist_ > 0.3) {  // moved 0.3m total
            cumulative_dist_ = 0.0;
            last_moved_time_ = now;
        }
        if (last_moved_time_ == 0.0) last_moved_time_ = now;
    }

    // ── Discovery / neighbor comms ───────────────────────────────────
    void onDiscovery(const swarmap_msgs::msg::NeighbourDiscovery::SharedPtr &msg)
    {
        if (msg->robot_id == robot_id_) return;
        double now = get_clock()->now().seconds();
        tracker_->updateNeighbour(msg->robot_id,
                                   msg->position.x, msg->position.y,
                                   msg->comm_radius, now);

        // FIX #9: use try_emplace to avoid overwriting existing subscription
        if (neighbour_map_subs_.find(msg->robot_id) == neighbour_map_subs_.end()) {
            const std::string ns = "/" + msg->robot_id;
            auto qos = rclcpp::QoS(5).reliable();
            neighbour_map_subs_.try_emplace(msg->robot_id,
                create_subscription<swarmap_msgs::msg::PartialMap>(
                    ns + "/map", qos,
                    [this](swarmap_msgs::msg::PartialMap::SharedPtr m){ onNeighbourMap(m); }));
            neighbour_bid_subs_.try_emplace(msg->robot_id,
                create_subscription<swarmap_msgs::msg::FrontierBid>(
                    ns + "/frontier_bid", qos,
                    [this](swarmap_msgs::msg::FrontierBid::SharedPtr m){ onNeighbourBid(m); }));
            RCLCPP_INFO(get_logger(), "[%s] Subscribed to neighbour %s",
                        robot_id_.c_str(), msg->robot_id.c_str());
        }
    }

    void onNeighbourMap(const swarmap_msgs::msg::PartialMap::SharedPtr &msg)
    {
        if (state_ == RobotState::FAILED) return;
        // FIX #13: validate incoming data size
        if (static_cast<int>(msg->data.size()) != msg->width * msg->height) return;
        std::unique_lock lock(grid_->mutex);
        merger_->merge(*grid_, *msg, 0, 0);
    }

    void onNeighbourBid(const swarmap_msgs::msg::FrontierBid::SharedPtr &msg)
    {
        if (msg->robot_id == robot_id_) return;
        explorer_->recordBid(*msg);
    }

    // ── TF / Status / Discovery publish ──────────────────────────────
    void broadcastTF(const rclcpp::Time &stamp)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = robot_id_ + "/odom";
        t.child_frame_id  = robot_id_ + "/base_link";
        t.transform.translation.x = pose_x_;
        t.transform.translation.y = pose_y_;
        tf2::Quaternion q; q.setRPY(0, 0, pose_theta_);
        t.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(t);
    }

    void publishMap()
    {
        if (state_ == RobotState::FAILED) return;
        std::shared_lock lock(grid_->mutex);
        swarmap_msgs::msg::PartialMap msg;
        msg.header.stamp = get_clock()->now();
        msg.header.frame_id = "map";
        msg.robot_id = robot_id_;
        msg.resolution = grid_->resolution();
        msg.origin_x = 0; msg.origin_y = 0;
        msg.width = grid_->width(); msg.height = grid_->height();
        int total = grid_->width() * grid_->height();
        msg.data.reserve(total); msg.confidence.reserve(total);
        for (int gy = 0; gy < grid_->height(); ++gy)
            for (int gx = 0; gx < grid_->width(); ++gx) {
                msg.data.push_back(grid_->getCellRos(gx, gy));
                msg.confidence.push_back(grid_->getCellConfidence(gx, gy));
                grid_->clearDirty(gx, gy);
            }
        pub_map_->publish(msg);
    }

    void publishStatus()
    {
        if (state_ == RobotState::FAILED) return;
        double now = get_clock()->now().seconds();
        auto neighbours = tracker_->getNeighbours(
            static_cast<float>(pose_x_), static_cast<float>(pose_y_),
            static_cast<float>(comm_radius_), now);
        swarmap_msgs::msg::RobotStatus msg;
        msg.header.stamp = get_clock()->now();
        msg.header.frame_id = "map";
        msg.robot_id = robot_id_;
        msg.pose.position.x = pose_x_;
        msg.pose.position.y = pose_y_;
        msg.battery_level = battery_;
        msg.is_active = (state_ != RobotState::FAILED);
        msg.current_state = stateStr(state_);
        msg.cells_mapped = grid_->mappedCellCount();
        msg.neighbour_ids = neighbours;
        pub_status_->publish(msg);
    }

    void publishDiscovery()
    {
        swarmap_msgs::msg::NeighbourDiscovery msg;
        msg.header.stamp = get_clock()->now();
        msg.header.frame_id = "map";
        msg.robot_id = robot_id_;
        msg.position.x = pose_x_;
        msg.position.y = pose_y_;
        msg.comm_radius = static_cast<float>(comm_radius_);
        pub_disc_->publish(msg);
    }

    // ── Dock helpers ─────────────────────────────────────────────────
    Dock nearestDock() const {
        Dock best = docks_.empty() ? Dock{25.0f, 25.0f} : docks_[0];
        float bestD = 1e9f;
        for (auto &d : docks_) {
            float dd = std::hypot(d.x - static_cast<float>(pose_x_),
                                  d.y - static_cast<float>(pose_y_));
            if (dd < bestD) { bestD = dd; best = d; }
        }
        return best;
    }

    // ── Exploration state machine ────────────────────────────────────
    void explorationTick()
    {
        if (state_ == RobotState::FAILED) return;

        if (state_ == RobotState::DOCKING) {
            double now = get_clock()->now().seconds();
            if (dock_arrive_time_ > 0 && (now - dock_arrive_time_) >= DOCK_RECHARGE_S) {
                battery_ = 1.0f;
                dock_arrive_time_ = 0.0;
                returning_to_dock_ = false;
                state_ = RobotState::EXPLORING;
                RCLCPP_INFO(get_logger(), "[%s] Battery recharged — resuming", robot_id_.c_str());
            }
            return;
        }

        if (state_ == RobotState::NAVIGATING) {
            driveTowardGoal();
            return;
        }

        if (state_ == RobotState::RETURNING) {
            auto dock = nearestDock();
            goal_x_ = dock.x; goal_y_ = dock.y;
            has_goal_ = true;
            returning_to_dock_ = true;
            nav_mode_ = NavMode::GOAL_SEEK;
            wall_follow_steps_ = 0;
            state_ = RobotState::NAVIGATING;
            return;
        }

        // IDLE / EXPLORING: detect frontiers
        std::shared_lock lock(grid_->mutex);
        auto clusters = explorer_->detect(*grid_,
                                           static_cast<float>(pose_x_),
                                           static_cast<float>(pose_y_),
                                           battery_);
        lock.unlock();

        if (clusters.empty()) {
            stop();
            state_ = RobotState::IDLE;
            return;
        }

        double now = get_clock()->now().seconds();
        explorer_->expireBids(now);

        for (auto &cl : clusters) {
            if (explorer_->winsAuction(robot_id_, cl.score,
                                        cl.centroid_wx, cl.centroid_wy)) {
                goal_x_ = cl.centroid_wx;
                goal_y_ = cl.centroid_wy;
                has_goal_ = true;
                nav_mode_ = NavMode::GOAL_SEEK;
                wall_follow_steps_ = 0;
                state_ = RobotState::NAVIGATING;

                swarmap_msgs::msg::FrontierBid bid;
                bid.header.stamp = get_clock()->now();
                bid.robot_id = robot_id_;
                bid.frontier_centroid.x = cl.centroid_wx;
                bid.frontier_centroid.y = cl.centroid_wy;
                bid.bid_score = cl.score;
                bid.battery_level = battery_;
                bid.claim = true;
                pub_bid_->publish(bid);
                return;
            }
        }
        state_ = RobotState::EXPLORING;
    }

    // ── Bug2-style navigation ────────────────────────────────────────
    void driveTowardGoal()
    {
        if (!has_goal_) { state_ = RobotState::EXPLORING; return; }

        float dx = goal_x_ - static_cast<float>(pose_x_);
        float dy = goal_y_ - static_cast<float>(pose_y_);
        float dist = std::hypot(dx, dy);

        // Check if we reached the goal
        if (dist < goal_tol_) {
            stop();
            has_goal_ = false;
            nav_mode_ = NavMode::GOAL_SEEK;
            wall_follow_steps_ = 0;

            // Check dock arrival
            bool at_dock = false;
            for (auto &d : docks_) {
                if (std::hypot(d.x - static_cast<float>(pose_x_),
                               d.y - static_cast<float>(pose_y_)) < goal_tol_ * 3.0f) {
                    at_dock = true; break;
                }
            }
            if (at_dock && returning_to_dock_) {
                dock_arrive_time_ = get_clock()->now().seconds();
                state_ = RobotState::DOCKING;
                RCLCPP_INFO(get_logger(), "[%s] Docked — recharging (%.0f%%)",
                            robot_id_.c_str(), battery_ * 100.0f);
                return;
            }

            explorer_->markVisited(goal_x_, goal_y_);
            returning_to_dock_ = false;
            state_ = RobotState::EXPLORING;

            swarmap_msgs::msg::FrontierBid release;
            release.header.stamp = get_clock()->now();
            release.robot_id = robot_id_;
            release.claim = false;
            pub_bid_->publish(release);
            return;
        }

        // Stuck detection — disabled during wall-follow (robot is actively maneuvering)
        double now = get_clock()->now().seconds();
        if (nav_mode_ == NavMode::GOAL_SEEK &&
            last_moved_time_ > 0.0 && (now - last_moved_time_) > STUCK_TIMEOUT_S) {
            RCLCPP_WARN(get_logger(), "[%s] Stuck — abandoning goal (%.1f, %.1f)",
                        robot_id_.c_str(), goal_x_, goal_y_);
            stop();
            explorer_->markVisited(goal_x_, goal_y_);
            has_goal_ = false;
            nav_mode_ = NavMode::GOAL_SEEK;
            wall_follow_steps_ = 0;
            last_moved_time_ = now;
            cumulative_dist_ = 0.0;
            state_ = returning_to_dock_ ? RobotState::RETURNING : RobotState::EXPLORING;
            return;
        }

        // Bug2 navigation
        constexpr float WALL_THRESH = 1.2f;
        geometry_msgs::msg::Twist cmd;

        if (nav_mode_ == NavMode::GOAL_SEEK) {
            // Drive toward goal
            float bearing = std::atan2(dy, dx);
            float angle_err = normalizeAngle(bearing - static_cast<float>(pose_theta_));

            if (nearest_front_ < WALL_THRESH) {
                // Hit a wall — switch to wall following
                nav_mode_ = NavMode::WALL_FOLLOW;
                hit_x_ = static_cast<float>(pose_x_);
                hit_y_ = static_cast<float>(pose_y_);
                wall_follow_steps_ = 0;
                RCLCPP_DEBUG(get_logger(), "[%s] Wall hit — following", robot_id_.c_str());
            } else {
                // Clear ahead — drive toward goal
                cmd.angular.z = std::clamp(2.5f * angle_err, -1.5f, 1.5f);
                cmd.linear.x  = std::clamp(0.6f * dist, 0.0f, 0.7f)
                                * std::max(0.15f, 1.0f - std::abs(angle_err) / 2.0f);
                pub_cmd_->publish(cmd);
                return;
            }
        }

        if (nav_mode_ == NavMode::WALL_FOLLOW) {
            ++wall_follow_steps_;

            float dist_from_hit = std::hypot(
                static_cast<float>(pose_x_) - hit_x_,
                static_cast<float>(pose_y_) - hit_y_);
            bool cleared_hit = dist_from_hit > 2.0f;
            bool front_clear = nearest_front_ > WALL_THRESH * 1.5f;

            if ((cleared_hit && front_clear) || wall_follow_steps_ > MAX_WALL_FOLLOW) {
                nav_mode_ = NavMode::GOAL_SEEK;
                wall_follow_steps_ = 0;
            } else {
                if (nearest_front_ < 0.5f) {
                    cmd.linear.x  = -0.2;
                    cmd.angular.z = 1.5;
                } else if (nearest_front_ < 1.0f) {
                    cmd.linear.x  = 0.2;
                    cmd.angular.z = 1.0;
                } else {
                    cmd.linear.x = 0.45;
                    float desired_wall = 1.0f;
                    if (nearest_right_ < desired_wall * 0.6f)
                        cmd.angular.z = 0.6f;
                    else if (nearest_right_ > desired_wall * 1.8f)
                        cmd.angular.z = -0.5f;
                    else
                        cmd.angular.z = -0.1f;
                }
                pub_cmd_->publish(cmd);
                return;
            }
        }

        // Fallback: goal-seek command
        float bearing = std::atan2(dy, dx);
        float angle_err = normalizeAngle(bearing - static_cast<float>(pose_theta_));
        cmd.angular.z = std::clamp(2.5f * angle_err, -1.5f, 1.5f);
        cmd.linear.x  = std::clamp(0.6f * dist, 0.0f, 0.7f)
                        * std::max(0.15f, 1.0f - std::abs(angle_err) / 2.0f);
        pub_cmd_->publish(cmd);
    }

    void stop()
    {
        geometry_msgs::msg::Twist zero;
        pub_cmd_->publish(zero);
    }

    // ── Battery management ───────────────────────────────────────────
    void drainBattery()
    {
        if (state_ == RobotState::DOCKING) return;

        float drain = static_cast<float>(get_parameter("battery_drain_rate").as_double());
        if (drain <= 0.0f) return;  // FIX #18: guard against zero drain
        battery_ = std::max(0.0f, battery_ - drain);

        if (state_ == RobotState::FAILED || state_ == RobotState::RETURNING
            || returning_to_dock_) return;

        auto dock = nearestDock();
        float dock_dist = std::hypot(
            static_cast<float>(pose_x_) - dock.x,
            static_cast<float>(pose_y_) - dock.y);
        constexpr float AVG_SPEED = 0.25f;
        float time_to_return = dock_dist / AVG_SPEED;
        float time_to_empty  = battery_ / drain;
        float safety = static_cast<float>(get_parameter("dock_return_safety").as_double());

        bool predict_stranded = (battery_ < 0.9f) &&
                                (time_to_return > safety * time_to_empty);
        bool critically_low   = (battery_ < 0.05f);

        if (predict_stranded || critically_low) {
            state_ = RobotState::RETURNING;
            RCLCPP_WARN(get_logger(),
                "[%s] Returning — battery %.0f%%, %.1fs to dock, %.1fs of charge left",
                robot_id_.c_str(), battery_ * 100.0f, time_to_return, time_to_empty);
        }
    }
};

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<swarmap::RobotNode>();
    using Transition = lifecycle_msgs::msg::Transition;
    node->trigger_transition(Transition::TRANSITION_CONFIGURE);
    node->trigger_transition(Transition::TRANSITION_ACTIVATE);
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
