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
#include "visualization_msgs/msg/marker.hpp"

#include "swarmap_core/occupancy_grid.hpp"
#include "swarmap_core/frontier_explorer.hpp"
#include "swarmap_core/map_merger.hpp"
#include "swarmap_core/neighbour_tracker.hpp"
#include "swarmap_core/path_planner.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

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
        declare_parameter("update_rate_hz",       10.0);
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
            3.0f, 6.0f,
            static_cast<float>(get_parameter("battery_weight").as_double()));
        merger_   = std::make_unique<MapMerger>();
        tracker_  = std::make_unique<NeighbourTracker>();
        planner_  = std::make_unique<PathPlanner>();
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
        pub_path_   = create_publisher<visualization_msgs::msg::Marker>(ns + "/path", qos_best);

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
                path_.clear(); path_idx_ = 0;
                has_goal_ = false;
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
        grid_.reset(); explorer_.reset(); merger_.reset(); tracker_.reset(); planner_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override { stop(); return CallbackReturn::SUCCESS; }

private:
    std::string robot_id_;
    RobotState  state_ = RobotState::IDLE;

    double pose_x_ = 0.0, pose_y_ = 0.0, pose_theta_ = 0.0;
    float  battery_ = 1.0f;
    double sensor_range_ = 5.0;
    double comm_radius_  = 8.0;
    double noise_level_  = 0.0;
    float  goal_tol_     = 0.3f;

    float  goal_x_ = 0.0f, goal_y_ = 0.0f;
    bool   has_goal_ = false;
    bool   returning_to_dock_ = false;

    // A* planned path (cell-level) + current waypoint index
    std::vector<std::pair<int,int>> path_;
    size_t path_idx_ = 0;
    int    replan_cooldown_ = 0;
    int    blocked_ticks_ = 0;   // consecutive ticks stopped by a wall

    float nearest_front_ = 999.0f;
    float nearest_left_  = 999.0f;
    float nearest_right_ = 999.0f;

    struct Dock { float x; float y; };
    std::vector<Dock> docks_;
    double dock_arrive_time_ = 0.0;
    static constexpr double DOCK_RECHARGE_S = 7.0;

    std::unique_ptr<OccupancyGrid>    grid_;
    std::unique_ptr<FrontierExplorer> explorer_;
    std::unique_ptr<MapMerger>        merger_;
    std::unique_ptr<NeighbourTracker> tracker_;
    std::unique_ptr<PathPlanner>      planner_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer>               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;

    rclcpp::Publisher<swarmap_msgs::msg::PartialMap>::SharedPtr      pub_map_;
    rclcpp::Publisher<swarmap_msgs::msg::RobotStatus>::SharedPtr     pub_status_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr          pub_cmd_;
    rclcpp::Publisher<swarmap_msgs::msg::NeighbourDiscovery>::SharedPtr pub_disc_;
    rclcpp::Publisher<swarmap_msgs::msg::FrontierBid>::SharedPtr     pub_bid_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             pub_acc_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr    pub_path_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    sub_scan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        sub_odom_;
    rclcpp::Subscription<swarmap_msgs::msg::NeighbourDiscovery>::SharedPtr sub_disc_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr          sub_reset_;

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

        float nf = msg->range_max;
        float nl = msg->range_max;
        float nr = msg->range_max;
        float angle = msg->angle_min;
        for (float raw_range : msg->ranges) {
            float r = std::clamp(raw_range, msg->range_min, msg->range_max);
            if (std::isnan(r) || std::isinf(r)) { angle += msg->angle_increment; continue; }
            float a = normalizeAngle(angle);
            float aa = std::abs(a);
            if (aa < 0.79f)               nf = std::min(nf, r);   // ±45° front
            else if (a > 0 && aa < 2.36f) nl = std::min(nl, r);   // left 45°-135°
            else if (a < 0 && aa < 2.36f) nr = std::min(nr, r);   // right 45°-135°
            angle += msg->angle_increment;
        }
        nearest_front_ = nf;
        nearest_left_  = nl;
        nearest_right_ = nr;

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

        std::unique_lock lock(grid_->mutex);
        for (auto &ray : rays) rayCast(px, py, ray.ex, ray.ey, ray.hit);
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

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr &msg)
    {
        pose_x_ = msg->pose.pose.position.x;
        pose_y_ = msg->pose.pose.position.y;
        auto &q = msg->pose.pose.orientation;
        pose_theta_ = std::atan2(2.0*(q.w*q.z + q.x*q.y),
                                 1.0 - 2.0*(q.y*q.y + q.z*q.z));
        broadcastTF(msg->header.stamp);
    }

    void onDiscovery(const swarmap_msgs::msg::NeighbourDiscovery::SharedPtr &msg)
    {
        if (msg->robot_id == robot_id_) return;
        double now = get_clock()->now().seconds();
        tracker_->updateNeighbour(msg->robot_id,
                                   msg->position.x, msg->position.y,
                                   msg->comm_radius, now);

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
        }
    }

    void onNeighbourMap(const swarmap_msgs::msg::PartialMap::SharedPtr &msg)
    {
        if (state_ == RobotState::FAILED) return;
        if (static_cast<int>(msg->data.size()) != msg->width * msg->height) return;
        std::unique_lock lock(grid_->mutex);
        merger_->merge(*grid_, *msg, 0, 0);
    }

    void onNeighbourBid(const swarmap_msgs::msg::FrontierBid::SharedPtr &msg)
    {
        if (msg->robot_id == robot_id_) return;
        explorer_->recordBid(*msg);
    }

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

    void publishPathMarker()
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = get_clock()->now();
        m.ns = robot_id_ + "_path";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = path_.empty() ? visualization_msgs::msg::Marker::DELETE
                                 : visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.1;
        m.color.r = 0.3; m.color.g = 0.9; m.color.b = 1.0; m.color.a = 0.9;
        geometry_msgs::msg::Point p0;
        p0.x = pose_x_; p0.y = pose_y_; p0.z = 0.1;
        m.points.push_back(p0);
        for (size_t i = path_idx_; i < path_.size(); ++i) {
            float wx, wy;
            grid_->gridToWorld(path_[i].first, path_[i].second, wx, wy);
            geometry_msgs::msg::Point p;
            p.x = wx; p.y = wy; p.z = 0.1;
            m.points.push_back(p);
        }
        pub_path_->publish(m);
    }

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

    // Plan A* from current pose to goal; store cells in path_
    bool replanPath()
    {
        int sgx, sgy, ggx, ggy;
        std::shared_lock lock(grid_->mutex);
        grid_->worldToGrid(pose_x_, pose_y_, sgx, sgy);
        grid_->worldToGrid(goal_x_, goal_y_, ggx, ggy);
        // stride=5 downsamples the 500x500 grid to an effective 100x100 for A*
        // clearance=2: keep 0.2 m buffer from walls so drifting during path
        // following doesn't push the robot into obstacles
        // stride=5: downsample 500x500 → effective 100x100 for fast A*
        auto p = planner_->plan(*grid_, sgx, sgy, ggx, ggy,
                                 /*unknown_as_free=*/true, /*clearance=*/2, /*stride=*/5);
        lock.unlock();
        if (p.empty()) {
            path_.clear();
            path_idx_ = 0;
            return false;
        }
        path_ = std::move(p);
        path_idx_ = 0;
        return true;
    }

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
            driveAlongPath();
            return;
        }

        if (state_ == RobotState::RETURNING) {
            auto dock = nearestDock();
            goal_x_ = dock.x; goal_y_ = dock.y;
            has_goal_ = true;
            returning_to_dock_ = true;
            if (replanPath()) {
                state_ = RobotState::NAVIGATING;
            } else {
                RCLCPP_WARN(get_logger(), "[%s] No path to dock — will retry", robot_id_.c_str());
            }
            return;
        }

        // EXPLORING / IDLE: pick a frontier
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
            if (!explorer_->winsAuction(robot_id_, cl.score, cl.centroid_wx, cl.centroid_wy)) continue;

            // Try to plan a path to this frontier. If A* fails, skip and try next.
            goal_x_ = cl.centroid_wx;
            goal_y_ = cl.centroid_wy;
            if (!replanPath()) continue;

            has_goal_ = true;
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
        state_ = RobotState::EXPLORING;
    }

    // ── Path following ──────────────────────────────────────────────
    void driveAlongPath()
    {
        if (!has_goal_ || path_.empty() || path_idx_ >= path_.size()) {
            // Reached end of planned path
            stop();
            has_goal_ = false;
            publishPathMarker();

            // Dock arrival?
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

        // Check if the next few waypoints are still free; replan if a newly
        // discovered obstacle is in the way (only every few ticks to save CPU).
        if (replan_cooldown_ > 0) --replan_cooldown_;
        if (replan_cooldown_ == 0) {
            std::shared_lock lock(grid_->mutex);
            for (size_t i = path_idx_; i < std::min(path_idx_ + 6, path_.size()); ++i) {
                if (grid_->getCellRos(path_[i].first, path_[i].second) == CELL_OCCUPIED) {
                    lock.unlock();
                    if (!replanPath()) {
                        // Can't get there — give up on this goal
                        RCLCPP_WARN(get_logger(), "[%s] Path blocked, no replan — abandoning goal",
                                    robot_id_.c_str());
                        explorer_->markVisited(goal_x_, goal_y_);
                        has_goal_ = false;
                        path_.clear(); path_idx_ = 0;
                        state_ = returning_to_dock_ ? RobotState::RETURNING : RobotState::EXPLORING;
                        return;
                    }
                    replan_cooldown_ = 5;
                    break;
                }
            }
            if (replan_cooldown_ == 0) replan_cooldown_ = 3;  // don't check every tick
        }

        // Skip past any waypoints the robot has already passed (keeps motion
        // smooth even when the robot overshoots a cell slightly)
        while (path_idx_ + 1 < path_.size()) {
            auto [nx, ny] = path_[path_idx_];
            float wx, wy;
            grid_->gridToWorld(nx, ny, wx, wy);
            float dx = wx - static_cast<float>(pose_x_);
            float dy = wy - static_cast<float>(pose_y_);
            if (std::hypot(dx, dy) < 0.6f) {
                ++path_idx_;
            } else {
                break;
            }
        }

        // Lookahead: aim at the waypoint ~1m ahead, not the one right at the robot
        size_t aim_idx = path_idx_;
        for (size_t i = path_idx_; i < path_.size(); ++i) {
            float wx, wy;
            grid_->gridToWorld(path_[i].first, path_[i].second, wx, wy);
            float d = std::hypot(wx - pose_x_, wy - pose_y_);
            aim_idx = i;
            if (d > 1.0f) break;
        }

        auto [cgx, cgy] = path_[aim_idx];
        float wx, wy;
        grid_->gridToWorld(cgx, cgy, wx, wy);
        float dx = wx - static_cast<float>(pose_x_);
        float dy = wy - static_cast<float>(pose_y_);
        float dist = std::hypot(dx, dy);

        // Final-goal reach check
        float final_wx, final_wy;
        grid_->gridToWorld(path_.back().first, path_.back().second, final_wx, final_wy);
        float goal_dist = std::hypot(final_wx - pose_x_, final_wy - pose_y_);
        if (goal_dist < goal_tol_) {
            path_idx_ = path_.size();  // triggers arrival branch next tick
            publishPathMarker();
            return;
        }

        float bearing = std::atan2(dy, dx);
        float angle_err = normalizeAngle(bearing - static_cast<float>(pose_theta_));

        geometry_msgs::msg::Twist cmd;

        // Wall-in-front handling: instead of freezing, reverse a bit and turn
        // toward the more open side so the robot always escapes.
        if (nearest_front_ < 0.35f) {
            ++blocked_ticks_;
            cmd.linear.x  = -0.15f;
            cmd.angular.z = (nearest_left_ > nearest_right_) ? 1.2f : -1.2f;
            pub_cmd_->publish(cmd);
            publishPathMarker();

            // If we've been blocked for >1s (10 ticks at 10Hz) the current path
            // is bad — force a replan from the robot's updated pose.
            if (blocked_ticks_ > 10) {
                blocked_ticks_ = 0;
                if (!replanPath()) {
                    RCLCPP_WARN(get_logger(), "[%s] Wall-blocked, no replan — abandoning goal",
                                robot_id_.c_str());
                    explorer_->markVisited(goal_x_, goal_y_);
                    has_goal_ = false;
                    path_.clear(); path_idx_ = 0;
                    state_ = returning_to_dock_ ? RobotState::RETURNING : RobotState::EXPLORING;
                }
            }
            return;
        }
        blocked_ticks_ = 0;

        float speed_scale = nearest_front_ < 1.0f ? 0.5f : 1.0f;
        cmd.angular.z = std::clamp(1.4f * angle_err, -1.0f, 1.0f);
        float heading_ok = std::max(0.2f, 1.0f - std::abs(angle_err) / 2.5f);
        cmd.linear.x  = std::clamp(0.6f * dist, 0.25f, 0.6f) * heading_ok * speed_scale;
        pub_cmd_->publish(cmd);
        publishPathMarker();
    }

    void stop()
    {
        geometry_msgs::msg::Twist zero;
        pub_cmd_->publish(zero);
    }

    void drainBattery()
    {
        if (state_ == RobotState::DOCKING) return;
        float drain = static_cast<float>(get_parameter("battery_drain_rate").as_double());
        if (drain <= 0.0f) return;
        battery_ = std::max(0.0f, battery_ - drain);
        if (state_ == RobotState::FAILED || state_ == RobotState::RETURNING || returning_to_dock_) return;
        auto dock = nearestDock();
        float dock_dist = std::hypot(
            static_cast<float>(pose_x_) - dock.x,
            static_cast<float>(pose_y_) - dock.y);
        constexpr float AVG_SPEED = 0.3f;
        float time_to_return = dock_dist / AVG_SPEED;
        float time_to_empty  = battery_ / drain;
        float safety = static_cast<float>(get_parameter("dock_return_safety").as_double());
        if (((battery_ < 0.9f) && (time_to_return > safety * time_to_empty)) ||
            battery_ < 0.05f) {
            state_ = RobotState::RETURNING;
            RCLCPP_WARN(get_logger(),
                "[%s] Returning — battery %.0f%%", robot_id_.c_str(), battery_ * 100.0f);
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
