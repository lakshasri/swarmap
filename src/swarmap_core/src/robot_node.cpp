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

namespace swarmap {

enum class RobotState { IDLE, EXPLORING, NAVIGATING, RETURNING, FAILED };

static std::string stateStr(RobotState s) {
    switch (s) {
        case RobotState::IDLE:       return "IDLE";
        case RobotState::EXPLORING:  return "EXPLORING";
        case RobotState::NAVIGATING: return "NAVIGATING";
        case RobotState::RETURNING:  return "RETURNING";
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
        declare_parameter("update_rate_hz",       5.0);
    }

    
    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        robot_id_    = get_parameter("robot_id").as_string();
        sensor_range_= get_parameter("sensor_range").as_double();
        comm_radius_ = get_parameter("comm_radius").as_double();
        noise_level_ = get_parameter("noise_level").as_double();
        battery_     = 1.0f;
        state_       = RobotState::IDLE;
        pose_x_ = pose_y_ = pose_theta_ = 0.0;

        float res    = get_parameter("map_resolution").as_double();
        int   w_cells = static_cast<int>(get_parameter("map_width_m").as_double()  / res);
        int   h_cells = static_cast<int>(get_parameter("map_height_m").as_double() / res);
        grid_ = std::make_unique<OccupancyGrid>(w_cells, h_cells, res,
                                                -(w_cells * res / 2.0f),
                                                -(h_cells * res / 2.0f));
        grid_->markAllDirty();

        explorer_ = std::make_unique<FrontierExplorer>(
            get_parameter("frontier_min_size").as_double(),
            2.0f, 5.0f,
            static_cast<float>(get_parameter("battery_weight").as_double()));
        merger_   = std::make_unique<MapMerger>();
        tracker_  = std::make_unique<NeighbourTracker>();
        tf_buffer_= std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto qos_reliable = rclcpp::QoS(10).reliable();
        auto qos_best     = rclcpp::QoS(10).best_effort();
        std::string ns = "/" + robot_id_;

        
        pub_map_    = create_publisher<swarmap_msgs::msg::PartialMap>(
                          ns + "/map",    qos_reliable);
        pub_status_ = create_publisher<swarmap_msgs::msg::RobotStatus>(
                          ns + "/status", qos_reliable);
        pub_cmd_    = create_publisher<geometry_msgs::msg::Twist>(
                          ns + "/cmd_vel", qos_best);
        pub_disc_   = create_publisher<swarmap_msgs::msg::NeighbourDiscovery>(
                          "/swarm/discovery", qos_reliable);
        pub_bid_    = create_publisher<swarmap_msgs::msg::FrontierBid>(
                          ns + "/frontier_bid", qos_reliable);
        pub_acc_    = create_publisher<std_msgs::msg::Float32>(
                          ns + "/map_accuracy", qos_reliable);

        
        sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
            ns + "/scan", qos_best,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg){ onScan(msg); });

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            ns + "/odom", qos_best,
            [this](nav_msgs::msg::Odometry::SharedPtr msg){ onOdom(msg); });

        sub_disc_ = create_subscription<swarmap_msgs::msg::NeighbourDiscovery>(
            "/swarm/discovery", qos_reliable,
            [this](swarmap_msgs::msg::NeighbourDiscovery::SharedPtr msg){ onDiscovery(msg); });

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
        timer_map_.reset();
        timer_status_.reset();
        timer_disc_.reset();
        timer_explore_.reset();
        timer_battery_.reset();
        stop();
        RCLCPP_INFO(get_logger(), "[%s] Deactivated", robot_id_.c_str());
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
    {
        grid_.reset();
        explorer_.reset();
        merger_.reset();
        tracker_.reset();
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

    float  goal_x_ = 0.0f, goal_y_ = 0.0f;
    bool   has_goal_ = false;

    
    double pose_x_last_ = 0.0, pose_y_last_ = 0.0;
    double last_moved_time_ = 0.0;
    static constexpr double STUCK_TIMEOUT_S  = 5.0;   
    static constexpr double STUCK_DIST_THRESH = 0.05;  

    std::unique_ptr<OccupancyGrid>    grid_;
    std::unique_ptr<FrontierExplorer> explorer_;
    std::unique_ptr<MapMerger>        merger_;
    std::unique_ptr<NeighbourTracker> tracker_;

    std::unique_ptr<tf2_ros::TransformBroadcaster>          tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer>                         tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>              tf_listener_;

    
    rclcpp::Publisher<swarmap_msgs::msg::PartialMap>::SharedPtr      pub_map_;
    rclcpp::Publisher<swarmap_msgs::msg::RobotStatus>::SharedPtr     pub_status_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr          pub_cmd_;
    rclcpp::Publisher<swarmap_msgs::msg::NeighbourDiscovery>::SharedPtr pub_disc_;
    rclcpp::Publisher<swarmap_msgs::msg::FrontierBid>::SharedPtr     pub_bid_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             pub_acc_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    sub_scan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        sub_odom_;
    rclcpp::Subscription<swarmap_msgs::msg::NeighbourDiscovery>::SharedPtr sub_disc_;

    
    std::unordered_map<std::string,
        rclcpp::Subscription<swarmap_msgs::msg::PartialMap>::SharedPtr>  neighbour_map_subs_;
    std::unordered_map<std::string,
        rclcpp::Subscription<swarmap_msgs::msg::FrontierBid>::SharedPtr> neighbour_bid_subs_;

    rclcpp::TimerBase::SharedPtr timer_map_, timer_status_, timer_disc_,
                                  timer_explore_, timer_battery_;

    std::default_random_engine rng_{std::random_device{}()};

    
    void onScan(const sensor_msgs::msg::LaserScan::SharedPtr &msg)
    {
        if (state_ == RobotState::FAILED) return;
        std::unique_lock lock(grid_->mutex);

        float angle = msg->angle_min;
        std::normal_distribution<float> noise_dist(0.0f,
            static_cast<float>(noise_level_ * sensor_range_));

        for (float raw_range : msg->ranges) {
            float r = raw_range;
            if (noise_level_ > 0.0) r += noise_dist(rng_);
            r = std::clamp(r, msg->range_min, msg->range_max);

            float ex = static_cast<float>(pose_x_) + r * std::cos(pose_theta_ + angle);
            float ey = static_cast<float>(pose_y_) + r * std::sin(pose_theta_ + angle);
            rayCast(static_cast<float>(pose_x_), static_cast<float>(pose_y_),
                    ex, ey, r < msg->range_max - 0.05f);
            angle += msg->angle_increment;
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

        while (true) {
            if (cx == gx1 && cy == gy1) {
                if (mark_endpoint)
                    grid_->updateCell(cx, cy, LOG_ODDS_OCCUPIED);
                else
                    grid_->updateCell(cx, cy, LOG_ODDS_FREE);
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

        
        double now = get_clock()->now().seconds();
        double dist = std::hypot(pose_x_ - pose_x_last_, pose_y_ - pose_y_last_);
        if (dist > STUCK_DIST_THRESH) {
            pose_x_last_    = pose_x_;
            pose_y_last_    = pose_y_;
            last_moved_time_= now;
        }
    }

    void onDiscovery(const swarmap_msgs::msg::NeighbourDiscovery::SharedPtr &msg)
    {
        if (msg->robot_id == robot_id_) return;
        double now = get_clock()->now().seconds();
        tracker_->updateNeighbour(msg->robot_id,
                                   msg->position.x, msg->position.y,
                                   msg->comm_radius, now);

        
        if (neighbour_map_subs_.count(msg->robot_id) == 0) {
            const std::string ns = "/" + msg->robot_id;
            auto qos = rclcpp::QoS(5).reliable();

            neighbour_map_subs_[msg->robot_id] =
                create_subscription<swarmap_msgs::msg::PartialMap>(
                    ns + "/map", qos,
                    [this](swarmap_msgs::msg::PartialMap::SharedPtr m){ onNeighbourMap(m); });

            neighbour_bid_subs_[msg->robot_id] =
                create_subscription<swarmap_msgs::msg::FrontierBid>(
                    ns + "/frontier_bid", qos,
                    [this](swarmap_msgs::msg::FrontierBid::SharedPtr m){ onNeighbourBid(m); });

            RCLCPP_INFO(get_logger(), "[%s] Subscribed to neighbour %s",
                        robot_id_.c_str(), msg->robot_id.c_str());
        }
    }

    void onNeighbourMap(const swarmap_msgs::msg::PartialMap::SharedPtr &msg)
    {
        if (state_ == RobotState::FAILED) return;
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
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, pose_theta_);
        t.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(t);
    }

    
    void publishMap()
    {
        if (state_ == RobotState::FAILED) return;
        std::shared_lock lock(grid_->mutex);

        swarmap_msgs::msg::PartialMap msg;
        msg.header.stamp    = get_clock()->now();
        msg.header.frame_id = "map";
        msg.robot_id        = robot_id_;
        msg.resolution      = grid_->resolution();
        msg.origin_x        = 0;
        msg.origin_y        = 0;
        msg.width           = grid_->width();
        msg.height          = grid_->height();

        int total = grid_->width() * grid_->height();
        msg.data.reserve(total);
        msg.confidence.reserve(total);

        for (int gy = 0; gy < grid_->height(); ++gy) {
            for (int gx = 0; gx < grid_->width(); ++gx) {
                msg.data.push_back(grid_->getCellRos(gx, gy));
                msg.confidence.push_back(grid_->getCellConfidence(gx, gy));
                grid_->clearDirty(gx, gy);
            }
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
        msg.header.stamp    = get_clock()->now();
        msg.header.frame_id = "map";
        msg.robot_id        = robot_id_;
        msg.pose.position.x = pose_x_;
        msg.pose.position.y = pose_y_;
        msg.battery_level   = battery_;
        msg.is_active       = (state_ != RobotState::FAILED);
        msg.current_state   = stateStr(state_);
        msg.cells_mapped    = grid_->mappedCellCount();
        msg.neighbour_ids   = neighbours;
        pub_status_->publish(msg);
    }

    void publishDiscovery()
    {
        swarmap_msgs::msg::NeighbourDiscovery msg;
        msg.header.stamp    = get_clock()->now();
        msg.header.frame_id = "map";
        msg.robot_id        = robot_id_;
        msg.position.x      = pose_x_;
        msg.position.y      = pose_y_;
        msg.comm_radius     = static_cast<float>(comm_radius_);
        pub_disc_->publish(msg);
    }

    
    void explorationTick()
    {
        if (state_ == RobotState::FAILED) return;

        if (state_ == RobotState::NAVIGATING) {
            driveTowardGoal();
            return;
        }

        if (state_ == RobotState::RETURNING) {
            goal_x_ = 0.0f; goal_y_ = 0.0f;
            has_goal_ = true;
            state_ = RobotState::NAVIGATING;
            return;
        }

        
        std::shared_lock lock(grid_->mutex);
        auto clusters = explorer_->detect(*grid_,
                                           static_cast<float>(pose_x_),
                                           static_cast<float>(pose_y_),
                                           battery_);
        lock.unlock();

        if (clusters.empty()) {
            stop();
            state_ = RobotState::IDLE;
            RCLCPP_INFO(get_logger(), "[%s] No frontiers — mapping complete", robot_id_.c_str());
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
                state_ = RobotState::NAVIGATING;

                swarmap_msgs::msg::FrontierBid bid;
                bid.header.stamp        = get_clock()->now();
                bid.robot_id            = robot_id_;
                bid.frontier_centroid.x = cl.centroid_wx;
                bid.frontier_centroid.y = cl.centroid_wy;
                bid.bid_score           = cl.score;
                bid.battery_level       = battery_;
                bid.claim               = true;
                pub_bid_->publish(bid);

                RCLCPP_DEBUG(get_logger(), "[%s] Claimed frontier (%.2f, %.2f) score=%.2f",
                             robot_id_.c_str(), cl.centroid_wx, cl.centroid_wy, cl.score);
                return;
            }
        }
        
        state_ = RobotState::EXPLORING;
    }

    
    void driveTowardGoal()
    {
        if (!has_goal_) { state_ = RobotState::EXPLORING; return; }

        
        double now = get_clock()->now().seconds();
        if (last_moved_time_ > 0.0 && (now - last_moved_time_) > STUCK_TIMEOUT_S) {
            RCLCPP_WARN(get_logger(), "[%s] Stuck — abandoning goal (%.2f, %.2f)",
                        robot_id_.c_str(), goal_x_, goal_y_);
            stop();
            explorer_->markVisited(goal_x_, goal_y_);  
            has_goal_ = false;
            last_moved_time_ = now;  
            state_ = RobotState::EXPLORING;
            return;
        }

        float dx = goal_x_ - static_cast<float>(pose_x_);
        float dy = goal_y_ - static_cast<float>(pose_y_);
        float dist = std::hypot(dx, dy);

        float goal_tol = get_parameter("goal_tolerance").as_double();
        if (dist < goal_tol) {
            stop();
            explorer_->markVisited(goal_x_, goal_y_);
            has_goal_ = false;
            state_ = RobotState::EXPLORING;

            
            swarmap_msgs::msg::FrontierBid release;
            release.header.stamp = get_clock()->now();
            release.robot_id     = robot_id_;
            release.claim        = false;
            pub_bid_->publish(release);
            return;
        }

        float bearing = std::atan2(dy, dx);
        float angle_err = bearing - static_cast<float>(pose_theta_);
        
        while (angle_err >  M_PI) angle_err -= 2.0f * M_PI;
        while (angle_err < -M_PI) angle_err += 2.0f * M_PI;

        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = std::clamp(1.5f * angle_err, -1.0f, 1.0f);
        cmd.linear.x  = std::clamp(0.5f * dist, 0.0f, 0.3f)
                        * (1.0f - std::abs(angle_err) / M_PI);
        pub_cmd_->publish(cmd);
    }

    void stop()
    {
        geometry_msgs::msg::Twist zero;
        pub_cmd_->publish(zero);
    }

    
    void drainBattery()
    {
        float drain  = static_cast<float>(get_parameter("battery_drain_rate").as_double());
        battery_ = std::max(0.0f, battery_ - drain);

        if (state_ == RobotState::FAILED || state_ == RobotState::RETURNING) return;

        // Predictive return: compare time-to-empty vs time-to-reach-dock.
        // drain is fraction per second (timer fires every 1 s).
        float dock_dist      = std::hypot(static_cast<float>(pose_x_),
                                          static_cast<float>(pose_y_));
        constexpr float AVG_SPEED = 0.25f;          // conservative m/s estimate
        float time_to_return = dock_dist / AVG_SPEED;
        float time_to_empty  = (drain > 0.0f) ? battery_ / drain : 1e9f;
        float safety         = static_cast<float>(
                                   get_parameter("dock_return_safety").as_double());

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
