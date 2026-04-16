#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "swarmap_msgs/msg/partial_map.hpp"
#include "swarmap_msgs/msg/robot_status.hpp"

using namespace std::chrono_literals;

namespace swarmap {

static std::string jsonStr(const std::string &s)
{
    return "\"" + s + "\"";
}
static std::string jsonKV(const std::string &k, const std::string &v)
{
    return jsonStr(k) + ":" + v;
}

struct RobotInfo {
    std::string id;
    bool        is_active   = true;
    float       battery     = 1.0f;
    std::string state       = "IDLE";
    int         cells_mapped= 0;
    double      last_seen_s = 0.0;
    double      x           = 0.0;
    double      y           = 0.0;
};

class MapAggregatorNode : public rclcpp::Node {
public:
    MapAggregatorNode()
        : rclcpp::Node("map_aggregator_node")
    {
        declare_parameter("num_robots",     10);
        declare_parameter("map_resolution", 0.1);
        declare_parameter("map_width_m",    50.0);
        declare_parameter("map_height_m",   50.0);
        declare_parameter("publish_rate_hz", 1.0);

        int   n   = get_parameter("num_robots").as_int();
        float res = get_parameter("map_resolution").as_double();
        int   w   = static_cast<int>(get_parameter("map_width_m").as_double()  / res);
        int   h   = static_cast<int>(get_parameter("map_height_m").as_double() / res);

        
        global_map_.header.frame_id    = "map";
        global_map_.info.resolution    = res;
        global_map_.info.width         = static_cast<uint32_t>(w);
        global_map_.info.height        = static_cast<uint32_t>(h);
        global_map_.info.origin.position.x = 0.0;
        global_map_.info.origin.position.y = 0.0;
        global_map_.info.origin.orientation.w = 1.0;
        global_map_.data.assign(static_cast<size_t>(w * h), -1);

        cell_prob_sum_.assign(static_cast<size_t>(w * h), 0.0f);
        cell_conf_sum_.assign(static_cast<size_t>(w * h), 0.0f);
        visit_counts_.assign(static_cast<size_t>(w * h), 0);

        auto qos_reliable = rclcpp::QoS(10).reliable();
        auto qos_best     = rclcpp::QoS(10).best_effort();

        
        pub_map_  = create_publisher<nav_msgs::msg::OccupancyGrid>(
                        "/swarm/global_map", rclcpp::QoS(1).transient_local());
        pub_conf_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
                        "/dashboard/confidence_map", rclcpp::QoS(1).transient_local());
        pub_stats_= create_publisher<std_msgs::msg::String>(
                        "/dashboard/stats",  qos_reliable);
        pub_events_= create_publisher<std_msgs::msg::String>(
                        "/dashboard/events", qos_reliable);
        pub_markers_= create_publisher<visualization_msgs::msg::MarkerArray>(
                        "/swarm/frontier_markers", qos_best);
        pub_robot_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
                        "/swarm/robot_markers", qos_best);


        qos_reliable_ = qos_reliable;
        for (int i = 0; i < n; ++i) {
            subscribeRobot("robot_" + std::to_string(i));
        }

        sub_spawn_ = create_subscription<std_msgs::msg::String>(
            "/swarm/spawn_robot", 10,
            [this](std_msgs::msg::String::SharedPtr m){
                subscribeRobot(m->data);
            });

        sub_kill_ = create_subscription<std_msgs::msg::String>(
            "/swarm/kill_robot", 10,
            [this](std_msgs::msg::String::SharedPtr m){
                std::lock_guard lock(robots_mutex_);
                robots_.erase(m->data);
                killed_ids_.insert(m->data);
            });

        sub_reset_ = create_subscription<std_msgs::msg::String>(
            "/swarm/reset_map", 10,
            [this](std_msgs::msg::String::SharedPtr){
                std::lock_guard glock(grid_mutex_);
                std::lock_guard rlock(robots_mutex_);
                size_t N = global_map_.data.size();
                std::fill(global_map_.data.begin(), global_map_.data.end(), -1);
                std::fill(cell_prob_sum_.begin(), cell_prob_sum_.end(), 0.0f);
                std::fill(cell_conf_sum_.begin(), cell_conf_sum_.end(), 0.0f);
                std::fill(visit_counts_.begin(), visit_counts_.end(), 0);
                RCLCPP_WARN(get_logger(), "Global map RESET — %zu cells cleared", N);
            });

        double rate_ms = 1000.0 / get_parameter("publish_rate_hz").as_double();
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(rate_ms)),
            [this](){ publishAll(); });

        RCLCPP_INFO(get_logger(), "MapAggregator ready — tracking %d robots", n);
    }

private:
    
    nav_msgs::msg::OccupancyGrid global_map_;
    std::vector<float>   cell_prob_sum_;
    std::vector<float>   cell_conf_sum_;
    std::vector<uint8_t> visit_counts_;
    std::mutex           grid_mutex_;

    
    std::unordered_map<std::string, RobotInfo> robots_;
    std::mutex robots_mutex_;

    
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr   pub_map_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr   pub_conf_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr          pub_stats_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr          pub_events_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_robot_markers_;

    std::vector<rclcpp::Subscription<swarmap_msgs::msg::PartialMap>::SharedPtr>  map_subs_;
    std::vector<rclcpp::Subscription<swarmap_msgs::msg::RobotStatus>::SharedPtr> status_subs_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_spawn_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_kill_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_reset_;
    std::unordered_set<std::string> subscribed_ids_;
    std::unordered_set<std::string> killed_ids_;
    rclcpp::QoS qos_reliable_ = rclcpp::QoS(10).reliable();

    rclcpp::TimerBase::SharedPtr timer_;

    void subscribeRobot(const std::string &id)
    {
        if (id.empty() || subscribed_ids_.count(id)) return;
        subscribed_ids_.insert(id);
        const std::string ns = "/" + id;

        map_subs_.push_back(
            create_subscription<swarmap_msgs::msg::PartialMap>(
                ns + "/map", qos_reliable_,
                [this](swarmap_msgs::msg::PartialMap::SharedPtr m){ onPartialMap(m); }));

        status_subs_.push_back(
            create_subscription<swarmap_msgs::msg::RobotStatus>(
                ns + "/status", qos_reliable_,
                [this](swarmap_msgs::msg::RobotStatus::SharedPtr m){ onRobotStatus(m); }));

        RCLCPP_INFO(get_logger(), "MapAggregator subscribed to %s", id.c_str());
    }

    
    void onPartialMap(const swarmap_msgs::msg::PartialMap::SharedPtr &msg)
    {
        const int W = static_cast<int>(global_map_.info.width);
        const int H = static_cast<int>(global_map_.info.height);

        std::lock_guard lock(grid_mutex_);
        for (int row = 0; row < msg->height; ++row) {
            for (int col = 0; col < msg->width; ++col) {
                int src_idx = row * msg->width + col;
                int8_t val  = msg->data[src_idx];
                if (val == -1) continue;

                int gx = msg->origin_x + col;
                int gy = msg->origin_y + row;
                if (gx < 0 || gx >= W || gy < 0 || gy >= H) continue;

                float conf = (src_idx < static_cast<int>(msg->confidence.size()))
                             ? msg->confidence[src_idx] : 0.5f;
                float p    = (val == 100) ? 1.0f : 0.0f;

                int idx = gy * W + gx;
                cell_prob_sum_[idx] += conf * p;
                cell_conf_sum_[idx] += conf;
                if (visit_counts_[idx] < 255) ++visit_counts_[idx];
            }
        }
    }

    void onRobotStatus(const swarmap_msgs::msg::RobotStatus::SharedPtr &msg)
    {
        std::lock_guard lock(robots_mutex_);
        if (killed_ids_.count(msg->robot_id)) return;
        auto &r       = robots_[msg->robot_id];
        r.id          = msg->robot_id;
        r.is_active   = msg->is_active;
        r.battery     = msg->battery_level;
        r.state       = msg->current_state;
        r.cells_mapped= msg->cells_mapped;
        r.last_seen_s = get_clock()->now().seconds();
        r.x           = msg->pose.position.x;
        r.y           = msg->pose.position.y;
    }

    
    void publishAll()
    {
        rebuildGlobalMap();
        publishStats();
        pub_map_->publish(global_map_);
        publishConfidenceMap();
        publishRobotMarkers();
    }

    void rebuildGlobalMap()
    {
        const size_t N = global_map_.data.size();
        std::lock_guard lock(grid_mutex_);
        global_map_.header.stamp = get_clock()->now();

        for (size_t i = 0; i < N; ++i) {
            float w = cell_conf_sum_[i];
            if (w < 1e-6f) {
                global_map_.data[i] = -1;
                continue;
            }
            float p = cell_prob_sum_[i] / w;
            global_map_.data[i] = (p > 0.6f) ? 100 : 0;
        }
    }

    void publishStats()
    {
        std::lock_guard lock(robots_mutex_);

        int active = 0, failed = 0;
        float total_coverage = computeCoverage();

        std::ostringstream oss;
        oss << "{";
        oss << jsonKV("coverage_pct", std::to_string(total_coverage * 100.0f)) << ",";

        
        oss << jsonStr("robots") << ":[";
        bool first = true;
        for (auto &[id, r] : robots_) {
            if (r.is_active) ++active; else ++failed;
            if (!first) oss << ",";
            first = false;
            oss << "{"
                << jsonKV("id",          jsonStr(r.id))              << ","
                << jsonKV("state",       jsonStr(r.state))           << ","
                << jsonKV("battery",     std::to_string(r.battery))  << ","
                << jsonKV("cells_mapped",std::to_string(r.cells_mapped)) << ","
                << jsonKV("x",           std::to_string(r.x))       << ","
                << jsonKV("y",           std::to_string(r.y))
                << "}";
        }
        oss << "],";
        oss << jsonKV("active_robots", std::to_string(active)) << ",";
        oss << jsonKV("failed_robots", std::to_string(failed));
        oss << "}";

        std_msgs::msg::String out;
        out.data = oss.str();
        pub_stats_->publish(out);
    }

    void publishConfidenceMap()
    {
        nav_msgs::msg::OccupancyGrid cmap;
        cmap.header = global_map_.header;
        cmap.info   = global_map_.info;
        const size_t N = visit_counts_.size();
        cmap.data.resize(N);
        std::lock_guard lock(grid_mutex_);
        for (size_t i = 0; i < N; ++i) {
            uint8_t v = visit_counts_[i];
            // FIX #5: prevent overflow — scale [0,255] to [0,100] safely
            cmap.data[i] = (v == 0) ? -1 : static_cast<int8_t>(std::min<int>(static_cast<int>(v) * 100 / 255, 100));
        }
        pub_conf_->publish(cmap);
    }

    void publishRobotMarkers()
    {
        visualization_msgs::msg::MarkerArray ma;
        std::lock_guard lock(robots_mutex_);

        // FIX: Clear all old markers first so killed robots vanish from RViz
        {
            visualization_msgs::msg::Marker clear;
            clear.header.frame_id = "map";
            clear.header.stamp = get_clock()->now();
            clear.ns = "";
            clear.id = 0;
            clear.action = visualization_msgs::msg::Marker::DELETEALL;
            ma.markers.push_back(clear);
        }
        int id = 0;

        // Dock markers (green cylinders)
        float mw = static_cast<float>(get_parameter("map_width_m").as_double());
        float mh = static_cast<float>(get_parameter("map_height_m").as_double());
        float docks[][2] = {{mw*0.50f, mh*0.50f},
                            {mw*0.25f, mh*0.25f}, {mw*0.75f, mh*0.25f},
                            {mw*0.25f, mh*0.75f}, {mw*0.75f, mh*0.75f}};
        for (int i = 0; i < 5; ++i) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = get_clock()->now();
            m.ns = "docks"; m.id = id++;
            m.type = visualization_msgs::msg::Marker::CYLINDER;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = docks[i][0];
            m.pose.position.y = docks[i][1];
            m.pose.position.z = 0.1;
            m.pose.orientation.w = 1.0;
            m.scale.x = 1.0; m.scale.y = 1.0; m.scale.z = 0.2;
            m.color.r = 0.0; m.color.g = 0.8; m.color.b = 0.4; m.color.a = 0.6;
            ma.markers.push_back(m);
        }

        // Robot body markers (colored spheres)
        static const float palette[][3] = {
            {0.0f,0.83f,1.0f}, {1.0f,0.3f,0.82f}, {1.0f,0.85f,0.3f}, {0.3f,1.0f,0.53f},
            {1.0f,0.58f,0.3f}, {0.72f,0.42f,1.0f}, {1.0f,0.31f,0.31f}, {0.83f,1.0f,0.3f},
        };
        int ri = 0;
        for (auto &[rid, r] : robots_) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = get_clock()->now();
            m.ns = "robots"; m.id = id++;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = r.x;
            m.pose.position.y = r.y;
            m.pose.position.z = 0.3;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.6; m.scale.y = 0.6; m.scale.z = 0.6;
            int ci = ri % 8;
            m.color.r = palette[ci][0]; m.color.g = palette[ci][1];
            m.color.b = palette[ci][2]; m.color.a = r.is_active ? 1.0f : 0.3f;
            ma.markers.push_back(m);

            // Label
            visualization_msgs::msg::Marker lbl;
            lbl.header = m.header;
            lbl.ns = "robot_labels"; lbl.id = id++;
            lbl.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            lbl.action = visualization_msgs::msg::Marker::ADD;
            lbl.pose.position.x = r.x;
            lbl.pose.position.y = r.y;
            lbl.pose.position.z = 1.0;
            lbl.pose.orientation.w = 1.0;
            lbl.scale.z = 0.5;
            lbl.color.r = 1.0; lbl.color.g = 1.0; lbl.color.b = 1.0; lbl.color.a = 1.0;
            lbl.text = rid;
            ma.markers.push_back(lbl);
            ++ri;
        }
        pub_robot_markers_->publish(ma);
    }

    float computeCoverage() const
    {
        int known = 0;
        for (int8_t v : global_map_.data)
            if (v != -1) ++known;
        return static_cast<float>(known) /
               static_cast<float>(global_map_.data.size());
    }
};

} 

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<swarmap::MapAggregatorNode>());
    rclcpp::shutdown();
    return 0;
}
