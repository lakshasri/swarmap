#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
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
        global_map_.info.origin.position.x = -(w * res / 2.0);
        global_map_.info.origin.position.y = -(h * res / 2.0);
        global_map_.info.origin.orientation.w = 1.0;
        global_map_.data.assign(static_cast<size_t>(w * h), -1);

        
        cell_prob_sum_.assign(static_cast<size_t>(w * h), 0.0f);
        cell_conf_sum_.assign(static_cast<size_t>(w * h), 0.0f);

        auto qos_reliable = rclcpp::QoS(10).reliable();
        auto qos_best     = rclcpp::QoS(10).best_effort();

        
        pub_map_  = create_publisher<nav_msgs::msg::OccupancyGrid>(
                        "/swarm/global_map", rclcpp::QoS(1).transient_local());
        pub_stats_= create_publisher<std_msgs::msg::String>(
                        "/dashboard/stats",  qos_reliable);
        pub_events_= create_publisher<std_msgs::msg::String>(
                        "/dashboard/events", qos_reliable);
        pub_markers_= create_publisher<visualization_msgs::msg::MarkerArray>(
                        "/swarm/frontier_markers", qos_best);

        
        for (int i = 0; i < n; ++i) {
            const std::string id = "robot_" + std::to_string(i);
            const std::string ns = "/" + id;

            map_subs_.push_back(
                create_subscription<swarmap_msgs::msg::PartialMap>(
                    ns + "/map", qos_reliable,
                    [this](swarmap_msgs::msg::PartialMap::SharedPtr m){ onPartialMap(m); }));

            status_subs_.push_back(
                create_subscription<swarmap_msgs::msg::RobotStatus>(
                    ns + "/status", qos_reliable,
                    [this](swarmap_msgs::msg::RobotStatus::SharedPtr m){ onRobotStatus(m); }));
        }

        double rate_ms = 1000.0 / get_parameter("publish_rate_hz").as_double();
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(rate_ms)),
            [this](){ publishAll(); });

        RCLCPP_INFO(get_logger(), "MapAggregator ready — tracking %d robots", n);
    }

private:
    
    nav_msgs::msg::OccupancyGrid global_map_;
    std::vector<float> cell_prob_sum_;   
    std::vector<float> cell_conf_sum_;   
    std::mutex         grid_mutex_;

    
    std::unordered_map<std::string, RobotInfo> robots_;
    std::mutex robots_mutex_;

    
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr   pub_map_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr          pub_stats_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr          pub_events_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

    std::vector<rclcpp::Subscription<swarmap_msgs::msg::PartialMap>::SharedPtr>  map_subs_;
    std::vector<rclcpp::Subscription<swarmap_msgs::msg::RobotStatus>::SharedPtr> status_subs_;

    rclcpp::TimerBase::SharedPtr timer_;

    
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
            }
        }
    }

    void onRobotStatus(const swarmap_msgs::msg::RobotStatus::SharedPtr &msg)
    {
        std::lock_guard lock(robots_mutex_);
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
