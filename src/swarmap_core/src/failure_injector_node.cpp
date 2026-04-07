#include <chrono>
#include <fstream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace swarmap {

// ─────────────────────────────────────────────────────────────────────────────
// FailureInjectorNode
// ─────────────────────────────────────────────────────────────────────────────
class FailureInjectorNode : public rclcpp::Node {
public:
    FailureInjectorNode()
        : rclcpp::Node("failure_injector_node"),
          rng_(std::random_device{}())
    {
        declare_parameter("num_robots",    10);
        declare_parameter("failure_rate",  0.0);   // failures per minute
        declare_parameter("failure_mode",  std::string("random"));  // random|progressive|cascade
        declare_parameter("log_file",      std::string("results/failure_log.csv"));

        num_robots_   = get_parameter("num_robots").as_int();
        failure_rate_ = get_parameter("failure_rate").as_double();
        failure_mode_ = get_parameter("failure_mode").as_string();
        log_file_     = get_parameter("log_file").as_string();

        pub_events_ = create_publisher<std_msgs::msg::String>("/swarm/events", 10);

        // Open log file
        if (!log_file_.empty()) {
            log_stream_.open(log_file_, std::ios::app);
            if (log_stream_.is_open())
                log_stream_ << "time_s,robot_id,failure_mode\n";
        }

        // Check every 10 s
        timer_ = create_wall_timer(10s, [this](){ tick(); });

        RCLCPP_INFO(get_logger(),
            "FailureInjector ready: rate=%.3f/min mode=%s",
            failure_rate_, failure_mode_.c_str());
    }

private:
    int         num_robots_;
    double      failure_rate_;
    std::string failure_mode_;
    std::string log_file_;
    std::ofstream log_stream_;

    std::unordered_set<int> failed_ids_;
    std::default_random_engine rng_;
    int cascade_next_ = 0;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_events_;
    rclcpp::TimerBase::SharedPtr timer_;

    void tick()
    {
        // Re-read parameters in case they were set via /swarm/set_param
        failure_rate_ = get_parameter("failure_rate").as_double();
        failure_mode_ = get_parameter("failure_mode").as_string();

        if (failure_rate_ <= 0.0) return;

        // Probability of at least one failure in the 10 s window
        // P = 1 - exp(-rate * dt_min)
        double dt_min = 10.0 / 60.0;
        double p_fail = 1.0 - std::exp(-failure_rate_ * dt_min);

        std::uniform_real_distribution<double> coin(0.0, 1.0);
        if (coin(rng_) > p_fail) return;

        // Pick target robot
        int target = pickTarget();
        if (target < 0) return;

        injectFailure(target);
    }

    int pickTarget()
    {
        // Collect active robots
        std::vector<int> active;
        for (int i = 0; i < num_robots_; ++i)
            if (failed_ids_.count(i) == 0) active.push_back(i);

        if (active.empty()) return -1;

        if (failure_mode_ == "progressive") {
            // Fail in order
            int t = active.front();
            return t;
        } else if (failure_mode_ == "cascade") {
            // Fail the lowest-ID active robot
            return active.front();
        } else {
            // random
            std::uniform_int_distribution<int> pick(0, static_cast<int>(active.size()) - 1);
            return active[pick(rng_)];
        }
    }

    void injectFailure(int robot_idx)
    {
        failed_ids_.insert(robot_idx);
        std::string rid = "robot_" + std::to_string(robot_idx);

        // Call lifecycle SHUTDOWN on the robot node
        std::string service_name = "/" + rid + "/robot_node/change_state";
        auto client = create_client<lifecycle_msgs::srv::ChangeState>(service_name);

        if (client->wait_for_service(1s)) {
            auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
            req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN;
            client->async_send_request(req);
        } else {
            RCLCPP_WARN(get_logger(), "Lifecycle service not ready for %s — skipping", rid.c_str());
        }

        // Publish event
        double now_s = get_clock()->now().seconds();
        std::ostringstream oss;
        oss << "{\"time_s\":" << now_s
            << ",\"robot_id\":\"" << rid << "\""
            << ",\"failure_mode\":\"" << failure_mode_ << "\"}";

        std_msgs::msg::String ev;
        ev.data = oss.str();
        pub_events_->publish(ev);

        // Log to CSV
        if (log_stream_.is_open())
            log_stream_ << now_s << "," << rid << "," << failure_mode_ << "\n";

        RCLCPP_WARN(get_logger(), "Injected %s failure on %s",
                    failure_mode_.c_str(), rid.c_str());
    }
};

} // namespace swarmap

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<swarmap::FailureInjectorNode>());
    rclcpp::shutdown();
    return 0;
}
