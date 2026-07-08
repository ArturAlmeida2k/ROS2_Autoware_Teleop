#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/float32.hpp"

#include "msg_manual_teleop/msg/network_metrics.hpp"

using Int8    = std_msgs::msg::Int8;
using Float32 = std_msgs::msg::Float32;
using CommandMetrics = msg_manual_teleop::msg::NetworkMetrics;

class TeleopTopicMonitorNode : public rclcpp::Node {
public:
    static constexpr int8_t STATE_OK    = 0;
    static constexpr int8_t STATE_WARN  = 1;
    static constexpr int8_t STATE_ERROR = 2;

    TeleopTopicMonitorNode() : Node("teleop_topic_monitor_node") {
        this->declare_parameter<double>("warn_timeout_ms",  100.0);
        this->declare_parameter<double>("error_timeout_ms", 250.0);
        warn_timeout_ms_  = this->get_parameter("warn_timeout_ms").as_double();
        error_timeout_ms_ = this->get_parameter("error_timeout_ms").as_double();
        
        sub_metrics_ = this->create_subscription<CommandMetrics>("/metrics/network/teleop_commands", 10,
            std::bind(&TeleopTopicMonitorNode::metrics_callback, this, std::placeholders::_1));

        pub_safety_state_ = this->create_publisher<Int8>("/teleop/safety_state", rclcpp::QoS(1));

        last_msg_time_ = this->now();

        monitor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&TeleopTopicMonitorNode::check_timeouts, this));

        RCLCPP_INFO(this->get_logger(), "Teleop Topic Monitor Node started.");
    }

private:
    double warn_timeout_ms_;
    double error_timeout_ms_;
    double latest_latency_ms_ = 0.0;
    rclcpp::Time last_msg_time_;
    int8_t current_state_ = STATE_ERROR;

    rclcpp::Subscription<CommandMetrics>::SharedPtr sub_metrics_;
    rclcpp::Publisher<Int8>::SharedPtr       pub_safety_state_;
    rclcpp::TimerBase::SharedPtr             monitor_timer_;
    

    void metrics_callback(const CommandMetrics::SharedPtr msg) {
        latest_latency_ms_ = msg->latency_ms; 
        last_msg_time_ = this->now();
    }

    void check_timeouts() {
        double elapsed_ms = (this->now() - last_msg_time_).seconds() * 1000.0;

        int8_t new_state;
        if      (elapsed_ms > error_timeout_ms_)         new_state = STATE_ERROR;
        else if (latest_latency_ms_ > error_timeout_ms_) new_state = STATE_ERROR;
        else if (latest_latency_ms_ > warn_timeout_ms_)  new_state = STATE_WARN;
        else                                              new_state = STATE_OK;

        if (new_state != current_state_) {
            if      (new_state == STATE_ERROR) RCLCPP_ERROR(this->get_logger(), "NETWORK ERROR: latency %.1f ms / timeout %.1f ms.", latest_latency_ms_, elapsed_ms);
            else if (new_state == STATE_WARN)  RCLCPP_WARN (this->get_logger(), "NETWORK WARN: High latency %.1f ms.", latest_latency_ms_);
            else                               RCLCPP_INFO (this->get_logger(), "NETWORK OK: Connection stable.");
            current_state_ = new_state;
        }

        Int8 state_msg;
        state_msg.data = current_state_;
        pub_safety_state_->publish(state_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopTopicMonitorNode>());
    rclcpp::shutdown();
    return 0;
}