#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <fstream>
#include <cstdlib> 
#include <string>

using Float32 = std_msgs::msg::Float32;

class TeleopLatencyLoggerNode : public rclcpp::Node {
public:
    TeleopLatencyLoggerNode() : Node("teleop_latency_logger_node") {

        const char* home_dir = std::getenv("HOME");
        std::string file_path = std::string(home_dir) + "/autoware_workspace/metrics/latency_cmd.csv";

        RCLCPP_INFO(this->get_logger(), "%s", file_path);

        if (log_file_.is_open()) {
            log_file_ << "timestamp_ms,latency_ms\n";
        } else {
            RCLCPP_ERROR(this->get_logger(), "Erro ao abrir ficheiro de log.");
        }

        sub_latency_ = this->create_subscription<Float32>(
            "/metrics/latency_cmd_ms", 10,
            std::bind(&TeleopLatencyLoggerNode::latency_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Teleop Latency Logger Node started.");
    }

    ~TeleopLatencyLoggerNode() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

private:
    std::ofstream log_file_;
    rclcpp::Subscription<Float32>::SharedPtr sub_latency_;

    void latency_callback(const Float32::SharedPtr msg) {
        if (log_file_.is_open()) {
            auto now_ms = this->now().nanoseconds() / 1e6;
            log_file_ << now_ms << "," << msg->data << "\n";
            log_file_.flush();
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopLatencyLoggerNode>());
    rclcpp::shutdown();
    return 0;
}