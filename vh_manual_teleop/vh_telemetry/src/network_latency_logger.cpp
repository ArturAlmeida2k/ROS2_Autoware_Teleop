#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include <fstream>
#include <cstdlib> 
#include <string>
#include <iomanip>
#include <filesystem> 
#include <ctime>   
#include <sstream>

using Float32 = std_msgs::msg::Float32;
using UInt32 = std_msgs::msg::UInt32;

class TeleopLatencyLoggerNode : public rclcpp::Node {
public:
    TeleopLatencyLoggerNode() : Node("teleop_latency_logger_node") {

        // Create a file with a the command latency with in the folder autoware_workspace/metrics/latency_cmd with the name of day and hour of creation
        
            const char* home_dir = std::getenv("HOME");
            std::string dir_path = std::string(home_dir) + "/autoware_workspace/metrics/latency_cmd";

            std::filesystem::create_directories(dir_path);

            auto now = std::chrono::system_clock::now();
            std::time_t now_c = std::chrono::system_clock::to_time_t(now);
            std::tm* now_tm = std::localtime(&now_c);

            std::ostringstream time_stream;
            time_stream << std::put_time(now_tm, "%Y%m%d_%H%M%S");

            std::string file_path = dir_path + "/latency_" + time_stream.str() + ".csv";

            RCLCPP_INFO(this->get_logger(), "A gravar dados em: %s", file_path.c_str());
        

        log_file_.open(file_path, std::ios::app);

        if (log_file_.is_open()) {
            log_file_ << "timestamp_ms,latency_ms,total_packets_lost\n";
        } else {
            RCLCPP_ERROR(this->get_logger(), "Erro ao abrir ficheiro de log.");
        }

        sub_latency_ = this->create_subscription<Float32>(
            "/metrics/latency_cmd_ms", 10,
            std::bind(&TeleopLatencyLoggerNode::latency_callback, this, std::placeholders::_1));

        sub_lost_packets_ = this->create_subscription<UInt32>(
            "/metrics/packets_lost", 10,
            std::bind(&TeleopLatencyLoggerNode::lost_callback, this, std::placeholders::_1));

            
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
    rclcpp::Subscription<UInt32>::SharedPtr sub_lost_packets_;

    uint32_t lost_packets_ = 0;

    void lost_callback(const UInt32::SharedPtr msg){
        lost_packets_ = msg->data;
    }

    void latency_callback(const Float32::SharedPtr msg) {
        if (log_file_.is_open()) {
            auto now_ms = this->now().nanoseconds() / 1e6;
            log_file_ << std::fixed << now_ms << "," << msg->data << "," << lost_packets_ << "\n";
            log_file_.flush();

            lost_packets_ = 0;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopLatencyLoggerNode>());
    rclcpp::shutdown();
    return 0;
}