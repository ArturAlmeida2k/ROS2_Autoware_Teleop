#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "teleop_msgs/msg/telemetry_state.hpp"
#include "teleop_msgs/msg/node_metrics.hpp"
#include "teleop_msgs/msg/command_enums.hpp"

using Float32        = std_msgs::msg::Float32;
using Int8           = std_msgs::msg::Int8;
using UInt8          = std_msgs::msg::UInt8;
using TelemetryState = teleop_msgs::msg::TelemetryState;
using Metrics        = teleop_msgs::msg::NodeMetrics;
using CmdEnums       = teleop_msgs::msg::CommandEnums;

// Substitui o telemetry_node quando não há Autoware: lê os tópicos do carro
// e publica /teleop/telemetry a 50 Hz, no mesmo formato.
class VehicleTelemetryNode : public rclcpp::Node {
public:
    VehicleTelemetryNode() : Node("vehicle_telemetry_node") {
        pub_telemetry_ = create_publisher<TelemetryState>("/teleop/telemetry", 10);

        state_.mode        = CmdEnums::OPERATION_MODE_LOCAL;
        state_.engaged     = false;
        state_.gear        = CmdEnums::GEAR_NONE;
        state_.turn_signal = CmdEnums::TURN_OFF;

        sub_wheel_[0] = create_subscription<Float32>("/vehicle/wheel_speed_fl", 10,
            [this](const Float32::SharedPtr msg) { wheel_kmh_[0] = msg->data; });
        sub_wheel_[1] = create_subscription<Float32>("/vehicle/wheel_speed_fr", 10,
            [this](const Float32::SharedPtr msg) { wheel_kmh_[1] = msg->data; });
        sub_wheel_[2] = create_subscription<Float32>("/vehicle/wheel_speed_rl", 10,
            [this](const Float32::SharedPtr msg) { wheel_kmh_[2] = msg->data; });
        sub_wheel_[3] = create_subscription<Float32>("/vehicle/wheel_speed_rr", 10,
            [this](const Float32::SharedPtr msg) { wheel_kmh_[3] = msg->data; });

        // 1 = frente, -1 = marcha-atrás
        sub_direction_ = create_subscription<Float32>(
            "/vehicle/direction", 10,
            [this](const Float32::SharedPtr msg) { direction_ = (msg->data < 0.0f) ? -1.0f : 1.0f; });

        // 0=P, 1=R, 2=N, 3=D
        sub_gear_ = create_subscription<Int8>(
            "/vehicle/gear", 10,
            [this](const Int8::SharedPtr msg) {
                switch (msg->data) {
                    case 0: state_.gear = CmdEnums::GEAR_PARK;    break;
                    case 1: state_.gear = CmdEnums::GEAR_REVERSE; break;
                    case 3: state_.gear = CmdEnums::GEAR_DRIVE;   break;
                    default: state_.gear = CmdEnums::GEAR_NONE;   break;  // N
                }
            });

        // 0=nenhum, 1=esquerda, 2=direita, 3=quatro piscas
        sub_blinkers_ = create_subscription<Int8>(
            "/vehicle/blinkers", 10,
            [this](const Int8::SharedPtr msg) {
                switch (msg->data) {
                    case 1: state_.turn_signal = CmdEnums::TURN_LEFT;   break;
                    case 2: state_.turn_signal = CmdEnums::TURN_RIGHT;  break;
                    case 3: state_.turn_signal = CmdEnums::TURN_HAZARD; break;
                    default: state_.turn_signal = CmdEnums::TURN_OFF;   break;
                }
            });

        // modo publicado pelo vehicle_interface
        sub_mode_ = create_subscription<UInt8>(
            "/teleop/vehicle_mode", rclcpp::QoS(1).reliable().transient_local(),
            [this](const UInt8::SharedPtr msg) {
                state_.mode    = msg->data;
                state_.engaged = (msg->data == CmdEnums::OPERATION_MODE_REMOTE);
            });

        sub_latency_ = create_subscription<Metrics>(
            "/metrics/e2e_command_latency", 10,
            [this](const Metrics::SharedPtr msg) { state_.e2e_command_ms = msg->latency_ms; });

        sub_network_state_ = create_subscription<Int8>(
            "/teleop/safety_state", 10,
            [this](const Int8::SharedPtr msg) { state_.network_state = msg->data; });

        timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { publish(); });

        RCLCPP_INFO(get_logger(), "Vehicle telemetry node started (direct vehicle interface).");
    }

private:
    TelemetryState state_{};
    float wheel_kmh_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float direction_ = 1.0f;
    uint32_t seq_num_ = 1;

    rclcpp::Publisher<TelemetryState>::SharedPtr pub_telemetry_;
    rclcpp::Subscription<Float32>::SharedPtr     sub_wheel_[4];
    rclcpp::Subscription<Float32>::SharedPtr     sub_direction_;
    rclcpp::Subscription<Int8>::SharedPtr        sub_gear_;
    rclcpp::Subscription<Int8>::SharedPtr        sub_blinkers_;
    rclcpp::Subscription<UInt8>::SharedPtr       sub_mode_;
    rclcpp::Subscription<Metrics>::SharedPtr     sub_latency_;
    rclcpp::Subscription<Int8>::SharedPtr        sub_network_state_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish() {
        float sum = 0.0f;
        for (float w : wheel_kmh_) sum += std::abs(w);
        state_.velocity_kmh = direction_ * sum / 4.0f;  // com sinal, como no Autoware

        state_.header.stamp    = now();
        state_.header.frame_id = "telemetry_node";
        state_.origin_stamp    = state_.header.stamp;
        state_.id              = seq_num_++;
        pub_telemetry_->publish(state_);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleTelemetryNode>());
    rclcpp::shutdown();
    return 0;
}
