#include <memory>
#include "rclcpp/rclcpp.hpp"

// Mensagens Autoware
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_report.hpp"
#include "autoware_vehicle_msgs/msg/hazard_lights_report.hpp"
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"

#include "msg_manual_teleop/msg/telemetry_state.hpp"
#include "msg_manual_teleop/msg/node_metrics.hpp"

#include "std_msgs/msg/int8.hpp"


using VelocityReport       = autoware_vehicle_msgs::msg::VelocityReport;
using GearReport           = autoware_vehicle_msgs::msg::GearReport;
using TurnIndicatorsReport = autoware_vehicle_msgs::msg::TurnIndicatorsReport;
using HazardLightsReport   = autoware_vehicle_msgs::msg::HazardLightsReport;
using OperationModeState   = autoware_adapi_v1_msgs::msg::OperationModeState;
using TelemetryState       = msg_manual_teleop::msg::TelemetryState;
using Metrics              = msg_manual_teleop::msg::NodeMetrics;
using Int8    = std_msgs::msg::Int8;


class TelemetrySubscriber : public rclcpp::Node {
public:
    TelemetrySubscriber() : Node("autoware_telemetry_node") {
        pub_telemetry_ = create_publisher<TelemetryState>("/telemetry/state", 10);

        sub_velocity_ = create_subscription<VelocityReport>(
            "/vehicle/status/velocity_status", 10,
            [this](const VelocityReport::SharedPtr msg) {
                state_.velocity_kmh = msg->longitudinal_velocity * 3.6f;
            });

        sub_gear_ = create_subscription<GearReport>(
            "/vehicle/status/gear_status", 10,
            [this](const GearReport::SharedPtr msg) {
                switch (msg->report){
                    case GearReport::PARK: state_.gear = 0; break;
                    case GearReport::DRIVE: state_.gear = 1; break;
                    case GearReport::REVERSE: state_.gear = 2; break;
                    default: break;
                }
            });

        sub_operation_mode_ = create_subscription<OperationModeState>(
            "/system/operation_mode/state", 10,
            [this](const OperationModeState::SharedPtr msg) {
                state_.mode    = msg->mode;
                state_.engaged = msg->is_autoware_control_enabled;
            });

        sub_turn_indicators_ = create_subscription<TurnIndicatorsReport>(
            "/vehicle/status/turn_indicators_status", 10,
            [this](const TurnIndicatorsReport::SharedPtr msg) {
                state_.turn_signal = msg->report;
            });

        sub_hazard_lights_ = create_subscription<HazardLightsReport>(
            "/vehicle/status/hazard_lights_status", 10,
            [this](const HazardLightsReport::SharedPtr msg) {
                state_.hazard = msg->report;
            });

        sub_latency_ = create_subscription<Metrics>(
            "/metrics/e2e_command_latency", 10,
            [this](const Metrics::SharedPtr msg){
                state_.e2e_command_ms = msg->latency_ms;
            });

        sub_network_state_ = create_subscription<Int8>(
            "/teleop/safety_state", 10,
            [this](const Int8::SharedPtr msg){
                state_.network_state = msg->data;
            });

        // Publica a 50Hz com o estado mais recente de todos os campos
        timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            [this]() { 
                publish_and_log(); 
            });
    }

private:
    TelemetryState state_{};
    rclcpp::Publisher<TelemetryState>::SharedPtr          pub_telemetry_;
    rclcpp::Subscription<VelocityReport>::SharedPtr       sub_velocity_;
    rclcpp::Subscription<GearReport>::SharedPtr           sub_gear_;
    rclcpp::Subscription<OperationModeState>::SharedPtr   sub_operation_mode_;
    rclcpp::Subscription<TurnIndicatorsReport>::SharedPtr sub_turn_indicators_;
    rclcpp::Subscription<HazardLightsReport>::SharedPtr   sub_hazard_lights_;
    rclcpp::Subscription<Metrics>::SharedPtr              sub_latency_;
    rclcpp::Subscription<Int8>::SharedPtr                 sub_network_state_;
    rclcpp::TimerBase::SharedPtr timer_;

    uint32_t seq_num_ = 1;  

    void publish_and_log() {
        state_.header.stamp    = this->now();
        state_.header.frame_id = "telemetry_node";
        state_.origin_stamp    = state_.header.stamp;
        state_.id              = seq_num_++;
        pub_telemetry_->publish(state_);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TelemetrySubscriber>());
    rclcpp::shutdown();
    return 0;
}