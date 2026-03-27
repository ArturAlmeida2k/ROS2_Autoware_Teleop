#include <memory>
#include "rclcpp/rclcpp.hpp"

// Mensagens Autoware
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_report.hpp"
#include "autoware_vehicle_msgs/msg/hazard_lights_report.hpp"
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"

// Mensagem custom
#include "vh_telemetry/msg/telemetry_state.hpp"

using VelocityReport       = autoware_vehicle_msgs::msg::VelocityReport;
using GearReport           = autoware_vehicle_msgs::msg::GearReport;
using TurnIndicatorsReport = autoware_vehicle_msgs::msg::TurnIndicatorsReport;
using HazardLightsReport   = autoware_vehicle_msgs::msg::HazardLightsReport;
using OperationModeState   = autoware_adapi_v1_msgs::msg::OperationModeState;
using TelemetryState       = vh_telemetry::msg::TelemetryState;

class TelemetrySubscriber : public rclcpp::Node {
public:
    TelemetrySubscriber() : Node("autoware_telemetry_node") {

        pub_telemetry_ = create_publisher<TelemetryState>("/telemetry/state", 10);

        sub_velocity_ = create_subscription<VelocityReport>(
            "/vehicle/status/velocity_status", 10,
            [this](const VelocityReport::SharedPtr msg) {
                state_.velocity_kmh = msg->longitudinal_velocity * 3.6f;
                publish_and_log();
            });

        sub_gear_ = create_subscription<GearReport>(
            "/vehicle/status/gear_status", 10,
            [this](const GearReport::SharedPtr msg) {
                state_.gear = msg->report;
                publish_and_log();
            });

        sub_operation_mode_ = create_subscription<OperationModeState>(
            "/system/operation_mode/state", 10,
            [this](const OperationModeState::SharedPtr msg) {
                state_.mode    = msg->mode;
                state_.engaged = msg->is_autoware_control_enabled;
                publish_and_log();
            });

        sub_turn_indicators_ = create_subscription<TurnIndicatorsReport>(
            "/vehicle/status/turn_indicators_status", 10,
            [this](const TurnIndicatorsReport::SharedPtr msg) {
                state_.turn_signal = msg->report;
                publish_and_log();
            });

        sub_hazard_lights_ = create_subscription<HazardLightsReport>(
            "/vehicle/status/hazard_lights_status", 10,
            [this](const HazardLightsReport::SharedPtr msg) {
                state_.hazard = msg->report;
                publish_and_log();
            });
    }

private:
    TelemetryState state_{};

    rclcpp::Publisher<TelemetryState>::SharedPtr pub_telemetry_;

    rclcpp::Subscription<VelocityReport>::SharedPtr       sub_velocity_;
    rclcpp::Subscription<GearReport>::SharedPtr           sub_gear_;
    rclcpp::Subscription<OperationModeState>::SharedPtr   sub_operation_mode_;
    rclcpp::Subscription<TurnIndicatorsReport>::SharedPtr sub_turn_indicators_;
    rclcpp::Subscription<HazardLightsReport>::SharedPtr   sub_hazard_lights_;

    void publish_and_log() {
        pub_telemetry_->publish(state_);

        const char* gear_str = "UNKNOWN";
        switch (state_.gear) {
            case GearReport::PARK:    gear_str = "PARK";    break;
            case GearReport::DRIVE:   gear_str = "DRIVE";   break;
            case GearReport::REVERSE: gear_str = "REVERSE"; break;
            case GearReport::NEUTRAL: gear_str = "NEUTRAL"; break;
        }

        const char* turn_str = "OFF";
        if      (state_.turn_signal == TurnIndicatorsReport::ENABLE_LEFT)  turn_str = "LEFT";
        else if (state_.turn_signal == TurnIndicatorsReport::ENABLE_RIGHT) turn_str = "RIGHT";

        RCLCPP_INFO(get_logger(),
            "Vel: %.1f km/h | Gear: %s | Mode: %d | Engaged: %s | Lights: %s | Hazard: %s",
            state_.velocity_kmh,
            gear_str,
            state_.mode,
            state_.engaged ? "YES" : "NO",
            turn_str,
            state_.hazard == HazardLightsReport::ENABLE ? "ON" : "OFF"
        );
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TelemetrySubscriber>());
    rclcpp::shutdown();
    return 0;
}