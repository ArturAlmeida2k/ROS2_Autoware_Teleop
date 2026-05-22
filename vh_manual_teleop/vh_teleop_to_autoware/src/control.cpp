#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include <algorithm>
#include <memory>
#include <cmath>

#include "msg_manual_teleop/msg/teleop_command.hpp"
#include <autoware_control_msgs/msg/control.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>

using Control               = autoware_control_msgs::msg::Control;
using GateMode              = tier4_control_msgs::msg::GateMode;
using GearCommand           = autoware_vehicle_msgs::msg::GearCommand;
using Bool                  = std_msgs::msg::Bool;
using VelocityReport        = autoware_vehicle_msgs::msg::VelocityReport;
using TurnIndicatorsCommand = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using HazardLightsCommand   = autoware_vehicle_msgs::msg::HazardLightsCommand;
using TeleopCommand         = msg_manual_teleop::msg::TeleopCommand;
using OperationModeState    = autoware_adapi_v1_msgs::msg::OperationModeState;

class AutowareControllerNode : public rclcpp::Node
{
public:
    AutowareControllerNode() : Node("autoware_controller_node")
    {
        sub_teleop_command_ = this->create_subscription<TeleopCommand>(
            "/teleop/command", 10,
            std::bind(&AutowareControllerNode::teleop_command_callback, this, std::placeholders::_1));

        sub_vlc_current_ = this->create_subscription<VelocityReport>(
            "/vehicle/status/velocity_status", 10,
            std::bind(&AutowareControllerNode::vlc_report_callback, this, std::placeholders::_1));

        sub_operation_mode_ = this->create_subscription<OperationModeState>(
            "/system/operation_mode/state", 10,
            std::bind(&AutowareControllerNode::operation_mode_callback, this, std::placeholders::_1));

        pub_control_cmd_     = this->create_publisher<Control>("/teleop/internal/control_cmd", rclcpp::QoS(1));
        pub_gear_cmd_        = this->create_publisher<GearCommand>("/teleop/internal/gear_cmd", 1);
        pub_turn_indicators_ = this->create_publisher<TurnIndicatorsCommand>("/teleop/internal/turn_indicators_cmd", 1);
        pub_hazard_lights_   = this->create_publisher<HazardLightsCommand>("/teleop/internal/hazard_lights_cmd", 1);
        pub_engage_cmd_      = this->create_publisher<Bool>("/teleop/internal/engage_cmd", 1);
        pub_gate_mode_       = this->create_publisher<GateMode>("/teleop/internal/gate_mode_cmd", rclcpp::QoS(1));

        RCLCPP_INFO(this->get_logger(), "Autoware Controller Node started.");
    }

private:
    float   vlc_target_            = 0.0f;
    float   vlc_current_           = 0.0f;
    float   steering_angle_target_ = 0.0f;
    float   brake_factor_          = 0.0f;
    int     gear_change_           = GearCommand::REVERSE;
    bool    last_engage_cmd_       = false;

    uint8_t current_mode_ = OperationModeState::UNKNOWN;

    const double BASE_KP_GAIN = 0.5;
    const double BRAKE_KP_MAX = 5.0;
    const double MAX_ACCEL    = 1.0;
    const double MAX_DECEL    = 5.0;

    rclcpp::Publisher<GateMode>::SharedPtr              pub_gate_mode_;
    rclcpp::Publisher<Control>::SharedPtr               pub_control_cmd_;
    rclcpp::Publisher<GearCommand>::SharedPtr           pub_gear_cmd_;
    rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr pub_turn_indicators_;
    rclcpp::Publisher<HazardLightsCommand>::SharedPtr   pub_hazard_lights_;
    rclcpp::Publisher<Bool>::SharedPtr                  pub_engage_cmd_;

    rclcpp::Subscription<TeleopCommand>::SharedPtr      sub_teleop_command_;
    rclcpp::Subscription<VelocityReport>::SharedPtr     sub_vlc_current_;
    rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_;

    void operation_mode_callback(const OperationModeState::SharedPtr msg)
    {
        current_mode_ = msg->mode;
        RCLCPP_DEBUG(this->get_logger(), "Operation mode: %d", current_mode_);
    }

    void vlc_report_callback(const VelocityReport::SharedPtr msg)
    {
        vlc_current_ = msg->longitudinal_velocity;
    }

    void teleop_command_callback(const TeleopCommand::SharedPtr msg)
    {
        vlc_target_            = msg->target_velocity;
        steering_angle_target_ = msg->target_steering_angle;
        brake_factor_          = msg->brake_factor;

        // --- Engage: edge detection ---
        bool engage_cmd = msg->engage_command;
        if (engage_cmd != last_engage_cmd_) {
            RCLCPP_INFO(this->get_logger(), "Engage command: %s", engage_cmd ? "TRUE" : "FALSE");

            GateMode gate_msg;
            gate_msg.data = GateMode::EXTERNAL;
            pub_gate_mode_->publish(gate_msg);

            Bool engage_msg;
            engage_msg.data = engage_cmd;
            pub_engage_cmd_->publish(engage_msg);

            last_engage_cmd_ = engage_cmd;
        }

        if (current_mode_ == OperationModeState::LOCAL || current_mode_ == OperationModeState::REMOTE) {
        // --- Sinais luminosos ---
            TurnIndicatorsCommand turn_cmd;
            turn_cmd.stamp = this->now();
            HazardLightsCommand hazard_cmd;
            hazard_cmd.stamp = this->now();

            if (msg->turn_signal == 4) {
                turn_cmd.command   = TurnIndicatorsCommand::DISABLE;
                hazard_cmd.command = HazardLightsCommand::ENABLE;
            } else {
                turn_cmd.command   = msg->turn_signal;
                hazard_cmd.command = HazardLightsCommand::DISABLE;
            }

            pub_turn_indicators_->publish(turn_cmd);
            pub_hazard_lights_->publish(hazard_cmd);
        

        // --- Gear (só em LOCAL ou REMOTE) ---
            gear_change_ = msg->gear;
            GearCommand gear_cmd;
            gear_cmd.stamp = this->now();
            switch (gear_change_) {
                case 0: gear_cmd.command = GearCommand::PARK;    break;
                case 1: gear_cmd.command = GearCommand::DRIVE;   break;
                case 2: gear_cmd.command = GearCommand::REVERSE; break;
            }
            pub_gear_cmd_->publish(gear_cmd);
        

        // --- Controlo (só em LOCAL ou REMOTE) ---
            auto control_cmd = std::make_unique<Control>();
            control_cmd->stamp = this->now();
            control_cmd->longitudinal.velocity = vlc_target_;

            double velocity_error   = static_cast<double>(vlc_target_) - std::abs(vlc_current_);
            double acceleration_cmd = 0.0;

            if (vlc_target_ <= 0.01f && brake_factor_ > 0.01f) {
                double dynamic_kp = BASE_KP_GAIN + (BRAKE_KP_MAX - BASE_KP_GAIN) * brake_factor_;
                acceleration_cmd  = std::clamp(dynamic_kp * velocity_error, -MAX_DECEL, 0.0);
            } else {
                acceleration_cmd  = std::clamp(BASE_KP_GAIN * velocity_error, -MAX_ACCEL, MAX_ACCEL);
            }

            if (std::abs(acceleration_cmd)       < 0.0001) acceleration_cmd       = 0.0;
            if (std::abs(steering_angle_target_) < 0.0001) steering_angle_target_ = 0.0f;

            control_cmd->longitudinal.acceleration   = acceleration_cmd;
            control_cmd->lateral.steering_tire_angle = steering_angle_target_;

            pub_control_cmd_->publish(std::move(control_cmd));
        }

    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutowareControllerNode>());
    rclcpp::shutdown();
    return 0;
}