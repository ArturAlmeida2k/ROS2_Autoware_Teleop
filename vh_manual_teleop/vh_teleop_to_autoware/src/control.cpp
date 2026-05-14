#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include <algorithm>
#include <memory>
#include <cmath>

#include "msg_manual_teleop/msg/teleop_command.hpp"

#include <autoware_control_msgs/msg/control.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>

using Control = autoware_control_msgs::msg::Control;
using GateMode = tier4_control_msgs::msg::GateMode;
using EngageSrv = tier4_external_api_msgs::srv::Engage;
using GearCommand = autoware_vehicle_msgs::msg::GearCommand;
using Float32 = std_msgs::msg::Float32;
using Bool = std_msgs::msg::Bool;
using Int32 = std_msgs::msg::Int32;
using String = std_msgs::msg::String;
using VelocityReport = autoware_vehicle_msgs::msg::VelocityReport;
using TurnIndicatorsCommand = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using HazardLightsCommand = autoware_vehicle_msgs::msg::HazardLightsCommand;

using TeleopCommand = msg_manual_teleop::msg::TeleopCommand;

class AutowareControllerNode : public rclcpp::Node
{
public:
    AutowareControllerNode() : Node("autoware_controller_node")
    {
        // --- Teleop subscriptions ---
        sub_teleop_command_ = this->create_subscription<TeleopCommand>(
            "/teleop/command", 10,
            std::bind(&AutowareControllerNode::teleop_command_callback, this, std::placeholders::_1));

        // --- Vehicle status subscription ---
        sub_vlc_current_ = this->create_subscription<VelocityReport>(
            "/vehicle/status/velocity_status", 10,
            std::bind(&AutowareControllerNode::vlc_report_callback, this, std::placeholders::_1));

        // --- Publishers and service clients (Autoware interface) ---
        client_engage_ = this->create_client<EngageSrv>("/api/autoware/set/engage");
        pub_gate_mode_ = this->create_publisher<GateMode>("/control/gate_mode_cmd", rclcpp::QoS(1));

        // --- Publishers for the Safety Gate ---
        pub_control_cmd_ = this->create_publisher<Control>("/teleop/internal/control_cmd", rclcpp::QoS(1));
        pub_gear_cmd_ = this->create_publisher<GearCommand>("/teleop/internal/gear_cmd", 1);
        pub_turn_indicators_ = this->create_publisher<TurnIndicatorsCommand>("/teleop/internal/turn_indicators_cmd", 1);
        pub_hazard_lights_ = this->create_publisher<HazardLightsCommand>("/teleop/internal/hazard_lights_cmd", 1);

        // --- Control loop timer (50 Hz) ---
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&AutowareControllerNode::publish_control_command, this));

        RCLCPP_INFO(this->get_logger(), "Autoware Controller Node started. Waiting for /teleop commands...");
    }


private:
  // --- Teleop inputs ---
  float vlc_target_              = 0.0f;
  float vlc_current_             = 0.0f;
  float steering_angle_target_   = 0.0f;
  bool  engage_target_           = false;
  bool  last_engage_target_      = false;
  float brake_factor_            = 0.0f;
  int   gear_change_             = 2;
  int   turn_signal_             = TurnIndicatorsCommand::DISABLE;

  // --- Control constants ---
  const double BASE_KP_GAIN = 0.5;
  const double BRAKE_KP_MAX = 5.0;
  const double MAX_ACCEL    = 1.0;
  const double MAX_DECEL    = 5.0;

  // --- ROS 2 interfaces ---
  rclcpp::Publisher<GateMode>::SharedPtr            pub_gate_mode_;
  rclcpp::Publisher<Control>::SharedPtr             pub_control_cmd_;
  rclcpp::Publisher<GearCommand>::SharedPtr         pub_gear_cmd_;
  rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr pub_turn_indicators_;
  rclcpp::Publisher<HazardLightsCommand>::SharedPtr   pub_hazard_lights_;

  rclcpp::Subscription<TeleopCommand>::SharedPtr    sub_teleop_command_;
  rclcpp::Subscription<VelocityReport>::SharedPtr   sub_vlc_current_;

  rclcpp::TimerBase::SharedPtr                      control_timer_;
  rclcpp::Client<EngageSrv>::SharedPtr              client_engage_;

  void teleop_command_callback(const TeleopCommand::SharedPtr msg)
  {
    // --- Velocity & steering ---
    vlc_target_            = msg->target_velocity;
    steering_angle_target_ = msg->target_steering_angle;
    brake_factor_          = msg->brake_factor;

    // --- Turn signal / hazard lights ---
    turn_signal_ = msg->turn_signal;
    {
      TurnIndicatorsCommand turn_cmd;
      turn_cmd.stamp = this->now();
      HazardLightsCommand hazard_cmd;
      hazard_cmd.stamp = this->now();

      if (turn_signal_ == 4) {                          
        turn_cmd.command   = TurnIndicatorsCommand::DISABLE;
        hazard_cmd.command = HazardLightsCommand::ENABLE;
      } else {
        turn_cmd.command   = turn_signal_;
        hazard_cmd.command = HazardLightsCommand::DISABLE;
      }

      pub_turn_indicators_->publish(turn_cmd);
      pub_hazard_lights_->publish(hazard_cmd);
    }

    // --- Gear change (only when engaged) ---
    if (engage_target_) {
      gear_change_ = msg->gear;
      GearCommand gear_cmd;
      gear_cmd.stamp = this->now();
      switch (gear_change_) {
        case 0: gear_cmd.command = GearCommand::PARK;    break;
        case 1: gear_cmd.command = GearCommand::DRIVE;   break;
        case 2: gear_cmd.command = GearCommand::REVERSE; break;
      }
      pub_gear_cmd_->publish(gear_cmd);
    }

    // --- Engage (only on rising/falling edge) ---
    engage_target_ = msg->engage_command;
    if (engage_target_ != last_engage_target_) {
      RCLCPP_INFO(this->get_logger(), "Engage command received: %s",
                  engage_target_ ? "TRUE" : "FALSE");
      set_autoware_engage(engage_target_);
      last_engage_target_ = engage_target_;
    }
  }

  // --- Vehicle velocity feedback ---
  void vlc_report_callback(const VelocityReport::SharedPtr msg)
  {
    vlc_current_ = msg->longitudinal_velocity;
  }

  // --- Engage command handling ---
  void set_autoware_engage(bool engage)
  {
    pub_gate_mode_->publish(
      tier4_control_msgs::build<GateMode>().data(GateMode::EXTERNAL));

    auto req = std::make_shared<EngageSrv::Request>();
    req->engage = engage;

    if (!client_engage_->service_is_ready()) {
      RCLCPP_ERROR(this->get_logger(), "Service /api/autoware/set/engage unavailable.");
      return;
    }

    client_engage_->async_send_request(
      req, []([[maybe_unused]] rclcpp::Client<EngageSrv>::SharedFuture) {});
  }

  // --- Main control loop (50 Hz) ---
  void publish_control_command()
  {
    if (!engage_target_) return;

    auto control_cmd = std::make_unique<Control>();
    control_cmd->stamp = this->now();

    control_cmd->longitudinal.velocity = vlc_target_;

    double velocity_error   = static_cast<double>(vlc_target_) - std::abs(vlc_current_);
    double acceleration_cmd = 0.0;

    if (vlc_target_ <= 0.01f && brake_factor_ > 0.01f) {
      double dynamic_kp  = BASE_KP_GAIN + (BRAKE_KP_MAX - BASE_KP_GAIN) * brake_factor_;
      acceleration_cmd   = dynamic_kp * velocity_error;
      acceleration_cmd   = std::clamp(acceleration_cmd, -MAX_DECEL, 0.0);
    } else {
      acceleration_cmd = BASE_KP_GAIN * velocity_error;
      acceleration_cmd = std::clamp(acceleration_cmd, -MAX_ACCEL, MAX_ACCEL);
    }

    if (std::abs(acceleration_cmd)       < 0.0001) acceleration_cmd       = 0.0;
    if (std::abs(steering_angle_target_) < 0.0001) steering_angle_target_ = 0.0f;

    control_cmd->longitudinal.acceleration      = acceleration_cmd;
    control_cmd->lateral.steering_tire_angle    = steering_angle_target_;

    pub_control_cmd_->publish(std::move(control_cmd));
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutowareControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}