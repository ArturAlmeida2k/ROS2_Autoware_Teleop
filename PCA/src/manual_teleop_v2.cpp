#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include <algorithm>
#include <memory>
#include <cmath>

#include <autoware_control_msgs/msg/control.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>

using Control = autoware_control_msgs::msg::Control;
using GateMode = tier4_control_msgs::msg::GateMode;
using EngageSrv = tier4_external_api_msgs::srv::Engage;
using GearCommand = autoware_vehicle_msgs::msg::GearCommand;
using Float32 = std_msgs::msg::Float32;
using Bool = std_msgs::msg::Bool;
using Int32 = std_msgs::msg::Int32;
using String = std_msgs::msg::String;
using VelocityReport = autoware_vehicle_msgs::msg::VelocityReport;

class AutowareControllerNode : public rclcpp::Node
{
public:
    AutowareControllerNode() : Node("autoware_controller_node")
    {
        // --- Teleop subscriptions ---
        sub_vlc_target_ = this->create_subscription<Float32>(
            "/teleop/target_velocity", 10,
            std::bind(&AutowareControllerNode::vlc_target_callback, this, std::placeholders::_1));

        sub_steering_target_ = this->create_subscription<Float32>(
            "/teleop/target_steering_angle", 10,
            std::bind(&AutowareControllerNode::steering_target_callback, this, std::placeholders::_1));

        sub_engage_target_ = this->create_subscription<Bool>(
            "/teleop/engage_command", 1,
            std::bind(&AutowareControllerNode::engage_callback, this, std::placeholders::_1));

        sub_brake_factor_ = this->create_subscription<Float32>(
            "/teleop/brake_factor", 10,
            std::bind(&AutowareControllerNode::brake_factor_callback, this, std::placeholders::_1));

        sub_gear_change_ = this->create_subscription<Int32>(
            "/teleop/gear_change", 10,
            std::bind(&AutowareControllerNode::gear_change_callback, this, std::placeholders::_1));


        // --- Vehicle status subscription ---
        sub_vlc_current_ = this->create_subscription<VelocityReport>(
            "/vehicle/status/velocity_status", 10,
            std::bind(&AutowareControllerNode::vlc_report_callback, this, std::placeholders::_1));

        // --- Publishers and service clients (Autoware interface) ---
        client_engage_ = this->create_client<EngageSrv>("/api/autoware/set/engage");
        pub_gate_mode_ = this->create_publisher<GateMode>("/control/gate_mode_cmd", rclcpp::QoS(1));
        pub_control_cmd_ = this->create_publisher<Control>("/external/selected/control_cmd", rclcpp::QoS(1));
        pub_gear_cmd_ = this->create_publisher<GearCommand>("/external/selected/gear_cmd", 1);

        // --- Control loop timer (50 Hz) ---
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&AutowareControllerNode::publish_control_command, this));

        RCLCPP_INFO(this->get_logger(), "Autoware Controller Node started. Waiting for /teleop commands...");
    }

private:
    // --- Teleop inputs ---
    float vlc_target_ = 0.0f;
    float vlc_current_ = 0.0f;
    float steering_angle_target_ = 0.0f;
    bool engage_target_ = false;
    bool last_engage_target = false;
    float brake_factor_ = 0.0f;
    int gear_change_ = 2;

    // --- Control constants ---
    const double BASE_KP_GAIN = 0.5;   // Base proportional gain
    const double BRAKE_KP_MAX = 5.0;   // Maximum brake proportional gain
    const double MAX_ACCEL = 1.0;      // Max allowed acceleration (m/s²)
    const double MAX_DECEL = 5.0;      // Max allowed deceleration (m/s²)

    // --- ROS 2 interfaces ---
    rclcpp::Publisher<GateMode>::SharedPtr pub_gate_mode_;
    rclcpp::Publisher<Control>::SharedPtr pub_control_cmd_;
    rclcpp::Publisher<GearCommand>::SharedPtr pub_gear_cmd_;
    rclcpp::Publisher<String>::SharedPtr pub_external_select_;
    rclcpp::Subscription<Float32>::SharedPtr sub_vlc_target_, sub_steering_target_, sub_brake_factor_;
    rclcpp::Subscription<Bool>::SharedPtr sub_engage_target_;
    rclcpp::Subscription<VelocityReport>::SharedPtr sub_vlc_current_;
    rclcpp::Subscription<Int32>::SharedPtr sub_gear_change_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Client<EngageSrv>::SharedPtr client_engage_;

    // --- Teleop Callbacks ---
    void vlc_target_callback(const Float32::SharedPtr msg)
    {
        vlc_target_ = msg->data;
    }

    void vlc_report_callback(const VelocityReport::SharedPtr msg)
    {
        vlc_current_ = msg->longitudinal_velocity;
    }

    void steering_target_callback(const Float32::SharedPtr msg)
    {
        steering_angle_target_ = msg->data;
    }

    void brake_factor_callback(const Float32::SharedPtr msg)
    {
        brake_factor_ = msg->data;
    }

    // --- Gear change handling ---
    void gear_change_callback(const Int32::SharedPtr msg){
        if (engage_target_){
            GearCommand gear_cmd;
            gear_cmd.stamp = this->now();
            gear_change_ = msg->data;
            switch (gear_change_)
            {
                case 0:
                    gear_cmd.command = GearCommand::PARK;
                    break;
                case 1:
                    gear_cmd.command = GearCommand::DRIVE;
                    break;
                case 2:
                    gear_cmd.command = GearCommand::REVERSE;
                    break;
            }
            pub_gear_cmd_->publish(gear_cmd);
        }
    }



    // --- Engage command handling ---    
    void set_autoware_engage(bool engage)
    {
        // 1. Gate Mode: EXTERNAL para ligar, AUTO para desligar
        auto gate_msg = std::make_unique<GateMode>();
        gate_msg->data = engage ? GateMode::EXTERNAL : GateMode::AUTO;
        pub_gate_mode_->publish(std::move(gate_msg));

        // 2. Seletor de Comando: Força o Autoware a ouvir a fonte "remote"
        if (engage) {
            auto select_msg = std::make_unique<String>();
            select_msg->data = "remote";
            pub_external_select_->publish(std::move(select_msg));
        }

        // 3. Engage Service
        if (client_engage_->service_is_ready()) {
            auto req = std::make_shared<EngageSrv::Request>();
            req->engage = engage;
            client_engage_->async_send_request(req, [](rclcpp::Client<EngageSrv>::SharedFuture){});
        }
    }

    void engage_callback(const Bool::SharedPtr msg)
    {
        engage_target_ = msg->data;
        if (engage_target_ != last_engage_target){
            RCLCPP_INFO(this->get_logger(), "Engage command received: %s", engage_target_ ? "TRUE" : "FALSE");
            this->set_autoware_engage(engage_target_);
            last_engage_target = engage_target_;
        }
    }

    // --- Main control loop (called by timer) ---
    void publish_control_command()
    {   
        if (!engage_target_) return;

        // Create control message
        auto control_cmd = std::make_unique<Control>();
        control_cmd->stamp = this->now();

        // Set target velocity
        control_cmd->longitudinal.velocity = vlc_target_;

        // Compute velocity error
        double velocity_error = static_cast<double>(vlc_target_) - abs(vlc_current_);
        double acceleration_cmd = 0.0;

        // --- Brake control logic ---
        if (vlc_target_ <= 0.01 && brake_factor_ > 0.01) {
            // Dynamic proportional gain between base and max
            double dynamic_kp = BASE_KP_GAIN + (BRAKE_KP_MAX - BASE_KP_GAIN) * brake_factor_;

            // Compute strong deceleration
            acceleration_cmd = dynamic_kp * velocity_error;

            // Clamp deceleration
            acceleration_cmd = std::clamp(acceleration_cmd, -MAX_DECEL, 0.0);

        } else {
            // --- Cruise / acceleration logic ---
            acceleration_cmd = BASE_KP_GAIN * velocity_error;

            // Clamp acceleration
            acceleration_cmd = std::clamp(acceleration_cmd, -MAX_ACCEL, MAX_ACCEL);
        }
        if (acceleration_cmd < 0.0001 && acceleration_cmd > -0.0001){
            acceleration_cmd = 0;
        }
        if (steering_angle_target_ < 0.0001 && steering_angle_target_ > -0.0001){
            steering_angle_target_ = 0;
        }
        // Assign acceleration command
        control_cmd->longitudinal.acceleration = acceleration_cmd;

        // Set steering command
        control_cmd->lateral.steering_tire_angle = steering_angle_target_;

        // Publish control command
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
