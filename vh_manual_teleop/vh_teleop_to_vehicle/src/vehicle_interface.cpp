#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "teleop_msgs/msg/vehicle_command.hpp"
#include "teleop_msgs/msg/command_enums.hpp"

using Float32        = std_msgs::msg::Float32;
using UInt8          = std_msgs::msg::UInt8;
using VehicleCommand = teleop_msgs::msg::VehicleCommand;
using CmdEnums       = teleop_msgs::msg::CommandEnums;

// Modos (faz o papel do Autoware):
//   LOCAL  -> o condutor de segurança tem o carro, não se envia nada ao carro.
//             É o estado inicial e o estado depois de o condutor carregar no travão.
//   REMOTE -> o operador conduz.
//   STOP   -> o operador desligou o engage, o carro é travado e mantido parado.
class VehicleInterfaceNode : public rclcpp::Node
{
public:
    VehicleInterfaceNode() : Node("vehicle_interface")
    {
        max_accel_     = declare_parameter<double>("max_accel", 1.0);
        max_decel_     = declare_parameter<double>("max_decel", 2.0);
        max_torque_    = declare_parameter<double>("max_steering_torque", 2.0);
        stop_decel_    = declare_parameter<double>("stop_decel", 2.0);
        cmd_timeout_s_ = declare_parameter<double>("command_timeout", 0.5);
        const double rate_hz = declare_parameter<double>("rate", 100.0);

        sub_cmd_ = create_subscription<VehicleCommand>(
            "/teleop/vehicle_command", 10,
            std::bind(&VehicleInterfaceNode::command_callback, this, std::placeholders::_1));

        sub_brake_ = create_subscription<Float32>(
            "/vehicle/brake_pressed", 10,
            std::bind(&VehicleInterfaceNode::brake_pressed_callback, this, std::placeholders::_1));

        sub_wheel_[0] = create_subscription<Float32>("/vehicle/wheel_speed_fl", 10,
            [this](const Float32::SharedPtr msg) { wheel_kmh_[0] = msg->data; });
        sub_wheel_[1] = create_subscription<Float32>("/vehicle/wheel_speed_fr", 10,
            [this](const Float32::SharedPtr msg) { wheel_kmh_[1] = msg->data; });
        sub_wheel_[2] = create_subscription<Float32>("/vehicle/wheel_speed_rl", 10,
            [this](const Float32::SharedPtr msg) { wheel_kmh_[2] = msg->data; });
        sub_wheel_[3] = create_subscription<Float32>("/vehicle/wheel_speed_rr", 10,
            [this](const Float32::SharedPtr msg) { wheel_kmh_[3] = msg->data; });

        pub_accel_  = create_publisher<Float32>("/control/desired_acceleration", 1);
        pub_torque_ = create_publisher<Float32>("/control/desired_steering_torque", 1);
        pub_speed_  = create_publisher<Float32>("/control/indicated_speed", 1);

        // modo para o control e o vehicle_telemetry (substitui o /system/operation_mode/state do Autoware)
        pub_mode_ = create_publisher<UInt8>("/teleop/vehicle_mode", rclcpp::QoS(1).reliable().transient_local());

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate_hz),
            std::bind(&VehicleInterfaceNode::loop, this));

        publish_mode();
        RCLCPP_INFO(get_logger(), "Vehicle interface started at %.0f Hz (mode LOCAL).", rate_hz);
    }

private:
    double max_accel_, max_decel_, max_torque_, stop_decel_, cmd_timeout_s_;

    std::optional<VehicleCommand> last_cmd_;
    std::optional<rclcpp::Time> last_cmd_time_;
    bool last_engage_cmd_ = false;

    bool brake_pressed_ = false;
    float wheel_kmh_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    uint8_t mode_ = CmdEnums::OPERATION_MODE_LOCAL;

    rclcpp::Subscription<VehicleCommand>::SharedPtr sub_cmd_;
    rclcpp::Subscription<Float32>::SharedPtr sub_brake_;
    rclcpp::Subscription<Float32>::SharedPtr sub_wheel_[4];
    rclcpp::Publisher<Float32>::SharedPtr pub_accel_, pub_torque_, pub_speed_;
    rclcpp::Publisher<UInt8>::SharedPtr   pub_mode_;
    rclcpp::TimerBase::SharedPtr timer_;

    void set_mode(uint8_t mode, const char *reason)
    {
        if (mode == mode_) return;
        mode_ = mode;
        publish_mode();
        RCLCPP_INFO(get_logger(), "Mode -> %s (%s)", mode == CmdEnums::OPERATION_MODE_REMOTE ? "REMOTE" : mode == CmdEnums::OPERATION_MODE_LOCAL ? "LOCAL" : "STOP", reason);
    }

    void publish_mode()
    {
        UInt8 msg;
        msg.data = mode_;
        pub_mode_->publish(msg);
    }

    // o condutor de segurança carregou no travão: devolve-lhe o carro
    void brake_pressed_callback(const Float32::SharedPtr msg)
    {
        brake_pressed_ = msg->data > 0.1f;
        if (brake_pressed_ && mode_ != CmdEnums::OPERATION_MODE_LOCAL) {
            set_mode(CmdEnums::OPERATION_MODE_LOCAL, "driver pressed the brake");
        }
    }

    void command_callback(const VehicleCommand::SharedPtr msg)
    {
        last_cmd_ = *msg;
        last_cmd_time_ = now();

        // o engage só atua quando muda
        const bool engage = msg->engage;
        if (engage != last_engage_cmd_) {
            if (engage) {
                if (brake_pressed_) {
                    RCLCPP_WARN(get_logger(), "Engage refused: brake pedal is pressed.");
                } else {
                    set_mode(CmdEnums::OPERATION_MODE_REMOTE, "engage requested");
                }
            } else if (mode_ == CmdEnums::OPERATION_MODE_REMOTE) {
                set_mode(CmdEnums::OPERATION_MODE_STOP, "disengage requested");
            }
            last_engage_cmd_ = engage;
        }
    }

    void loop()
    {
        // velocidade para o painel de instrumentos
        float sum = 0.0f;
        for (float w : wheel_kmh_) sum += std::abs(w);
        Float32 speed;
        speed.data = sum / 4.0f;
        pub_speed_->publish(speed);

        Float32 accel;
        Float32 torque;

        switch (mode_) {
            case CmdEnums::OPERATION_MODE_REMOTE: {
                const bool stale = !last_cmd_time_ || (now() - *last_cmd_time_).seconds() > cmd_timeout_s_;
                if (stale) {
                    // o control deixou de enviar: trava e solta o volante
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No command from control, braking.");
                    accel.data  = static_cast<float>(-stop_decel_);
                    torque.data = 0.0f;
                } else {
                    accel.data  = static_cast<float>(std::clamp<double>(last_cmd_->acceleration, -max_decel_, max_accel_));
                    torque.data = static_cast<float>(std::clamp<double>(last_cmd_->steering_torque, -max_torque_, max_torque_));
                }
                break;
            }

            case CmdEnums::OPERATION_MODE_STOP:
                // mantém o carro travado e o volante livre
                accel.data  = static_cast<float>(-stop_decel_);
                torque.data = 0.0f;
                break;

            case CmdEnums::OPERATION_MODE_LOCAL:
            default:
                // o condutor manda, não envia nada ao carro
                return;
        }

        pub_accel_->publish(accel);
        pub_torque_->publish(torque);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}
