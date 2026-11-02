#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "teleop_msgs/msg/vehicle_command.hpp"
#include "teleop_msgs/msg/command_enums.hpp"

using Float32        = std_msgs::msg::Float32;
using VehicleCommand = teleop_msgs::msg::VehicleCommand;
using CmdEnums       = teleop_msgs::msg::CommandEnums;

// Nó entre o control e o carro. Envia ao carro o último comando do control a ritmo fixo
// e aplica as proteções que não podem depender da chegada de comandos:
//   - limites da interface do carro (aceleração de -2 a 1 m/s2, binário de -2 a 2 Nm)
//   - travagem se o control deixar de enviar comandos
//   - não envia nada se o condutor de segurança carregar no travão
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
            [this](const VehicleCommand::SharedPtr msg) {
                last_cmd_ = *msg;
                last_cmd_time_ = now();
            });

        sub_brake_ = create_subscription<Float32>(
            "/vehicle/brake_pressed", 10,
            [this](const Float32::SharedPtr msg) { brake_pressed_ = msg->data > 0.5f; });

        auto wheel_cb = [this](int i) {
            return [this, i](const Float32::SharedPtr msg) { wheel_kmh_[i] = msg->data; };
        };
        sub_wheel_[0] = create_subscription<Float32>("/vehicle/wheel_speed_fl", 10, wheel_cb(0));
        sub_wheel_[1] = create_subscription<Float32>("/vehicle/wheel_speed_fr", 10, wheel_cb(1));
        sub_wheel_[2] = create_subscription<Float32>("/vehicle/wheel_speed_rl", 10, wheel_cb(2));
        sub_wheel_[3] = create_subscription<Float32>("/vehicle/wheel_speed_rr", 10, wheel_cb(3));

        pub_accel_  = create_publisher<Float32>("/control/desired_acceleration", 1);
        pub_torque_ = create_publisher<Float32>("/control/desired_steering_torque", 1);
        pub_speed_  = create_publisher<Float32>("/control/indicated_speed", 1);

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate_hz),
            std::bind(&VehicleInterfaceNode::loop, this));

        RCLCPP_INFO(get_logger(), "Vehicle interface started at %.0f Hz.", rate_hz);
    }

private:
    double max_accel_, max_decel_, max_torque_, stop_decel_, cmd_timeout_s_;

    std::optional<VehicleCommand> last_cmd_;
    std::optional<rclcpp::Time> last_cmd_time_;
    bool brake_pressed_ = false;
    float wheel_kmh_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    rclcpp::Subscription<VehicleCommand>::SharedPtr sub_cmd_;
    rclcpp::Subscription<Float32>::SharedPtr sub_brake_;
    rclcpp::Subscription<Float32>::SharedPtr sub_wheel_[4];
    rclcpp::Publisher<Float32>::SharedPtr pub_accel_, pub_torque_, pub_speed_;
    rclcpp::TimerBase::SharedPtr timer_;

    void loop()
    {
        // velocidade para o painel de instrumentos
        float sum = 0.0f;
        for (float w : wheel_kmh_) sum += std::abs(w);
        Float32 speed;
        speed.data = sum / 4.0f;
        pub_speed_->publish(speed);

        // o condutor tem o carro, ou ainda não há comandos
        if (brake_pressed_ || !last_cmd_ || last_cmd_->mode == CmdEnums::OPERATION_MODE_LOCAL) {
            return;
        }

        Float32 accel;
        Float32 torque;

        const bool stale = (now() - *last_cmd_time_).seconds() > cmd_timeout_s_;
        if (stale) {
            // o control deixou de enviar: trava e solta o volante
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No command from control, braking.");
            accel.data  = static_cast<float>(-stop_decel_);
            torque.data = 0.0f;
        } else {
            accel.data  = static_cast<float>(std::clamp<double>(last_cmd_->acceleration, -max_decel_, max_accel_));
            torque.data = static_cast<float>(std::clamp<double>(last_cmd_->steering_torque, -max_torque_, max_torque_));
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
