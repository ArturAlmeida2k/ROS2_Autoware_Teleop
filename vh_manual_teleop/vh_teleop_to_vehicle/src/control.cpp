#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "teleop_msgs/msg/teleop_command.hpp"
#include "teleop_msgs/msg/vehicle_command.hpp"
#include "teleop_msgs/msg/node_metrics.hpp"
#include "teleop_msgs/msg/command_enums.hpp"

using Float32        = std_msgs::msg::Float32;
using UInt8          = std_msgs::msg::UInt8;
using TeleopCommand  = teleop_msgs::msg::TeleopCommand;
using VehicleCommand = teleop_msgs::msg::VehicleCommand;
using Metrics        = teleop_msgs::msg::NodeMetrics;
using CmdEnums       = teleop_msgs::msg::CommandEnums;

class ControlNode : public rclcpp::Node
{
public:
    ControlNode() : Node("control")
    {
        // parâmetros (os do volante têm de ser confirmados no carro)
        steering_ratio_      = declare_parameter<double>("steering_ratio", 14.0);   // volante / roda
        steering_sign_       = declare_parameter<double>("steering_sign", 1.0);     // -1 se o carro usar o sentido oposto
        max_wheel_angle_deg_ = declare_parameter<double>("max_steering_wheel_deg", 480.0);
        steer_kp_            = declare_parameter<double>("steering_kp", 0.02);      // Nm por grau de erro
        steer_kd_            = declare_parameter<double>("steering_kd", 0.002);     // Nm por grau/s
        base_kp_             = declare_parameter<double>("speed_kp", 0.5);
        brake_kp_max_        = declare_parameter<double>("brake_kp_max", 5.0);
        max_accel_           = declare_parameter<double>("max_accel", 1.0);
        max_decel_           = declare_parameter<double>("max_decel", 2.0);

        sub_safe_command_ = create_subscription<TeleopCommand>(
            "/teleop/safe_command", 10,
            std::bind(&ControlNode::safe_command_callback, this, std::placeholders::_1));

        // estado do carro
        sub_wheel_[0] = create_subscription<Float32>("/vehicle/wheel_speed_fl", 10,
            [this](const Float32::SharedPtr msg) { wheel_kmh_[0] = msg->data; });
        sub_wheel_[1] = create_subscription<Float32>("/vehicle/wheel_speed_fr", 10,
            [this](const Float32::SharedPtr msg) { wheel_kmh_[1] = msg->data; });
        sub_wheel_[2] = create_subscription<Float32>("/vehicle/wheel_speed_rl", 10,
            [this](const Float32::SharedPtr msg) { wheel_kmh_[2] = msg->data; });
        sub_wheel_[3] = create_subscription<Float32>("/vehicle/wheel_speed_rr", 10,
            [this](const Float32::SharedPtr msg) { wheel_kmh_[3] = msg->data; });

        sub_steering_ = create_subscription<Float32>(
            "/vehicle/steering_angle", 10,
            std::bind(&ControlNode::steering_callback, this, std::placeholders::_1));

        // modo decidido pelo vehicle_interface (como o /system/operation_mode/state do Autoware)
        sub_mode_ = create_subscription<UInt8>(
            "/teleop/vehicle_mode", rclcpp::QoS(1).reliable().transient_local(),
            [this](const UInt8::SharedPtr msg) { current_mode_ = msg->data; });

        pub_vehicle_cmd_ = create_publisher<VehicleCommand>("/teleop/vehicle_command", 10);

        rclcpp::QoS metrics_qos(10);
        metrics_qos.best_effort();
        metrics_qos.durability_volatile();
        pub_metrics_end_to_end_  = create_publisher<Metrics>("/metrics/e2e_command_latency", 10);
        pub_metrics_safety_gate_ = create_publisher<Metrics>("/metrics/safety_gate", metrics_qos);
        pub_metrics_control_     = create_publisher<Metrics>("/metrics/control", metrics_qos);

        RCLCPP_INFO(get_logger(), "Control node started (direct vehicle interface).");
    }

private:
    // ---- parâmetros
    double steering_ratio_, steering_sign_, max_wheel_angle_deg_;
    double steer_kp_, steer_kd_;
    double base_kp_, brake_kp_max_, max_accel_, max_decel_;

    // ---- estado do carro
    float wheel_kmh_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    std::optional<float> steering_wheel_deg_;
    double steering_rate_dps_ = 0.0;
    std::optional<rclcpp::Time> prev_steering_time_;

    uint8_t current_mode_ = CmdEnums::OPERATION_MODE_LOCAL;

    rclcpp::Subscription<TeleopCommand>::SharedPtr sub_safe_command_;
    rclcpp::Subscription<Float32>::SharedPtr       sub_wheel_[4];
    rclcpp::Subscription<Float32>::SharedPtr       sub_steering_;
    rclcpp::Subscription<UInt8>::SharedPtr         sub_mode_;

    rclcpp::Publisher<VehicleCommand>::SharedPtr pub_vehicle_cmd_;
    rclcpp::Publisher<Metrics>::SharedPtr pub_metrics_end_to_end_, pub_metrics_safety_gate_, pub_metrics_control_;

    double speed_mps() const
    {
        double sum = 0.0;
        for (float w : wheel_kmh_) sum += std::abs(w);
        return (sum / 4.0) / 3.6;
    }

    void publish_metric(const rclcpp::Publisher<Metrics>::SharedPtr &pub, uint32_t id,
                        const rclcpp::Time &rx_time, const rclcpp::Time &tx_time)
    {
        auto msg = std::make_unique<Metrics>();
        msg->id = id;
        msg->tx = tx_time;
        msg->rx = rx_time;
        msg->latency_ms = (rx_time - tx_time).seconds() * 1000.0;
        pub->publish(std::move(msg));
    }

    // ângulo do volante e a sua velocidade, calculada com o tempo das mensagens do carro
    void steering_callback(const Float32::SharedPtr msg)
    {
        const rclcpp::Time t = now();
        if (steering_wheel_deg_ && prev_steering_time_) {
            const double dt = (t - *prev_steering_time_).seconds();
            if (dt > 0.0) {
                const double raw_rate = (msg->data - *steering_wheel_deg_) / dt;
                steering_rate_dps_ = 0.7 * steering_rate_dps_ + 0.3 * raw_rate;  // filtro passa-baixo
            }
        }
        steering_wheel_deg_ = msg->data;
        prev_steering_time_ = t;
    }

    double compute_acceleration(const TeleopCommand &cmd) const
    {
        const double target = cmd.target_velocity;

        const double error = target - speed_mps();
        
        if (target <= 0.01 && cmd.brake_factor > 0.01f) {
            const double kp = base_kp_ + (brake_kp_max_ - base_kp_) * cmd.brake_factor;
            return std::clamp(kp * error, -max_decel_, 0.0);
        }
        return std::clamp(base_kp_ * error, -max_accel_, max_accel_);
    }

    double compute_torque(const TeleopCommand &cmd) const
    {
        if (!steering_wheel_deg_) return 0.0;

        double target_deg = steering_sign_ * cmd.target_steering_angle * 180.0 / M_PI * steering_ratio_;
        target_deg = std::clamp(target_deg, -max_wheel_angle_deg_, max_wheel_angle_deg_);

        return steer_kp_ * (target_deg - *steering_wheel_deg_) - steer_kd_ * steering_rate_dps_;
    }

    void publish_vehicle_command(double accel, double torque, bool engage)
    {
        auto out = std::make_unique<VehicleCommand>();
        out->header.stamp    = now();
        out->header.frame_id = "control";
        out->acceleration    = static_cast<float>(accel);
        out->steering_torque = static_cast<float>(torque);
        out->engage          = engage;
        pub_vehicle_cmd_->publish(std::move(out));
    }

    void safe_command_callback(const TeleopCommand::SharedPtr msg)
    {
        auto start_time = now();

        // envia sempre, para o pedido de engage chegar ao vehicle_interface,
        // mas só calcula aceleração e binário em REMOTE (como o control da versão com Autoware)
        if (current_mode_ == CmdEnums::OPERATION_MODE_REMOTE) {
            publish_vehicle_command(compute_acceleration(*msg), compute_torque(*msg), msg->engage_command);
        } else {
            publish_vehicle_command(0.0, 0.0, msg->engage_command);
        }

        auto end_time = now();

        // id 0 = paragem gerada pelo safety gate, não entra nas métricas
        if (msg->id != 0) {
            publish_metric(pub_metrics_safety_gate_, msg->id, start_time, rclcpp::Time(msg->header.stamp));
            publish_metric(pub_metrics_control_, msg->id, end_time, start_time);
            publish_metric(pub_metrics_end_to_end_, msg->id, end_time, rclcpp::Time(msg->origin_stamp));
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
