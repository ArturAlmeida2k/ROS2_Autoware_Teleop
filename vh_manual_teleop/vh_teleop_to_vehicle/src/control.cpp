#include <algorithm>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "teleop_msgs/msg/teleop_command.hpp"
#include "teleop_msgs/msg/vehicle_command.hpp"
#include "teleop_msgs/msg/node_metrics.hpp"

using Float32        = std_msgs::msg::Float32;
using TeleopCommand  = teleop_msgs::msg::TeleopCommand;
using VehicleCommand = teleop_msgs::msg::VehicleCommand;
using Metrics        = teleop_msgs::msg::NodeMetrics;

// Equivalente ao nó control do Autoware: converte cada /teleop/safe_command num
// /teleop/vehicle_command (aceleração, ângulo das rodas, engage e mudança) para o
// vehicle_interface. Só reage quando chega um comando, como antes.
class ControlNode : public rclcpp::Node
{
public:
    ControlNode() : Node("control")
    {
        base_kp_      = declare_parameter<double>("speed_kp", 0.5);
        brake_kp_max_ = declare_parameter<double>("brake_kp_max", 5.0);
        max_accel_    = declare_parameter<double>("max_accel", 1.0);
        max_decel_    = declare_parameter<double>("max_decel", 5.0);

        sub_safe_command_ = create_subscription<TeleopCommand>(
            "/teleop/safe_command", 10,
            std::bind(&ControlNode::safe_command_callback, this, std::placeholders::_1));

        // velocidade medida pelo vehicle_interface (m/s, sem sinal)
        sub_speed_ = create_subscription<Float32>(
            "/teleop/vehicle_speed", 10,
            [this](const Float32::SharedPtr msg) { speed_ = msg->data; });

        pub_vehicle_cmd_ = create_publisher<VehicleCommand>("/teleop/vehicle_command", 10);

        rclcpp::QoS metrics_qos(10);
        metrics_qos.best_effort();
        metrics_qos.durability_volatile();
        pub_metrics_end_to_end_  = create_publisher<Metrics>("/metrics/e2e_command_latency", 10);
        pub_metrics_safety_gate_ = create_publisher<Metrics>("/metrics/safety_gate", metrics_qos);
        pub_metrics_control_     = create_publisher<Metrics>("/metrics/control", metrics_qos);

        RCLCPP_INFO(get_logger(), "Control node started (vehicle interface).");
    }

private:
    double base_kp_, brake_kp_max_, max_accel_, max_decel_;
    float speed_ = 0.0f;

    rclcpp::Subscription<TeleopCommand>::SharedPtr sub_safe_command_;
    rclcpp::Subscription<Float32>::SharedPtr       sub_speed_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr   pub_vehicle_cmd_;
    rclcpp::Publisher<Metrics>::SharedPtr pub_metrics_end_to_end_, pub_metrics_safety_gate_, pub_metrics_control_;

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

    void safe_command_callback(const TeleopCommand::SharedPtr msg)
    {
        auto start_time = now();

        // a travar, o ganho sobe com o brake_factor (P simples na velocidade)
        const double target = msg->target_velocity;
        const double error  = target - std::abs(speed_);
        double accel;
        if (target <= 0.01 && msg->brake_factor > 0.01f) {
            const double kp = base_kp_ + (brake_kp_max_ - base_kp_) * msg->brake_factor;
            accel = std::clamp(kp * error, -max_decel_, 0.0);
        } else {
            accel = std::clamp(base_kp_ * error, -max_accel_, max_accel_);
        }
        if (std::abs(accel) < 0.0001) accel = 0.0;

        auto cmd = std::make_unique<VehicleCommand>();
        cmd->header.stamp        = start_time;
        cmd->header.frame_id     = "control";
        cmd->acceleration        = static_cast<float>(accel);
        cmd->steering_tire_angle = msg->target_steering_angle;
        cmd->engage              = msg->engage_command;
        cmd->gear                = msg->gear;
        pub_vehicle_cmd_->publish(std::move(cmd));

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
