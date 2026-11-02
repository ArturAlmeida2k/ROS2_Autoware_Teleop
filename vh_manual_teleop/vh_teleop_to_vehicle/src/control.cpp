#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "teleop_msgs/msg/teleop_command.hpp"
#include "teleop_msgs/msg/vehicle_command.hpp"
#include "teleop_msgs/msg/node_metrics.hpp"
#include "teleop_msgs/msg/command_enums.hpp"

using Float32        = std_msgs::msg::Float32;
using Int8           = std_msgs::msg::Int8;
using UInt8          = std_msgs::msg::UInt8;
using TeleopCommand  = teleop_msgs::msg::TeleopCommand;
using VehicleCommand = teleop_msgs::msg::VehicleCommand;
using Metrics        = teleop_msgs::msg::NodeMetrics;
using CmdEnums       = teleop_msgs::msg::CommandEnums;

// Equivalente ao nó control da versão com Autoware: converte cada /teleop/safe_command
// num /teleop/vehicle_command (aceleração e binário no volante) e gere os modos.
// Reage a cada comando, sem timer. O ritmo fixo e as proteções ficam no vehicle_interface.
//
// Modos:
//   LOCAL  -> o condutor de segurança tem o carro, não se envia nada ao carro.
//             É o estado inicial e o estado depois de o condutor carregar no travão.
//   REMOTE -> o operador conduz.
//   STOP   -> o operador desligou o engage, o carro é travado e mantido parado
//             até novo engage ou travão.
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
        stop_decel_          = declare_parameter<double>("stop_decel", 2.0);

        sub_safe_command_ = create_subscription<TeleopCommand>(
            "/teleop/safe_command", 10,
            std::bind(&ControlNode::safe_command_callback, this, std::placeholders::_1));

        // estado do carro
        auto wheel_cb = [this](int i) {
            return [this, i](const Float32::SharedPtr msg) { wheel_kmh_[i] = msg->data; };
        };
        sub_wheel_[0] = create_subscription<Float32>("/vehicle/wheel_speed_fl", 10, wheel_cb(0));
        sub_wheel_[1] = create_subscription<Float32>("/vehicle/wheel_speed_fr", 10, wheel_cb(1));
        sub_wheel_[2] = create_subscription<Float32>("/vehicle/wheel_speed_rl", 10, wheel_cb(2));
        sub_wheel_[3] = create_subscription<Float32>("/vehicle/wheel_speed_rr", 10, wheel_cb(3));

        sub_steering_ = create_subscription<Float32>(
            "/vehicle/steering_angle", 10,
            std::bind(&ControlNode::steering_callback, this, std::placeholders::_1));

        sub_gear_ = create_subscription<Int8>(
            "/vehicle/gear", 10,
            [this](const Int8::SharedPtr msg) { vehicle_gear_ = msg->data; });

        sub_brake_ = create_subscription<Float32>(
            "/vehicle/brake_pressed", 10,
            std::bind(&ControlNode::brake_pressed_callback, this, std::placeholders::_1));

        pub_vehicle_cmd_ = create_publisher<VehicleCommand>("/teleop/vehicle_command", 10);

        // modo para o vehicle_telemetry (substitui o /system/operation_mode/state do Autoware)
        pub_mode_ = create_publisher<UInt8>("/teleop/vehicle_mode", rclcpp::QoS(1).reliable().transient_local());

        rclcpp::QoS metrics_qos(10);
        metrics_qos.best_effort();
        metrics_qos.durability_volatile();
        pub_metrics_end_to_end_  = create_publisher<Metrics>("/metrics/e2e_command_latency", 10);
        pub_metrics_safety_gate_ = create_publisher<Metrics>("/metrics/safety_gate", metrics_qos);
        pub_metrics_control_     = create_publisher<Metrics>("/metrics/control", metrics_qos);

        publish_mode();
        RCLCPP_INFO(get_logger(), "Control node started (direct vehicle interface, mode LOCAL).");
    }

private:
    // ---- parâmetros
    double steering_ratio_, steering_sign_, max_wheel_angle_deg_;
    double steer_kp_, steer_kd_;
    double base_kp_, brake_kp_max_, max_accel_, max_decel_, stop_decel_;

    // ---- estado do carro
    float wheel_kmh_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    std::optional<float> steering_wheel_deg_;
    double steering_rate_dps_ = 0.0;
    std::optional<rclcpp::Time> prev_steering_time_;
    int8_t vehicle_gear_ = -1;          // 0=P, 1=R, 2=N, 3=D
    bool brake_pressed_ = false;

    bool last_engage_cmd_ = false;
    uint8_t mode_ = CmdEnums::OPERATION_MODE_LOCAL;

    rclcpp::Subscription<TeleopCommand>::SharedPtr sub_safe_command_;
    rclcpp::Subscription<Float32>::SharedPtr       sub_wheel_[4];
    rclcpp::Subscription<Float32>::SharedPtr       sub_steering_;
    rclcpp::Subscription<Int8>::SharedPtr          sub_gear_;
    rclcpp::Subscription<Float32>::SharedPtr       sub_brake_;

    rclcpp::Publisher<VehicleCommand>::SharedPtr pub_vehicle_cmd_;
    rclcpp::Publisher<UInt8>::SharedPtr          pub_mode_;
    rclcpp::Publisher<Metrics>::SharedPtr pub_metrics_end_to_end_, pub_metrics_safety_gate_, pub_metrics_control_;

    double speed_mps() const
    {
        double sum = 0.0;
        for (float w : wheel_kmh_) sum += std::abs(w);
        return (sum / 4.0) / 3.6;
    }

    void set_mode(uint8_t mode, const char *reason)
    {
        if (mode == mode_) return;
        mode_ = mode;
        publish_mode();
        RCLCPP_INFO(get_logger(), "Mode -> %s (%s)",
            mode == CmdEnums::OPERATION_MODE_REMOTE ? "REMOTE" :
            mode == CmdEnums::OPERATION_MODE_LOCAL  ? "LOCAL"  : "STOP", reason);
    }

    void publish_mode()
    {
        UInt8 msg;
        msg.data = mode_;
        pub_mode_->publish(msg);
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
    // (não depende do ritmo a que chegam os comandos)
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

    // o condutor de segurança carregou no travão: devolve-lhe o carro
    void brake_pressed_callback(const Float32::SharedPtr msg)
    {
        brake_pressed_ = msg->data > 0.5f;
        if (brake_pressed_ && mode_ != CmdEnums::OPERATION_MODE_LOCAL) {
            set_mode(CmdEnums::OPERATION_MODE_LOCAL, "driver pressed the brake");
            publish_vehicle_command(0.0, 0.0);
        }
    }

    // aceleração com o controlador P da versão com Autoware
    double compute_acceleration(const TeleopCommand &cmd) const
    {
        double target = cmd.target_velocity;

        // não há tópico para mudar a mudança: só acelera se a mudança do carro
        // coincidir com a pedida, senão apenas trava
        const bool gear_ok =
            (cmd.gear == CmdEnums::GEAR_DRIVE   && vehicle_gear_ == 3) ||
            (cmd.gear == CmdEnums::GEAR_REVERSE && vehicle_gear_ == 1);
        if (!gear_ok && target > 0.0) target = 0.0;

        const double error = target - speed_mps();
        if (target <= 0.01 && cmd.brake_factor > 0.01f) {
            const double kp = base_kp_ + (brake_kp_max_ - base_kp_) * cmd.brake_factor;
            return std::clamp(kp * error, -max_decel_, 0.0);
        }
        return std::clamp(base_kp_ * error, -max_decel_, max_accel_);
    }

    // binário no volante: PD sobre o ângulo do volante
    double compute_torque(const TeleopCommand &cmd) const
    {
        if (!steering_wheel_deg_) return 0.0;

        double target_deg = steering_sign_ * cmd.target_steering_angle * 180.0 / M_PI * steering_ratio_;
        target_deg = std::clamp(target_deg, -max_wheel_angle_deg_, max_wheel_angle_deg_);

        return steer_kp_ * (target_deg - *steering_wheel_deg_) - steer_kd_ * steering_rate_dps_;
    }

    void publish_vehicle_command(double accel, double torque)
    {
        auto out = std::make_unique<VehicleCommand>();
        out->header.stamp    = now();
        out->header.frame_id = "control";
        out->acceleration    = static_cast<float>(accel);
        out->steering_torque = static_cast<float>(torque);
        out->mode            = mode_;
        pub_vehicle_cmd_->publish(std::move(out));
    }

    void safe_command_callback(const TeleopCommand::SharedPtr msg)
    {
        auto start_time = now();

        // o engage só atua quando muda
        const bool engage = msg->engage_command;
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

        switch (mode_) {
            case CmdEnums::OPERATION_MODE_REMOTE:
                publish_vehicle_command(compute_acceleration(*msg), compute_torque(*msg));
                break;
            case CmdEnums::OPERATION_MODE_STOP:
                // mantém o carro travado e o volante livre
                publish_vehicle_command(-stop_decel_, 0.0);
                break;
            default:
                // LOCAL: o vehicle_interface não envia nada ao carro
                publish_vehicle_command(0.0, 0.0);
                break;
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
