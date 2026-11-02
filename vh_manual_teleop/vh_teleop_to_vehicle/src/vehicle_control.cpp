#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "teleop_msgs/msg/teleop_command.hpp"
#include "teleop_msgs/msg/node_metrics.hpp"
#include "teleop_msgs/msg/command_enums.hpp"

using Float32       = std_msgs::msg::Float32;
using Int8          = std_msgs::msg::Int8;
using UInt8         = std_msgs::msg::UInt8;
using TeleopCommand = teleop_msgs::msg::TeleopCommand;
using Metrics       = teleop_msgs::msg::NodeMetrics;
using CmdEnums      = teleop_msgs::msg::CommandEnums;

using namespace std::chrono_literals;

// Substitui o nó control quando não há Autoware: converte o /teleop/safe_command
// diretamente nos tópicos do carro (/control/desired_acceleration,
// /control/desired_steering_torque e /control/indicated_speed).
//
// Estados:
//   LOCAL  -> o condutor de segurança tem o carro, o nó não publica comandos.
//             É o estado inicial e o estado depois de o condutor carregar no travão.
//   REMOTE -> o operador conduz.
//   STOP   -> o operador desligou o engage, o carro é travado e mantido parado
//             (equivalente ao modo STOP do Autoware) até novo engage ou travão.
class VehicleControlNode : public rclcpp::Node
{
public:
    VehicleControlNode() : Node("vehicle_control_node")
    {
        // parâmetros (todos têm de ser confirmados no carro)
        steering_ratio_      = declare_parameter<double>("steering_ratio", 14.0);   // volante / roda
        steering_sign_       = declare_parameter<double>("steering_sign", 1.0);     // inverte se o carro usar o sentido oposto
        max_wheel_angle_deg_ = declare_parameter<double>("max_steering_wheel_deg", 480.0);
        steer_kp_            = declare_parameter<double>("steering_kp", 0.02);      // Nm por grau de erro
        steer_kd_            = declare_parameter<double>("steering_kd", 0.002);     // Nm por grau/s
        max_torque_          = declare_parameter<double>("max_steering_torque", 2.0);
        base_kp_             = declare_parameter<double>("speed_kp", 0.5);
        brake_kp_max_        = declare_parameter<double>("brake_kp_max", 5.0);
        max_accel_           = declare_parameter<double>("max_accel", 1.0);
        max_decel_           = declare_parameter<double>("max_decel", 2.0);
        stop_decel_          = declare_parameter<double>("stop_decel", 2.0);
        cmd_timeout_s_       = declare_parameter<double>("command_timeout", 0.5);
        const double rate_hz = declare_parameter<double>("control_rate", 100.0);

        sub_safe_command_ = create_subscription<TeleopCommand>(
            "/teleop/safe_command", 10,
            std::bind(&VehicleControlNode::safe_command_callback, this, std::placeholders::_1));

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
            [this](const Float32::SharedPtr msg) { steering_wheel_deg_ = msg->data; });

        sub_gear_ = create_subscription<Int8>(
            "/vehicle/gear", 10,
            [this](const Int8::SharedPtr msg) { vehicle_gear_ = msg->data; });

        sub_brake_ = create_subscription<Float32>(
            "/vehicle/brake_pressed", 10,
            std::bind(&VehicleControlNode::brake_pressed_callback, this, std::placeholders::_1));

        // comandos para o carro
        pub_accel_  = create_publisher<Float32>("/control/desired_acceleration", 1);
        pub_torque_ = create_publisher<Float32>("/control/desired_steering_torque", 1);
        pub_speed_  = create_publisher<Float32>("/control/indicated_speed", 1);

        // modo para o vehicle_telemetry (substitui o /system/operation_mode/state do Autoware)
        pub_mode_ = create_publisher<UInt8>("/teleop/vehicle_mode", rclcpp::QoS(1).reliable().transient_local());

        // métricas, iguais às do nó control
        rclcpp::QoS metrics_qos(10);
        metrics_qos.best_effort();
        metrics_qos.durability_volatile();
        pub_metrics_end_to_end_  = create_publisher<Metrics>("/metrics/e2e_command_latency", 10);
        pub_metrics_safety_gate_ = create_publisher<Metrics>("/metrics/safety_gate", metrics_qos);
        pub_metrics_control_     = create_publisher<Metrics>("/metrics/control", metrics_qos);

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate_hz),
            std::bind(&VehicleControlNode::control_loop, this));

        publish_mode();
        RCLCPP_INFO(get_logger(), "Vehicle control node started (direct vehicle interface, mode LOCAL).");
    }

private:
    // ---- parâmetros
    double steering_ratio_, steering_sign_, max_wheel_angle_deg_;
    double steer_kp_, steer_kd_, max_torque_;
    double base_kp_, brake_kp_max_, max_accel_, max_decel_, stop_decel_;
    double cmd_timeout_s_;

    // ---- estado do carro
    float wheel_kmh_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    std::optional<float> steering_wheel_deg_;
    int8_t vehicle_gear_ = -1;          // 0=P, 1=R, 2=N, 3=D
    bool brake_pressed_ = false;

    // ---- estado pedido pela estação
    float target_velocity_ = 0.0f;      // m/s
    float brake_factor_ = 0.0f;
    float target_tyre_angle_ = 0.0f;    // rad
    uint8_t requested_gear_ = CmdEnums::GEAR_NONE;
    bool last_engage_cmd_ = false;
    std::optional<rclcpp::Time> last_cmd_time_;

    uint8_t mode_ = CmdEnums::OPERATION_MODE_LOCAL;

    // ---- derivada do ângulo do volante
    std::optional<float> prev_steering_deg_;
    double steering_rate_dps_ = 0.0;
    std::optional<rclcpp::Time> prev_loop_time_;

    rclcpp::Subscription<TeleopCommand>::SharedPtr sub_safe_command_;
    rclcpp::Subscription<Float32>::SharedPtr       sub_wheel_[4];
    rclcpp::Subscription<Float32>::SharedPtr       sub_steering_;
    rclcpp::Subscription<Int8>::SharedPtr          sub_gear_;
    rclcpp::Subscription<Float32>::SharedPtr       sub_brake_;

    rclcpp::Publisher<Float32>::SharedPtr pub_accel_, pub_torque_, pub_speed_;
    rclcpp::Publisher<UInt8>::SharedPtr   pub_mode_;
    rclcpp::Publisher<Metrics>::SharedPtr pub_metrics_end_to_end_, pub_metrics_safety_gate_, pub_metrics_control_;
    rclcpp::TimerBase::SharedPtr timer_;

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

    // o condutor de segurança carregou no travão: devolve-lhe o carro
    void brake_pressed_callback(const Float32::SharedPtr msg)
    {
        brake_pressed_ = msg->data > 0.5f;
        if (brake_pressed_ && mode_ != CmdEnums::OPERATION_MODE_LOCAL) {
            set_mode(CmdEnums::OPERATION_MODE_LOCAL, "driver pressed the brake");
        }
    }

    void safe_command_callback(const TeleopCommand::SharedPtr msg)
    {
        auto start_time = now();

        target_velocity_   = msg->target_velocity;
        brake_factor_      = msg->brake_factor;
        target_tyre_angle_ = msg->target_steering_angle;
        requested_gear_    = msg->gear;
        last_cmd_time_     = start_time;

        // o engage só atua quando muda, como no nó control
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

        // os comandos são aplicados no control_loop, aqui só ficam as métricas
        auto end_time = now();
        if (msg->id != 0) {
            publish_metric(pub_metrics_safety_gate_, msg->id, start_time, rclcpp::Time(msg->header.stamp));
            publish_metric(pub_metrics_control_, msg->id, end_time, start_time);
            publish_metric(pub_metrics_end_to_end_, msg->id, end_time, rclcpp::Time(msg->origin_stamp));
        }
    }

    // aceleração pedida, com o mesmo controlador P do nó control
    double compute_acceleration(double speed)
    {
        // comandos parados há demasiado tempo: trava
        const bool stale = !last_cmd_time_ ||
                           (now() - *last_cmd_time_).seconds() > cmd_timeout_s_;
        if (stale) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No command received, braking.");
            return -stop_decel_;
        }

        double target = target_velocity_;

        // não há tópico para mudar a mudança: só acelera se a mudança do carro
        // coincidir com a pedida, senão apenas trava
        const bool gear_ok =
            (requested_gear_ == CmdEnums::GEAR_DRIVE   && vehicle_gear_ == 3) ||
            (requested_gear_ == CmdEnums::GEAR_REVERSE && vehicle_gear_ == 1);
        if (!gear_ok && target > 0.0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Requested gear does not match the vehicle gear (%d), throttle ignored.", vehicle_gear_);
            target = 0.0;
        }

        const double error = target - speed;
        double accel;
        if (target <= 0.01 && brake_factor_ > 0.01f) {
            const double kp = base_kp_ + (brake_kp_max_ - base_kp_) * brake_factor_;
            accel = std::clamp(kp * error, -max_decel_, 0.0);
        } else {
            accel = std::clamp(base_kp_ * error, -max_decel_, max_accel_);
        }
        return accel;
    }

    // binário no volante: PD sobre o ângulo do volante
    double compute_torque(double dt)
    {
        if (!steering_wheel_deg_) return 0.0;

        const double measured = *steering_wheel_deg_;
        if (prev_steering_deg_ && dt > 0.0) {
            const double raw_rate = (measured - *prev_steering_deg_) / dt;
            steering_rate_dps_ = 0.7 * steering_rate_dps_ + 0.3 * raw_rate;  // filtro passa-baixo
        }
        prev_steering_deg_ = static_cast<float>(measured);

        double target_deg = steering_sign_ * target_tyre_angle_ * 180.0 / M_PI * steering_ratio_;
        target_deg = std::clamp(target_deg, -max_wheel_angle_deg_, max_wheel_angle_deg_);

        const double torque = steer_kp_ * (target_deg - measured) - steer_kd_ * steering_rate_dps_;
        return std::clamp(torque, -max_torque_, max_torque_);
    }

    void control_loop()
    {
        const rclcpp::Time t = now();
        const double dt = prev_loop_time_ ? (t - *prev_loop_time_).seconds() : 0.0;
        prev_loop_time_ = t;

        const double speed = speed_mps();

        Float32 speed_msg;
        speed_msg.data = static_cast<float>(speed * 3.6);
        pub_speed_->publish(speed_msg);

        Float32 accel_msg;
        Float32 torque_msg;

        switch (mode_) {
            case CmdEnums::OPERATION_MODE_REMOTE:
                accel_msg.data  = static_cast<float>(compute_acceleration(speed));
                torque_msg.data = static_cast<float>(compute_torque(dt));
                break;

            case CmdEnums::OPERATION_MODE_STOP:
                // mantém o carro travado e o volante livre
                accel_msg.data  = static_cast<float>(-stop_decel_);
                torque_msg.data = 0.0f;
                prev_steering_deg_.reset();
                steering_rate_dps_ = 0.0;
                break;

            case CmdEnums::OPERATION_MODE_LOCAL:
            default:
                // o condutor manda, não publica comandos
                prev_steering_deg_.reset();
                steering_rate_dps_ = 0.0;
                return;
        }

        pub_accel_->publish(accel_msg);
        pub_torque_->publish(torque_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleControlNode>());
    rclcpp::shutdown();
    return 0;
}
