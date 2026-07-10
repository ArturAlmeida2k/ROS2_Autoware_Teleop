#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include <algorithm>
#include <memory>
#include <cmath>

#include "msg_manual_teleop/msg/teleop_command.hpp"
#include "msg_manual_teleop/msg/node_metrics.hpp"
#include <autoware_control_msgs/msg/control.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include "tier4_external_api_msgs/srv/engage.hpp"

using Control               = autoware_control_msgs::msg::Control;
using GateMode              = tier4_control_msgs::msg::GateMode;
using GearCommand           = autoware_vehicle_msgs::msg::GearCommand;
using Bool                  = std_msgs::msg::Bool;
using VelocityReport        = autoware_vehicle_msgs::msg::VelocityReport;
using TurnIndicatorsCommand = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using HazardLightsCommand   = autoware_vehicle_msgs::msg::HazardLightsCommand;
using OperationModeState    = autoware_adapi_v1_msgs::msg::OperationModeState;
using EngageSrv             = tier4_external_api_msgs::srv::Engage;
using TeleopCommand         = msg_manual_teleop::msg::TeleopCommand;
using Metrics               = msg_manual_teleop::msg::NodeMetrics;

class AutowareControllerNode : public rclcpp::Node
{
public:
    AutowareControllerNode() : Node("autoware_controller_node")
    {
        // 1. Subscrições
        // Agora ouve os comandos já validados e filtrados pelo Safety Gate
        sub_safe_command_ = this->create_subscription<TeleopCommand>(
            "/teleop/safe_command", 10,
            std::bind(&AutowareControllerNode::safe_command_callback, this, std::placeholders::_1));

        sub_vlc_current_ = this->create_subscription<VelocityReport>(
            "/vehicle/status/velocity_status", 10,
            std::bind(&AutowareControllerNode::vlc_report_callback, this, std::placeholders::_1));

        sub_operation_mode_ = this->create_subscription<OperationModeState>(
            "/system/operation_mode/state", 10,
            std::bind(&AutowareControllerNode::operation_mode_callback, this, std::placeholders::_1));

        // 2. Publicadores Autoware
        pub_control_cmd_     = this->create_publisher<Control>("/external/selected/control_cmd", rclcpp::QoS(1));
        pub_gear_cmd_        = this->create_publisher<GearCommand>("/external/selected/gear_cmd", 1);
        pub_turn_indicators_ = this->create_publisher<TurnIndicatorsCommand>("/external/selected/turn_indicators_cmd", 1);
        pub_hazard_lights_   = this->create_publisher<HazardLightsCommand>("/external/selected/hazard_lights_cmd", 1);
        pub_gate_mode_       = this->create_publisher<GateMode>("/control/gate_mode_cmd", rclcpp::QoS(1));
        
        // 3. Clientes/Serviços Autoware
        client_engage_       = this->create_client<EngageSrv>("/api/autoware/set/engage");

        // 4. Publicadores de Métricas
        rclcpp::QoS metrics_qos(10);       
        metrics_qos.best_effort();    
        metrics_qos.durability_volatile();

        pub_metrics_end_to_end_  = this->create_publisher<Metrics>("/metrics/e2e_command_latency", 10);
        pub_metrics_safety_gate_ = this->create_publisher<Metrics>("/metrics/safety_gate", metrics_qos);
        pub_metrics_control_     = this->create_publisher<Metrics>("/metrics/control", metrics_qos); 

        RCLCPP_INFO(this->get_logger(), "Autoware Controller Node started. (Translation & Metrics mode)");
    }

private:
    float   vlc_target_            = 0.0f;
    float   vlc_current_           = 0.0f;
    float   steering_angle_target_ = 0.0f;
    float   brake_factor_          = 0.0f;
    int     gear_change_           = GearCommand::REVERSE;
    bool    last_engage_cmd_       = false;
    uint8_t current_mode_          = OperationModeState::UNKNOWN;

    const double BASE_KP_GAIN = 0.5;
    const double BRAKE_KP_MAX = 5.0;
    const double MAX_ACCEL    = 1.0;
    const double MAX_DECEL    = 5.0;

    rclcpp::Publisher<GateMode>::SharedPtr              pub_gate_mode_;
    rclcpp::Publisher<Control>::SharedPtr               pub_control_cmd_;
    rclcpp::Publisher<GearCommand>::SharedPtr           pub_gear_cmd_;
    rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr pub_turn_indicators_;
    rclcpp::Publisher<HazardLightsCommand>::SharedPtr   pub_hazard_lights_;
    
    rclcpp::Publisher<Metrics>::SharedPtr               pub_metrics_end_to_end_;
    rclcpp::Publisher<Metrics>::SharedPtr               pub_metrics_safety_gate_;
    rclcpp::Publisher<Metrics>::SharedPtr               pub_metrics_control_;

    rclcpp::Subscription<TeleopCommand>::SharedPtr      sub_safe_command_;
    rclcpp::Subscription<VelocityReport>::SharedPtr     sub_vlc_current_;
    rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_;

    rclcpp::Client<EngageSrv>::SharedPtr                client_engage_;

    // Função genérica para aceitar qualquer publicador e os tempos em rclcpp::Time
    void publish_metric(const rclcpp::Publisher<Metrics>::SharedPtr& pub, uint32_t id, const rclcpp::Time &rx_time, const rclcpp::Time &tx_time)
    {
        auto msg = std::make_unique<Metrics>();
        msg->id = id;
        msg->tx = tx_time; 
        msg->rx = rx_time;
        
        rclcpp::Duration latency = rx_time - tx_time;
        
        msg->latency_ms = latency.seconds() * 1000.0;
        
        pub->publish(std::move(msg));
    }

    void operation_mode_callback(const OperationModeState::SharedPtr msg)
    {
        current_mode_ = msg->mode;
    }

    void vlc_report_callback(const VelocityReport::SharedPtr msg)
    {
        vlc_current_ = msg->longitudinal_velocity;
    }

    void safe_command_callback(const TeleopCommand::SharedPtr msg)
    {
        // 1. Regista o momento em que a mensagem entra neste nó
        auto start_time = this->now();

        // 2. EXTRAÇÃO DE VARIÁVEIS
        vlc_target_            = msg->target_velocity;
        steering_angle_target_ = msg->target_steering_angle;
        brake_factor_          = msg->brake_factor;

        // 3. ENGAGE LOGIC
        bool engage_cmd = msg->engage_command;
        if (engage_cmd != last_engage_cmd_) {
            GateMode gate_msg;
            gate_msg.data = GateMode::EXTERNAL;
            pub_gate_mode_->publish(gate_msg);

            if (client_engage_->service_is_ready()) {
                auto req = std::make_shared<EngageSrv::Request>();
                req->engage = engage_cmd;
                client_engage_->async_send_request(req, []([[maybe_unused]] rclcpp::Client<EngageSrv>::SharedFuture) {});
            }
            last_engage_cmd_ = engage_cmd;
        }

        // 4. AUTOWARE TRANSLATION (Se os modos o permitirem)
        if (current_mode_ == OperationModeState::LOCAL || current_mode_ == OperationModeState::REMOTE) {
            
            // --- Sinais luminosos ---
            TurnIndicatorsCommand turn_cmd;
            turn_cmd.stamp = start_time;
            HazardLightsCommand hazard_cmd;
            hazard_cmd.stamp = start_time;

            if (msg->turn_signal == 3 || msg->turn_signal == 4) {
                turn_cmd.command   = TurnIndicatorsCommand::DISABLE;
                hazard_cmd.command = HazardLightsCommand::ENABLE;
            } else {
                turn_cmd.command   = msg->turn_signal;
                hazard_cmd.command = HazardLightsCommand::DISABLE;
            }
        
            // --- Mudanças (Gear) ---
            gear_change_ = msg->gear;
            GearCommand gear_cmd;
            gear_cmd.stamp = start_time;
            switch (gear_change_) {
                case 0: gear_cmd.command = GearCommand::PARK;    break;
                case 1: gear_cmd.command = GearCommand::DRIVE;   break;
                case 2: gear_cmd.command = GearCommand::REVERSE; break;
            }
        
            // --- Controlo de Movimento (Control) ---
            auto control_cmd = std::make_unique<Control>();
            control_cmd->stamp = start_time;
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

            pub_turn_indicators_->publish(turn_cmd);
            pub_hazard_lights_->publish(hazard_cmd);
            pub_gear_cmd_->publish(gear_cmd);
            pub_control_cmd_->publish(std::move(control_cmd));
        }

        // 5. Regista o tempo de conclusão do nó
        auto end_time = this->now();

        // 6. PUBLICAÇÃO DAS MÉTRICAS
        
        // A) Latência do Salto (Hop): desde a saída do Safety Gate até à entrada neste nó
        publish_metric(pub_metrics_safety_gate_, msg->id, start_time, rclcpp::Time(msg->header.stamp));
        
        // B) Latência de Processamento Interno: o tempo despendido dentro desta função callback
        publish_metric(pub_metrics_control_, msg->id, end_time, start_time);
        
        // C) Latência End-to-End: desde o volante (origin_stamp) até o processamento estar concluído
        publish_metric(pub_metrics_end_to_end_, msg->id, end_time, rclcpp::Time(msg->origin_stamp));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutowareControllerNode>());
    rclcpp::shutdown();
    return 0;
}