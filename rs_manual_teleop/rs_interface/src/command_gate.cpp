#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <chrono> 
#include <algorithm>

#include "teleop_msgs/msg/teleop_command.hpp"
#include "teleop_msgs/msg/telemetry_state.hpp" 
#include "teleop_msgs/msg/node_metrics.hpp" 
#include "teleop_msgs/msg/command_enums.hpp"

using TeleopCommand = teleop_msgs::msg::TeleopCommand;
using Telemetry = teleop_msgs::msg::TelemetryState;
using Metrics = teleop_msgs::msg::NodeMetrics;
using CmdEnums = teleop_msgs::msg::CommandEnums;

using namespace std::chrono_literals; 

class CommandGate : public rclcpp::Node
{
public:
    CommandGate() : Node("command_gate")
    {
        // --- 1. Publishers ---
        pub_final_command_ = this->create_publisher<TeleopCommand>("/teleop/command", 10);

        // For metrics
        rclcpp::QoS metrics_qos(10);       
        metrics_qos.best_effort();    
        metrics_qos.durability_volatile();
        pub_metrics_ = this->create_publisher<Metrics>("/metrics/controller", metrics_qos);

        // --- 2. Subscribers ---
        sub_raw_command_ = this->create_subscription<TeleopCommand>(
            "/teleop/raw_command", 10,
            std::bind(&CommandGate::raw_command_callback, this, std::placeholders::_1));
        
        sub_telemetry_ = this->create_subscription<Telemetry>(
            "/teleop/telemetry", 10,
            [this](const Telemetry::SharedPtr msg) {
                
                telemetry_watchdog_->reset();

                // Sincronização Inicial ou Reconexão
                if (!is_telemetry_valid_) {
                    target_engage_state_ = (msg->mode == CmdEnums::OPERATION_MODE_REMOTE);
                    target_gear_ = msg->gear;
                    target_turn_signal_ = msg->turn_signal; 

                    is_telemetry_valid_ = true;
                    
                    RCLCPP_INFO(this->get_logger(), "Telemetria sincronizada.");
                }

                current_mode_ = msg->mode;
                current_engage_status_ = msg->engaged;
                current_velocity_ = msg->velocity_kmh;
                current_turn_signal_ = msg->turn_signal;
                current_gear_ = msg->gear;
            });

        // --- 3. Watchdog Timer (Proteção contra perda de sinal) ---
        // Se passarem 3s sem o watchdog ser reiniciado no callback acima, esta função é chamada
        telemetry_watchdog_ = this->create_wall_timer(
            3s, std::bind(&CommandGate::telemetry_timeout_callback, this));

        RCLCPP_INFO(this->get_logger(), "Nó CommandGate iniciado. A aguardar /teleop/raw_command e /teleop/telemetry.");
    }

private:
    static constexpr float MAX_VLC_ = 10.0f; // m/s

    // --- Variáveis de Leitura da Telemetria ---
    int current_mode_ = 0;
    bool current_engage_status_ = false;
    float current_velocity_ = 0.0f;
    int current_turn_signal_ = 0;
    int current_gear_ = 0;

    // --- Variáveis de Retenção de Estado (A tua Lógica) ---
    bool is_telemetry_valid_ = false;
    bool target_engage_state_ = false;
    bool last_received_engage_button_ = false;
    bool last_received_uplink_button_ = false;

    int target_gear_ = CmdEnums::GEAR_PARK;
    int last_requested_gear_ = CmdEnums::GEAR_NONE;
    int target_turn_signal_ = CmdEnums::TURN_OFF; 
    int last_received_turn_button_ = 0;
    int current_uplink_mode_ = CmdEnums::UPLINK_VIDEO;

    // --- Interfaces ROS 2 ---
    rclcpp::Subscription<TeleopCommand>::SharedPtr sub_raw_command_;
    rclcpp::Subscription<Telemetry>::SharedPtr sub_telemetry_;
    rclcpp::Publisher<TeleopCommand>::SharedPtr pub_final_command_;
    rclcpp::Publisher<Metrics>::SharedPtr pub_metrics_;
    rclcpp::TimerBase::SharedPtr telemetry_watchdog_;

    // --- Callback do Watchdog (Perda de Telemetria) ---
    void telemetry_timeout_callback()
    {
        if (is_telemetry_valid_) {
            RCLCPP_WARN(this->get_logger(), "Sinal de telemetria perdido! Comandos suspensos.");
            
            is_telemetry_valid_ = false;

            current_mode_ = 0; 
            current_velocity_ = 0.0f;
            current_gear_ = 0;

            target_engage_state_ = false;
            last_received_engage_button_ = false; 
            target_gear_ = CmdEnums::GEAR_PARK;
            last_requested_gear_ = CmdEnums::GEAR_NONE;
            target_turn_signal_ = CmdEnums::TURN_OFF;
            last_received_turn_button_ = 0;
            last_received_uplink_button_ = false;
            current_uplink_mode_ = CmdEnums::UPLINK_VIDEO;
        }
    }

    void publish_metrics(uint32_t id, const rclcpp::Time &rx_time, const builtin_interfaces::msg::Time &tx_msg_time)
    {
        rclcpp::Time tx_time(tx_msg_time);

        auto msg = std::make_unique<Metrics>();
        msg->id = id;
        msg->tx = tx_time;
        msg->rx = rx_time;
        
        rclcpp::Duration latency = rx_time - tx_time;

        msg->latency_ms = latency.seconds() * 1000.0;

        pub_metrics_->publish(std::move(msg));

    }

    // --- Callback Principal de Comandos ---
    void raw_command_callback(const TeleopCommand::SharedPtr msg)
    {
        auto start_time = this->now();

        auto final_msg = std::make_unique<TeleopCommand>();

        final_msg->header.stamp = start_time; 
        final_msg->header.frame_id = "command_gate";  
        final_msg->origin_stamp = msg->origin_stamp;
        final_msg->id = msg->id;

        // Se não houver telemetria ativa
        if (!is_telemetry_valid_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Sem telemetria válida. Comandos suprimidos.");
            return; 
        }

        // -------------------------------------------------------------
        // 2. LÓGICA DO ENGAGE (Deteção de Flanco Positivo)
        // -------------------------------------------------------------
        bool current_engage_button = msg->engage_command;

        // Flanco positivo: o botão está a ser premido agora, mas não estava no ciclo anterior
        if (current_engage_button && !last_received_engage_button_) {
            if (current_velocity_ < 0.1f) {
                target_engage_state_ = !target_engage_state_;
                RCLCPP_INFO(this->get_logger(), "Toggle Engage recebido. Novo estado objetivo: %s", target_engage_state_ ? "TRUE" : "FALSE");
            } else {
                RCLCPP_WARN(this->get_logger(), "Tentativa de alterar Engage bloqueada: Veículo em movimento (Velocidade: %.2f)", current_velocity_);
            }
        }
        
        last_received_engage_button_ = current_engage_button;
        
        final_msg->engage_command = target_engage_state_;

        // -------------------------------------------------------------
        // 3. VALIDAÇÃO DE MODO E BLOCO DE LÓGICA
        // -------------------------------------------------------------
        if (current_mode_ == CmdEnums::OPERATION_MODE_REMOTE) {
            
            final_msg->target_velocity = msg->target_velocity * (MAX_VLC_ / 3.6f);
            final_msg->brake_factor = msg->brake_factor;
            final_msg->target_steering_angle = msg->target_steering_angle;

            int requested_gear = msg->gear;
            bool gear_button_edge = (requested_gear != CmdEnums::GEAR_NONE && requested_gear != last_requested_gear_);
            
            if (gear_button_edge) {
                if (current_velocity_ < 0.1f) {
                    if (current_gear_ == CmdEnums::GEAR_PARK) {
                        if (requested_gear == CmdEnums::GEAR_PARK) target_gear_ = CmdEnums::GEAR_DRIVE;
                    } else {
                        target_gear_ = requested_gear;
                    }
                    last_requested_gear_ = requested_gear;  
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Tentativa de mudar de mudança bloqueada: Veículo em movimento (Velocidade: %.2f)", current_velocity_);
                }
            } else if (requested_gear == CmdEnums::GEAR_NONE) {
                last_requested_gear_ = CmdEnums::GEAR_NONE;
            }
            final_msg->gear = target_gear_;

            int current_button = msg->turn_signal;

            bool pressed_right  = (current_button == CmdEnums::TURN_RIGHT && last_received_turn_button_ != CmdEnums::TURN_RIGHT);
            bool pressed_left   = (current_button == CmdEnums::TURN_LEFT && last_received_turn_button_ != CmdEnums::TURN_LEFT);
            bool pressed_hazard = (current_button == CmdEnums::TURN_HAZARD && last_received_turn_button_ != CmdEnums::TURN_HAZARD);

            if (pressed_right) {
                target_turn_signal_ = (current_turn_signal_ == CmdEnums::TURN_RIGHT) ? CmdEnums::TURN_OFF : CmdEnums::TURN_RIGHT;
            }
            else if (pressed_left) {
                target_turn_signal_ = (current_turn_signal_ == CmdEnums::TURN_LEFT) ? CmdEnums::TURN_OFF : CmdEnums::TURN_LEFT;
            }
            else if (pressed_hazard) {
                target_turn_signal_ = (current_turn_signal_ == CmdEnums::TURN_HAZARD) ? CmdEnums::TURN_OFF : CmdEnums::TURN_HAZARD;
            }

            last_received_turn_button_ = current_button;
            final_msg->turn_signal = target_turn_signal_;

            bool pressed_uplink = (msg->uplink_mode == 1);

            if (pressed_uplink && !last_received_uplink_button_){
                current_uplink_mode_ = (current_uplink_mode_ == CmdEnums::UPLINK_VIDEO) ? CmdEnums::UPLINK_POINTCLOUD : CmdEnums::UPLINK_VIDEO;
            }

            last_received_uplink_button_ = pressed_uplink;

            final_msg->uplink_mode = current_uplink_mode_;

        } else {
            final_msg->target_velocity = 0.0f;
            final_msg->brake_factor = 0.0f;
            final_msg->target_steering_angle = 0.0f;
            
            target_gear_ = CmdEnums::GEAR_PARK;
            final_msg->gear = target_gear_;
            
            target_turn_signal_ = CmdEnums::TURN_OFF;
            final_msg->turn_signal = target_turn_signal_;

            last_received_uplink_button_ = false;
            current_uplink_mode_ = CmdEnums::UPLINK_VIDEO; 
            final_msg->uplink_mode = current_uplink_mode_;
        }

        // -------------------------------------------------------------
        // 4. PUBLICAÇÃO
        // -------------------------------------------------------------
        pub_final_command_->publish(std::move(final_msg));

        // Metrics
        publish_metrics(msg->id, start_time, msg->header.stamp);

    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CommandGate>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}