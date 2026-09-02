#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <chrono> 

#include "msg_manual_teleop/msg/teleop_command.hpp"
#include "msg_manual_teleop/msg/telemetry_state.hpp" 
#include "msg_manual_teleop/msg/node_metrics.hpp" 

using TeleopCommand = msg_manual_teleop::msg::TeleopCommand;
using Telemetry = msg_manual_teleop::msg::TelemetryState;
using Metrics = msg_manual_teleop::msg::NodeMetrics;
using namespace std::chrono_literals; 

class ComandGate : public rclcpp::Node
{
public:
    ComandGate() : Node("comand_gate")
    {
        // --- 1. Publishers ---
        pub_final_command_ = this->create_publisher<TeleopCommand>("/teleop/command", 10);

        // For metrics
        rclcpp::QoS metrics_qos(10);       
        metrics_qos.best_effort();    
        metrics_qos.durability_volatile();
        pub_metrics_ = this->create_publisher<Metrics>("/metrics/controller", metrics_qos);

        // --- 2. Subscribers ---
        sub_filtered_command_ = this->create_subscription<TeleopCommand>(
            "/teleop/filtered_command", 10,
            std::bind(&ComandGate::filtered_command_callback, this, std::placeholders::_1));
        
        sub_telemetry_ = this->create_subscription<Telemetry>(
            "/telemetry/state", 10,
            [this](const Telemetry::SharedPtr msg) {
                
                telemetry_watchdog_->reset();

                // Sincronização Inicial ou Reconexão
                if (!is_telemetry_valid_) {
                    target_engage_state_ = (msg->mode == 4);
                    target_gear_ = msg->gear;
                    target_turn_signal_ = msg->turn_signal; 

                    is_telemetry_valid_ = true;
                    
                    RCLCPP_INFO(this->get_logger(), "Telemetria sincronizada.");
                }

                current_mode_ = msg->mode;
                current_engage_status_ = msg->engaged;
                current_velocity_ = msg->velocity_kmh;
                current_turn_signal_ = msg->turn_signal;
                current_hazard_signal_ = msg->hazard;
                current_gear_ = msg->gear;
            });

        // --- 3. Watchdog Timer (Proteção contra perda de sinal) ---
        // Se passarem 3s sem o watchdog ser reiniciado no callback acima, esta função é chamada
        telemetry_watchdog_ = this->create_wall_timer(
            3s, std::bind(&ComandGate::telemetry_timeout_callback, this));

        RCLCPP_INFO(this->get_logger(), "Nó ComandGate iniciado. A aguardar /teleop/filtered_command e /teleop/telemetry.");
    }

private:
    // --- Variáveis de Leitura da Telemetria ---
    int current_mode_ = 0;
    bool current_engage_status_ = false;
    float current_velocity_ = 0.0f;
    int current_turn_signal_ = 0;
    int current_hazard_signal_ = 0;
    int current_gear_ = 0;

    // --- Variáveis de Retenção de Estado (A tua Lógica) ---
    bool is_telemetry_valid_ = false;
    bool target_engage_state_ = false;
    bool last_received_engage_button_ = false;
    bool last_received_uplink_button_ = false;

    int target_gear_ = 0;
    int target_turn_signal_ = 1; 
    int last_received_turn_button_ = 0;
    int current_uplink_mode_ = 2;

    // --- Interfaces ROS 2 ---
    rclcpp::Subscription<TeleopCommand>::SharedPtr sub_filtered_command_;
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
            target_gear_ = 0;
            target_turn_signal_ = 1;
            last_received_turn_button_ = 0;
            last_received_uplink_button_ = false;
            current_uplink_mode_ = 2;
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
    void filtered_command_callback(const TeleopCommand::SharedPtr msg)
    {
        auto start_time = this->now();

        auto final_msg = std::make_unique<TeleopCommand>();

        final_msg->header.stamp = start_time; 
        final_msg->header.frame_id = "comand_gate";  
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
        if (current_mode_ == 4) {
            
            final_msg->target_velocity = msg->target_velocity;
            final_msg->brake_factor = msg->brake_factor;
            final_msg->target_steering_angle = msg->target_steering_angle;

            int requested_gear = msg->gear;

            if (requested_gear != 0) {
                if (current_gear_ == 0) {
                    if (requested_gear == 2) target_gear_ = 1; 
                } 
                else {
                    if (requested_gear == 1) target_gear_ = 0; 
                    else if (requested_gear == 2) target_gear_ = 1; 
                    else if (requested_gear == 3) target_gear_ = 2; 
                }
            }
            final_msg->gear = target_gear_;

            int current_button = msg->turn_signal;

            bool pressed_right  = (current_button == 1 && last_received_turn_button_ != 1);
            bool pressed_left   = (current_button == 2 && last_received_turn_button_ != 2);
            bool pressed_hazard = (current_button == 3 && last_received_turn_button_ != 3);

            if (pressed_right) {
                target_turn_signal_ = (current_turn_signal_ == 3) ? 1 : 3;
            }
            else if (pressed_left) {
                target_turn_signal_ = (current_turn_signal_ == 2) ? 1 : 2;
            }
            else if (pressed_hazard) {
                target_turn_signal_ = (current_hazard_signal_ == 2) ? 1 : 4;
            }

            last_received_turn_button_ = current_button;
            final_msg->turn_signal = target_turn_signal_;

            bool pressed_uplink = (msg->uplink_mode == 1);

            if (pressed_uplink && !last_received_uplink_button_){
                current_uplink_mode_ = (current_uplink_mode_ == 2) ? 3 : 2;
            }

            last_received_uplink_button_ = pressed_uplink;

            final_msg->uplink_mode = current_uplink_mode_;

        } else {
            final_msg->target_velocity = 0.0f;
            final_msg->brake_factor = 0.0f;
            final_msg->target_steering_angle = 0.0f;
            
            target_gear_ = 0;
            final_msg->gear = 0;
            
            target_turn_signal_ = 1;
            final_msg->turn_signal = 1;

            last_received_uplink_button_ = false;
            current_uplink_mode_ = 2; 
            final_msg->uplink_mode = 2;
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
    auto node = std::make_shared<ComandGate>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}