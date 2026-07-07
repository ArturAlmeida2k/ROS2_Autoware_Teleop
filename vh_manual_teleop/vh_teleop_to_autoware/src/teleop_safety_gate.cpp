#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include <algorithm>
#include <memory>

#include "msg_manual_teleop/msg/teleop_command.hpp"
#include "msg_manual_teleop/msg/node_metrics.hpp"


using Int8 = std_msgs::msg::Int8;
using TeleopCommand = msg_manual_teleop::msg::TeleopCommand;
using Metrics = msg_manual_teleop::msg::NodeMetrics;

class TeleopSafetyGateNode : public rclcpp::Node {
public:
    static constexpr int8_t STATE_OK = 0;
    static constexpr int8_t STATE_WARN = 1;
    static constexpr int8_t STATE_ERROR = 2;

    TeleopSafetyGateNode() : Node("teleop_safety_gate_node") {
        // Parâmetros de segurança
        this->declare_parameter<float>("warning_velocity_limit", 2.77f); // ~10 km/h
        
        warning_velocity_limit_ = this->get_parameter("warning_velocity_limit").as_double();

        // Subscrições
        // Recebe o comando que vem diretamente da rede (do Decoder)
        sub_teleop_cmd_ = this->create_subscription<TeleopCommand>(
            "/teleop/command", 10,
            std::bind(&TeleopSafetyGateNode::teleop_cmd_callback, this, std::placeholders::_1));

        sub_safety_state_ = this->create_subscription<Int8>(
            "/teleop/safety_state", 10,
            std::bind(&TeleopSafetyGateNode::safety_state_callback, this, std::placeholders::_1));

        // Publicador do comando seguro (ainda com o formato customizado para reter o ID e origin_stamp)
        pub_safe_cmd_ = this->create_publisher<TeleopCommand>("/teleop/safe_command", 10);

        // For metrics
        rclcpp::QoS metrics_qos(10);       
        metrics_qos.best_effort();    
        metrics_qos.durability_volatile();
        pub_metrics_ = this->create_publisher<Metrics>("/metrics/command_decoder", metrics_qos);


        RCLCPP_INFO(this->get_logger(), "Teleop Safety Gate Node started. (Command filtering mode)");
    }

private:
    int8_t current_state_ = STATE_ERROR;
    float warning_velocity_limit_;

    rclcpp::Subscription<TeleopCommand>::SharedPtr sub_teleop_cmd_;
    rclcpp::Subscription<Int8>::SharedPtr sub_safety_state_;
    rclcpp::Publisher<TeleopCommand>::SharedPtr pub_safe_cmd_;
    rclcpp::Publisher<Metrics>::SharedPtr pub_metrics_;

    void safety_state_callback(const Int8::SharedPtr msg) {
        current_state_ = msg->data;
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

    void teleop_cmd_callback(const TeleopCommand::SharedPtr msg) {
        auto start_time = this->now();

        auto safe_msg = std::make_unique<TeleopCommand>(*msg);
        
        // Atualiza o header stamp para o salto atual
        safe_msg->header.stamp = start_time;
        safe_msg->header.frame_id = "safety_gate";

        switch (current_state_) {
            case STATE_OK:
                // Sem restrições
                break;
                
            case STATE_WARN:
                // Limita a velocidade máxima (frente e marcha-atrás)
                safe_msg->target_velocity = std::clamp(
                    safe_msg->target_velocity, 
                    0.0f, 
                    warning_velocity_limit_
                );
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                     "Warning state: Limiting velocity to %.2f m/s", warning_velocity_limit_);
                break;
                
            case STATE_ERROR:
            default:
                // Força paragem do veículo, altera mudanças para Neutro/Park e ativa 4 piscas
                safe_msg->target_velocity = 0.0f;
                safe_msg->brake_factor = 1.0f; // Força travagem máxima no próximo nó
                safe_msg->target_steering_angle = 0.0f;
                safe_msg->engage_command = false;
                safe_msg->turn_signal = 3; // 3 = Hazard signal na tua lógica original
                safe_msg->gear = 0; // 0 = Park
                
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                      "Error state: Forcing stop and hazard lights.");
                break;
        }

        pub_safe_cmd_->publish(std::move(safe_msg));

        publish_metrics(msg->id, start_time, msg->header.stamp);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopSafetyGateNode>());
    rclcpp::shutdown();
    return 0;
}