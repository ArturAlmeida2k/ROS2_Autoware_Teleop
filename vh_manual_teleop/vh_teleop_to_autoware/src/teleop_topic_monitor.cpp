#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"

using Float32 = std_msgs::msg::Float32;
using Int8 = std_msgs::msg::Int8;

class TeleopTopicMonitorNode : public rclcpp::Node {
public:
    // Definição dos estados de segurança (iguais aos da Safety Gate)
    static constexpr int8_t STATE_OK = 0;
    static constexpr int8_t STATE_WARN = 1;
    static constexpr int8_t STATE_ERROR = 2;

    TeleopTopicMonitorNode() : Node("teleop_topic_monitor_node") {
        // Parâmetros de tempo de limite (em milissegundos)
        this->declare_parameter<double>("warn_timeout_ms", 100.0); // 100ms sem dados = Aviso
        this->declare_parameter<double>("error_timeout_ms", 250.0); // 250ms sem dados = Erro Crítico
        
        warn_timeout_ms_ = this->get_parameter("warn_timeout_ms").as_double();
        error_timeout_ms_ = this->get_parameter("error_timeout_ms").as_double();

        // Subscrição do tópico que serve de "Heartbeat" (batimento cardíaco)
        sub_heartbeat_ = this->create_subscription<Float32>(
            "/teleop/target_velocity", 10,
            std::bind(&TeleopTopicMonitorNode::heartbeat_callback, this, std::placeholders::_1));

        // Publicador do estado de segurança (vai para a Safety Gate)
        pub_safety_state_ = this->create_publisher<Int8>("/teleop/safety_state", rclcpp::QoS(1));

        // Inicia o relógio
        last_msg_time_ = this->now();
        current_state_ = STATE_ERROR; // Começa em erro até receber a primeira mensagem

        // Timer de verificação (Roda a 100Hz / a cada 10ms)
        monitor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&TeleopTopicMonitorNode::check_timeouts, this));
            
        RCLCPP_INFO(this->get_logger(), "Teleop Topic Monitor Node started.");
    }

private:
    double warn_timeout_ms_;
    double error_timeout_ms_;
    rclcpp::Time last_msg_time_;
    int8_t current_state_;

    rclcpp::Subscription<Float32>::SharedPtr sub_heartbeat_;
    rclcpp::Publisher<Int8>::SharedPtr pub_safety_state_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;

    // Sempre que chega uma mensagem, atualizamos o relógio
    void heartbeat_callback(const Float32::SharedPtr /*msg*/) {
        last_msg_time_ = this->now();
    }

    // Função executada a cada 10ms para avaliar a saúde da rede
    void check_timeouts() {
        double elapsed_ms = (this->now() - last_msg_time_).seconds() * 1000.0;
        int8_t new_state = STATE_OK;

        if (elapsed_ms > error_timeout_ms_) {
            new_state = STATE_ERROR;
        } else if (elapsed_ms > warn_timeout_ms_) {
            new_state = STATE_WARN;
        } else {
            new_state = STATE_OK;
        }

        // Logs para informar mudanças de estado no terminal
        if (new_state != current_state_) {
            if (new_state == STATE_ERROR) {
                RCLCPP_ERROR(this->get_logger(), "NETWORK ERROR: Timeout (%.1f ms). Stopping vehicle.", elapsed_ms);
            } else if (new_state == STATE_WARN) {
                RCLCPP_WARN(this->get_logger(), "NETWORK WARN: High Latency (%.1f ms). Limiting velocity.", elapsed_ms);
            } else if (new_state == STATE_OK) {
                RCLCPP_INFO(this->get_logger(), "NETWORK OK: Connection stable.");
            }
            current_state_ = new_state;
        }

        // Publica o estado continuamente para a Safety Gate
        Int8 state_msg;
        state_msg.data = current_state_;
        pub_safety_state_->publish(state_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopTopicMonitorNode>());
    rclcpp::shutdown();
    return 0;
}