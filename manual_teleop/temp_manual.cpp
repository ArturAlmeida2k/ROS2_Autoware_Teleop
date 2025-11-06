#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include <iostream>
#include <string>

#include <autoware_control_msgs/msg/control.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>

using Control = autoware_control_msgs::msg::Control;
using GateMode = tier4_control_msgs::msg::GateMode;
using EngageSrv = tier4_external_api_msgs::srv::Engage;
using Float32 = std_msgs::msg::Float32;
using Bool = std_msgs::msg::Bool;

class AutowareControllerNode : public rclcpp::Node
{
public:
    AutowareControllerNode() : Node("autoware_controller_node")
    {
        // Subscrição para receber a velocidade desejada do Nó de Input
        sub_vlc_ = this->create_subscription<Float32>(
            "/teleop/target_velocity", 10, 
            std::bind(&AutowareControllerNode::vlc_callback, this, std::placeholders::_1));

        // Subscrição para receber o comando Engage do Nó de Input
        sub_engage_ = this->create_subscription<Bool>(
            "/teleop/engage_command", 1, 
            std::bind(&AutowareControllerNode::engage_callback, this, std::placeholders::_1));

        // 1. Cliente para chamar o serviço engage do Autoware
        client_engage_ = this->create_client<EngageSrv>("/api/autoware/set/engage");
        // 2. Publicador para o GateMode do Autoware
        pub_gate_mode_ = this->create_publisher<GateMode>("/control/gate_mode_cmd", rclcpp::QoS(1));
        // 3. Publicador para o Comando de Controle (Velocidade) do Autoware
        pub_control_cmd_ = this->create_publisher<Control>("/external/selected/control_cmd", rclcpp::QoS(1));
        
        // 4. Configurar e iniciar o Timer para publicação contínua (100ms = 10Hz)
        // ESSENCIAL: Este timer garante que o comando chegue ao Autoware continuamente.
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&AutowareControllerNode::publish_control_command, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Nó de Controle iniciado. Aguardando comandos em /teleop/...");
    }

private:
    float vlc_target_ = 0.0f;
    bool engage_target_ = false;

    // Publicadores e Clientes para o Autoware
    rclcpp::Client<EngageSrv>::SharedPtr client_engage_;
    rclcpp::Publisher<GateMode>::SharedPtr pub_gate_mode_;
    rclcpp::Publisher<Control>::SharedPtr pub_control_cmd_; 

    // Subscrever dos comandos do Nó de Input
    rclcpp::Subscription<Float32>::SharedPtr sub_vlc_;
    rclcpp::Subscription<Bool>::SharedPtr sub_engage_;
    
    // Timer para o loop de publicação (10 Hz)
    rclcpp::TimerBase::SharedPtr control_timer_; 


    void vlc_callback(const Float32::SharedPtr msg)
    {
        vlc_target_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Velocidade-alvo recebida: %.2f m/s", vlc_target_);
    }

    void engage_callback(const Bool::SharedPtr msg)
    {
        engage_target_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Comando Engage recebido: %s", engage_target_ ? "TRUE" : "FALSE");
        this->set_autoware_engage(engage_target_);
    }

    void set_autoware_engage(bool engage)
    {
        // 1. Publicar GateMode::EXTERNAL (necessário antes de engatar no Autoware)
        pub_gate_mode_->publish(tier4_control_msgs::build<GateMode>().data(GateMode::EXTERNAL));
        RCLCPP_INFO(this->get_logger(), "Publicou GateMode::EXTERNAL.");
        
        // 2. Chamar o serviço Engage
        auto req = std::make_shared<EngageSrv::Request>();
        req->engage = engage;

        if (!client_engage_->service_is_ready()) {
            RCLCPP_ERROR(this->get_logger(), "Serviço /api/autoware/set/engage indisponível.");
            return;
        }
        client_engage_->async_send_request(
            req, []([[maybe_unused]] rclcpp::Client<EngageSrv>::SharedFuture result) {});
        RCLCPP_INFO(this->get_logger(), "Serviço Engage chamado com valor: %s", engage ? "TRUE" : "FALSE");
    }

    /**
     * @brief Publica o comando de controle continuamente (chamado pelo timer).
     */
    void publish_control_command()
    {
        // 1. Criar a mensagem de comando de controle
        auto control_cmd = std::make_unique<Control>();
        
        control_cmd->stamp = this->now();
        
        // Define a velocidade longitudinal (x) com o valor alvo
        // Nota: A velocidade é 0.0 se não estiver em engage ou se o input for 0.0
        control_cmd->longitudinal.velocity = vlc_target_;
        
        // Manter a aceleração e o ângulo de direção em zero por simplicidade
        control_cmd->longitudinal.acceleration = 0.0; 
        control_cmd->lateral.steering_tire_angle = 0.0;

        // 2. Publicar o comando
        pub_control_cmd_->publish(std::move(control_cmd));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutowareControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
