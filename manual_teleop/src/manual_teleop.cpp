#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include <iostream>
#include <string>
#include <cmath> 
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp> 
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>

using Control = autoware_control_msgs::msg::Control;
using GateMode = tier4_control_msgs::msg::GateMode;
using EngageSrv = tier4_external_api_msgs::srv::Engage;
using Float32 = std_msgs::msg::Float32;
using Bool = std_msgs::msg::Bool;
using VelocityReport = autoware_vehicle_msgs::msg::VelocityReport; 

class AutowareControllerNode : public rclcpp::Node
{
public:
    AutowareControllerNode() : Node("autoware_controller_node")
    {
        // Subscrição para Velocidade Alvo
        sub_vlc_target_ = this->create_subscription<Float32>(
            "/teleop/target_velocity", 10, 
            std::bind(&AutowareControllerNode::vlc_target_callback, this, std::placeholders::_1));

        // Subscrição para Ângulo de Direção Alvo 
        sub_steering_target_ = this->create_subscription<Float32>(
            "/teleop/target_steering_angle", 10, 
            std::bind(&AutowareControllerNode::steering_target_callback, this, std::placeholders::_1));

        // Subscrição para o estado de engage
        sub_engage_target_ = this->create_subscription<Bool>(
            "/teleop/engage_command", 1, 
            std::bind(&AutowareControllerNode::engage_callback, this, std::placeholders::_1));
        
        // Subscrição para ler a velocidade atual do veículo 
        sub_vlc_current_ = this->create_subscription<VelocityReport>(
            "/vehicle/status/velocity_status", 10, 
            std::bind(&AutowareControllerNode::vlc_report_callback, this, std::placeholders::_1));

        // 1. Cliente para chamar o serviço engage do Autoware
        client_engage_ = this->create_client<EngageSrv>("/api/autoware/set/engage");
        // 2. Publicador para o GateMode do Autoware
        pub_gate_mode_ = this->create_publisher<GateMode>("/control/gate_mode_cmd", rclcpp::QoS(1));
        // 3. Publicador para o Comando de Controle (Velocidade, Aceleração, Direção)
        pub_control_cmd_ = this->create_publisher<Control>("/external/selected/control_cmd", rclcpp::QoS(1));
        
        // 4. Configurar e iniciar o Timer para publicação contínua (100ms = 10Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&AutowareControllerNode::publish_control_command, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Nó de Controle Autoware iniciado. Aguardando comandos em /teleop/...");
    }

private:
    float vlc_target_ = 0.0f;       // Velocidade alvo do utilizador
    float vlc_current_ = 0.0f;      // Velocidade atual do veículo
    float steering_angle_target_ = 0.0f; // Ângulo de direção alvo
    bool engage_target_ = false;

    // Constante do controlador Proporcional (P) para velocidade
    const double KP_GAIN = 0.5; 
    
    // Publicadores e Clientes para o Autoware
    rclcpp::Client<EngageSrv>::SharedPtr client_engage_;
    rclcpp::Publisher<GateMode>::SharedPtr pub_gate_mode_;
    rclcpp::Publisher<Control>::SharedPtr pub_control_cmd_; 

    // Subscrições
    rclcpp::Subscription<Float32>::SharedPtr sub_vlc_target_;
    rclcpp::Subscription<Float32>::SharedPtr sub_steering_target_;
    rclcpp::Subscription<Bool>::SharedPtr sub_engage_target_;
    rclcpp::Subscription<VelocityReport>::SharedPtr sub_vlc_current_;
    
    // Timer para o loop de publicação (10 Hz)
    rclcpp::TimerBase::SharedPtr control_timer_; 


    void vlc_target_callback(const Float32::SharedPtr msg)
    {
        vlc_target_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Velocidade-alvo recebida: %.2f m/s", vlc_target_);
    }

    void steering_target_callback(const Float32::SharedPtr msg) 
    {
        steering_angle_target_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Direção-alvo recebida: %.4f rad", steering_angle_target_);
    }
    
    void vlc_report_callback(const VelocityReport::SharedPtr msg)
    {
        vlc_current_ = msg->longitudinal_velocity;
    }

    void engage_callback(const Bool::SharedPtr msg)
    {
        engage_target_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Comando Engage recebido: %s", engage_target_ ? "TRUE" : "FALSE");
        this->set_autoware_engage(engage_target_);
    }

    void set_autoware_engage(bool engage)
    {
        // Publicar GateMode::EXTERNAL (necessário antes de engatar no Autoware)
        pub_gate_mode_->publish(tier4_control_msgs::build<GateMode>().data(GateMode::EXTERNAL));
        RCLCPP_INFO(this->get_logger(), "Publicou GateMode::EXTERNAL.");
        
        // Chamar o serviço Engage
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
        
        // --- LÓGICA DE VELOCIDADE E ACELERAÇÃO ---
        
        // Se não estiver engaged OU a velocidade alvo for 0.0, forcamos o ângulo a 0.0 e a aceleração a 0.0 para garantir segurança.
        if (!engage_target_ && vlc_target_ == 0.0f) {
            control_cmd->longitudinal.velocity = 0.0;
            control_cmd->longitudinal.acceleration = 0.0;
            control_cmd->lateral.steering_tire_angle = 0.0;
        } else {
            // Velocidade Alvo
            control_cmd->longitudinal.velocity = vlc_target_;

            // Cálculo da aceleração usando P-Controller
            double velocity_error = static_cast<double>(vlc_target_) - vlc_current_;
            double acceleration_cmd = KP_GAIN * velocity_error;

            // Limitar a aceleração/desaceleração
            acceleration_cmd = std::clamp(acceleration_cmd, -1.0, 1.0); 
            control_cmd->longitudinal.acceleration = acceleration_cmd;

            // --- LÓGICA DE DIREÇÃO ---
            control_cmd->lateral.steering_tire_angle = steering_angle_target_;
        }


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