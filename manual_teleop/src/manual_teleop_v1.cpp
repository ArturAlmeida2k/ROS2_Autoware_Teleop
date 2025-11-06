#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <string>

// #include "autoware_vehicle_msgs/msg/velocity_report.hpp"
// #include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
// #include "geometry_msgs/msg/twist.hpp"
#include <autoware_control_msgs/msg/control.hpp>
// #include <autoware_vehicle_msgs/msg/engage.hpp>
// #include <autoware_vehicle_msgs/msg/gear_command.hpp>
// #include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>

using Control = autoware_control_msgs::msg::Control;
using GateMode = tier4_control_msgs::msg::GateMode;
using EngageSrv = tier4_external_api_msgs::srv::Engage;

class KeyboardInputNode : public rclcpp::Node
{
public:
    KeyboardInputNode() : Node("keyboard_input_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("keyboard_input", 10);
        RCLCPP_INFO(this->get_logger(), "Nó iniciado. Escreve algo e pressiona Enter (CTRL+C para sair).");
        
        // 1. Cliente para chamar o serviço engage
        client_engage_ = this->create_client<EngageSrv>("/api/autoware/set/engage");
        // 2. Publicador para o GateMode
        pub_gate_mode_ = this->create_publisher<GateMode>("/control/gate_mode_cmd", rclcpp::QoS(1));
        // 3. Publicador para o Comando de Controle (Velocidade)
        pub_control_cmd_ = this->create_publisher<Control>("/external/selected/control_cmd", rclcpp::QoS(1));
        
        // 4. Configurar e iniciar o Timer para publicação contínua (100ms = 10Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&KeyboardInputNode::publish_control_command, this)
        );

        // Thread separada para ler input sem bloquear o ROS
        input_thread_ = std::thread([this]() { this->read_input(); });
    }

    ~KeyboardInputNode()
    {
        running_ = false;
        if (input_thread_.joinable())
            input_thread_.join();
    }

private:

    // Variável para armazenar a velocidade de cruzeiro atual (m/s)
    float vlc = 0.0f;
    const float MAX_VLC = 5.0f; // Velocidade máxima (ex: 5.0 m/s = 18 km/h)
    const float MIN_VLC = 0.0f; // Velocidade mínima (0.0 m/s)
    const float STEP_VLC = 0.5f; // Passo de mudança de velocidade (m/s)

    rclcpp::TimerBase::SharedPtr control_timer_; 

    void read_input()
    {
        char input;
        while (rclcpp::ok() && running_)
        {
            // Adiciona feedback de status e comandos ao utilizador
            std::cout << "\nComandos: (E)ngage, (W) Aumenta Vlc, (S) Diminui Vlc, (Q) Sair\n";
            std::cout << "Vlc Atual: " << vlc << " m/s\n";
            std::cout << "> ";
            
            // Tenta ler apenas um caractere.
            if (!(std::cin >> input))
                break;
             
            char upper_input = std::toupper(input);

            if (upper_input == 'E')
            {
                this->enable();
            }
            else if (upper_input == 'W')
            {
                this->ChangeVlc(STEP_VLC);
            }
            else if (upper_input == 'S')
            {
                this->ChangeVlc(-STEP_VLC);
            }
            // LIMPA O RESTO DA LINHA DE INPUT
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }
    void enable()
    {
        // gate mode
        {
        pub_gate_mode_->publish(tier4_control_msgs::build<GateMode>().data(GateMode::EXTERNAL));
        RCLCPP_INFO(this->get_logger(), "Publicou GateMode::EXTERNAL.");
        }
        // engage
        {
        auto req = std::make_shared<EngageSrv::Request>();
        req->engage = true;
        RCLCPP_DEBUG(this->get_logger(), "client request");
        if (!client_engage_->service_is_ready()) {
            RCLCPP_DEBUG(this->get_logger(), "client is unavailable");
            return;
        }
        client_engage_->async_send_request(
            req, []([[maybe_unused]] rclcpp::Client<EngageSrv>::SharedFuture result) {});
        RCLCPP_INFO(this->get_logger(), "Enviou Engage = TRUE.");

        }
    }

    void ChangeVlc(float change)
    {
        // 1. Calcular a nova velocidade e garantir que ela esteja dentro dos limites
        float new_vlc = vlc + change;
        if (new_vlc >= MAX_VLC){
            vlc = MAX_VLC;
        }
        else if(new_vlc <= MIN_VLC){
            vlc = MIN_VLC;
        }
        else{
            vlc = new_vlc;
        }
        RCLCPP_INFO(this->get_logger(), "Velocidade alterada para: %.2f m/s", vlc);
    }

    void publish_control_command()
    {
        // 1. Criar a mensagem de comando de controle
        auto control_cmd = std::make_unique<Control>();
        
        control_cmd->stamp = this->now();
        
        // Define a velocidade longitudinal (x) com o valor atual de cruzeiro
        control_cmd->longitudinal.velocity = vlc;
        
        // Manter a aceleração e o ângulo de direção em zero (ou em valores seguros)
        control_cmd->longitudinal.acceleration = 0.0; 
        control_cmd->lateral.steering_tire_angle = 0.0;

        // 2. Publicar o comando
        pub_control_cmd_->publish(std::move(control_cmd));
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    rclcpp::Client<EngageSrv>::SharedPtr client_engage_;
    rclcpp::Publisher<GateMode>::SharedPtr pub_gate_mode_;
    rclcpp::Publisher<Control>::SharedPtr pub_control_cmd_; 
    
    std::thread input_thread_;
    std::atomic<bool> running_{true};
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardInputNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}