#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include <iostream>
#include <string>
#include <limits> // Necessário para std::numeric_limits
#include <cmath>   // Necessário para std::clamp (ou use if/else se não quiser incluir cmath)

using Float32 = std_msgs::msg::Float32;
using Bool = std_msgs::msg::Bool;

class KeyboardPublisherNode : public rclcpp::Node
{
public:
    KeyboardPublisherNode() : Node("keyboard_publisher_node")
    {
        // Publicador para Velocidade Alvo
        pub_vlc_ = this->create_publisher<Float32>("/teleop/target_velocity", 10);
        
        // NOVO: Publicador para Ângulo de Direção Alvo
        pub_steering_ = this->create_publisher<Float32>("/teleop/target_steering_angle", 10);
        
        // Publicador para o comando Engage
        pub_engage_ = this->create_publisher<Bool>("/teleop/engage_command", 1);

        RCLCPP_INFO(this->get_logger(), "Nó de Input do Teclado iniciado.");
        RCLCPP_INFO(this->get_logger(), "Comandos: (E)ngage, (W/S) Velocidade, (A/D) Direção.");
        
        // Thread separada para ler input sem bloquear o ROS
        input_thread_ = std::thread([this]() { this->read_input(); });
    }

    ~KeyboardPublisherNode()
    {
        running_ = false;
        if (input_thread_.joinable())
            input_thread_.join();
    }

private:
    // --- Variáveis de Velocidade ---
    float vlc_target_ = 0.0f;
    const float MAX_VLC = 5.0f;   // Velocidade máxima (m/s)
    const float MIN_VLC = 0.0f;   // Velocidade mínima (m/s)
    const float STEP_VLC = 0.5f;  // Passo de mudança de velocidade (m/s)

    // --- Variáveis de Direção (Steering) ---
    float steering_angle_target_ = 0.0f; // Ângulo alvo em RADIANOS
    // Estes limites são sugestões. Ajuste-os com base no veículo (ex: ~28 graus)
    const float MAX_STEERING = 0.5f;  // Máximo em Radianos (~28.6 graus)
    const float MIN_STEERING = -0.5f; // Mínimo em Radianos (~-28.6 graus)
    const float STEP_STEERING = 0.05f; // Passo de mudança em Radianos (~2.8 graus)
    
    // --- Outras Variáveis ---
    bool is_engaged_ = false;

    // Publicadores
    rclcpp::Publisher<Float32>::SharedPtr pub_vlc_;
    rclcpp::Publisher<Float32>::SharedPtr pub_steering_; // NOVO
    rclcpp::Publisher<Bool>::SharedPtr pub_engage_;
    
    std::thread input_thread_;
    std::atomic<bool> running_{true};


    // --- Funções de Leitura e Controle ---
    void read_input()
    {
        char input;
        while (rclcpp::ok() && running_)
        {
            // Apresentar o estado atual
            std::cout << "\n------------------------------------------------\n";
            std::cout << "STATUS: " << (is_engaged_ ? "ENGATADO (E)" : "DESENGATADO (E)") << "\n";
            std::cout << "Velocidade Alvo: " << vlc_target_ << " m/s (W/S)\n";
            std::cout << "Direção Alvo: " << steering_angle_target_ << " rad (A/D)\n";
            std::cout << "------------------------------------------------\n";
            std::cout << "> ";
            
            // Tenta ler apenas um caractere.
            if (!(std::cin >> input))
                break;
             
            char upper_input = std::toupper(input);

            if (upper_input == 'E')
            {
                this->toggle_engage();
            }
            else if (upper_input == 'W')
            {
                this->ChangeVlc(STEP_VLC);
            }
            else if (upper_input == 'S')
            {
                this->ChangeVlc(-STEP_VLC);
            }
            else if (upper_input == 'A') // Esquerda
            {
                this->ChangeSteering(STEP_STEERING);
            }
            else if (upper_input == 'D') // Direita
            {
                this->ChangeSteering(-STEP_STEERING);
            }
            
            // LIMPA O RESTO DA LINHA DE INPUT para evitar que comandos fiquem presos
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }

    void toggle_engage()
    {
        is_engaged_ = !is_engaged_;
        auto msg = std::make_unique<Bool>();
        msg->data = is_engaged_;
        pub_engage_->publish(std::move(msg));
        RCLCPP_INFO(this->get_logger(), "Comando Engage enviado: %s", is_engaged_ ? "TRUE" : "FALSE");
    }

    void ChangeVlc(float change)
    {
        // 1. Calcular a nova velocidade
        float new_vlc = vlc_target_ + change;
        
        // 2. Aplicar limites
        if (new_vlc >= MAX_VLC){
            vlc_target_ = MAX_VLC;
        }
        else if(new_vlc <= MIN_VLC){
            vlc_target_ = MIN_VLC;
        }
        else{
            vlc_target_ = new_vlc;
        }
        
        // 3. Publicar a nova velocidade alvo
        auto msg = std::make_unique<Float32>();
        msg->data = vlc_target_;
        pub_vlc_->publish(std::move(msg));
        RCLCPP_INFO(this->get_logger(), "Velocidade alvo publicada: %.2f m/s", vlc_target_);
    }

    void ChangeSteering(float change)
    {
        // 1. Calcular o novo ângulo
        float new_angle = steering_angle_target_ + change;

        // 2. Aplicar limites
        if (new_angle >= MAX_STEERING){
            steering_angle_target_ = MAX_STEERING;
        }
        else if(new_angle <= MIN_STEERING){
            steering_angle_target_ = MIN_STEERING;
        }
        else{
            steering_angle_target_ = new_angle;
        }

        // 3. Publicar o novo ângulo alvo
        auto msg = std::make_unique<Float32>();
        msg->data = steering_angle_target_;
        pub_steering_->publish(std::move(msg));
        RCLCPP_INFO(this->get_logger(), "Direção alvo publicada: %.4f rad", steering_angle_target_);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}