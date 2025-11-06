#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include <iostream>
#include <string>
#include <cmath>
#include <limits>
#include <thread>
#include <atomic>

using Float32 = std_msgs::msg::Float32;
using Bool = std_msgs::msg::Bool;

class KeyboardPublisherNode : public rclcpp::Node
{
public:
    KeyboardPublisherNode() : Node("keyboard_publisher_node")
    {
        // Publica a velocidade desejada (Float32) e o estado do engage (Bool)
        pub_vlc_ = this->create_publisher<Float32>("/teleop/target_velocity", 10);
        pub_engage_ = this->create_publisher<Bool>("/teleop/engage_command", 1);
        
        RCLCPP_INFO(this->get_logger(), "Nó de Input iniciado. Comandos serao publicados.");
        
        // Thread separada para ler input
        input_thread_ = std::thread([this]() { this->read_input(); });
    }

    ~KeyboardPublisherNode()
    {
        running_ = false;
        if (input_thread_.joinable())
            input_thread_.join();
    }

private:
    // Velocidade de cruzeiro atual. Inicializamos a 0.0, mas o valor real sera gerido aqui.
    float vlc = 0.0f; 
    std::atomic<bool> engage_state{false}; // Estado do Engage
    
    // Constantes de controle (replicadas para dar feedback no terminal)
    const float MAX_VLC = 5.0f;
    const float MIN_VLC = 0.0f;
    const float STEP_VLC = 0.5f;

    rclcpp::Publisher<Float32>::SharedPtr pub_vlc_;
    rclcpp::Publisher<Bool>::SharedPtr pub_engage_;
    std::thread input_thread_;
    std::atomic<bool> running_{true};

    void read_input()
    {
        char input;
        while (rclcpp::ok() && running_)
        {
            // Feedback de status e comandos
            std::cout << "\nComandos: (E)ngage/Disengage, (W) Aumenta Vlc, (S) Diminui Vlc, (Q) Sair\n";
            std::cout << "STATUS: [Engage: " << (engage_state ? "ON" : "OFF") << "] [Vlc: " << vlc << " m/s]\n";
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
            else if (upper_input == 'Q')
            {
                running_ = false;
                break;
            }

            // LIMPA O RESTO DA LINHA DE INPUT para evitar conflitos
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }

    void toggle_engage()
    {
        engage_state = !engage_state;
        auto msg = std::make_unique<Bool>();
        msg->data = engage_state;
        pub_engage_->publish(std::move(msg));
        RCLCPP_INFO(this->get_logger(), "Comando Engage publicado: %s", engage_state ? "TRUE" : "FALSE");
    }

    void ChangeVlc(float change)
    {
        // 1. Calcular a nova velocidade e garantir que ela esteja dentro dos limites
        float new_vlc = vlc + change;
        vlc = std::clamp(new_vlc, MIN_VLC, MAX_VLC);

        // 2. Publicar a velocidade desejada (apenas quando muda)
        auto msg = std::make_unique<Float32>();
        msg->data = vlc;
        pub_vlc_->publish(std::move(msg));

        RCLCPP_INFO(this->get_logger(), "Comando de velocidade publicado: %.2f m/s", vlc);
    }
};

int main(int argc, char *argv[])
{
    // Adiciona o limite de input para a classe std::cin
    using std::numeric_limits;
    using std::streamsize;
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}