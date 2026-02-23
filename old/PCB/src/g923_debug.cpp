#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip> // Adicionado para std::fixed e std::setprecision

using Joy = sensor_msgs::msg::Joy;

class G923TeleopNode : public rclcpp::Node
{
public:
    G923TeleopNode() : Node("teleop_g923_node")
    {
        // Subscrição do joystick (publicado pelo joy_node)
        sub_joy_ = this->create_subscription<Joy>(
            "/joy", 10, 
            std::bind(&G923TeleopNode::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "--- NÓ DE DEBUG G923 INICIADO ---");
        RCLCPP_INFO(this->get_logger(), "Mova o volante e pedais para ver os índices.");
        RCLCPP_INFO(this->get_logger(), "----------------------------------");
    }

private:
    rclcpp::Subscription<Joy>::SharedPtr sub_joy_;

    /**
     * @brief Callback que recebe as mensagens do joystick e imprime os valores brutos.
     */
    void joy_callback(const Joy::SharedPtr msg)
    {
        std::stringstream ss;
        
        // --- Imprimir Eixos (Volante e Pedais) ---
        ss << "EIXOS (" << msg->axes.size() << "): ";
        for (size_t i = 0; i < msg->axes.size(); ++i) {
            // Formata a saída: [Eixo i: Valor]
            // Certifique-se de que o setprecision está disponível (incluído no código)
            ss << "[" << i << ": " << std::fixed << std::setprecision(2) << msg->axes[i] << "] ";
        }
        
        // --- Adicionar a quebra de linha para separação visual ---
        ss << "\nBOTÕES (" << msg->buttons.size() << "): ";
        
        // --- Imprimir Botões ---
        for (size_t i = 0; i < msg->buttons.size(); ++i) {
            // Formata a saída: [Botão i: Estado]
            if (msg->buttons[i] == 1) {
                 ss << "[" << i << ": ON] ";
            } else {
                 ss << "[" << i << ": OFF] ";
            }
        }

        // Publica no log do ROS 2
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<G923TeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}