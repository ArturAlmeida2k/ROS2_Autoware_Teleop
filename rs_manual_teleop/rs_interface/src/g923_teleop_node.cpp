// Lê o volante Logitech G923 (/joy_throttled) e publica os comandos crus em /teleop/raw_command.

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <algorithm>
#include <memory>

#include "teleop_msgs/msg/teleop_command.hpp"
#include "teleop_msgs/msg/command_enums.hpp"

using Joy = sensor_msgs::msg::Joy;
using TeleopCommand = teleop_msgs::msg::TeleopCommand;
using CmdEnums = teleop_msgs::msg::CommandEnums;

class G923TeleopNode : public rclcpp::Node
{
public:
    G923TeleopNode() : Node("g923_teleop_node")
    {
        pub_raw_command_ = this->create_publisher<TeleopCommand>("/teleop/raw_command", 10);

        sub_joy_ = this->create_subscription<Joy>(
            "/joy_throttled", 10, 
            std::bind(&G923TeleopNode::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Nó G923 Teleop iniciado. Mapeamento de controlo ativo. Publicando em /teleop/raw_command.");
    }

private:
    // mapeamento do G923 (índices do joy_node)

    // eixos entre -1.0 e 1.0
    const int AXIS_STEERING = 0;   // volante: 0 ao centro, -1 direita, 1 esquerda
    const int AXIS_CLUTCH = 1;     // embraiagem, -1 em repouso
    const int AXIS_THROTTLE = 2;   // acelerador, -1 em repouso
    const int AXIS_BRAKE = 3;      // travão, -1 em repouso
    
    const int BUTTON_ENGAGE_1 = 6; // R2
    const int BUTTON_ENGAGE_2 = 7; // L2
    const int BUTTON_TURN_SIGNAL_RIGHT = 10; // R3
    const int BUTTON_TURN_SIGNAL_LEFT = 11; // L3
    const int BUTTON_HAZARD_SIGNAL = 23; // botão Enter
    const int BUTTON_UPLINK_MODE = 8; // alterna vídeo / pointcloud

    const int GEAR_DRIVE = 4; // patilha direita
    const int GEAR_REVERSE = 5; // patilha esquerda
    const int GEAR_PARKED_AXIS = 5; // eixo do D-pad (cima/baixo)
    
    const float MAX_STEERING_RAD = 0.5f; // ~28.6 graus

    u_int16_t seq_num_ = 1;
    
    rclcpp::Publisher<TeleopCommand>::SharedPtr pub_raw_command_;
    rclcpp::Subscription<Joy>::SharedPtr sub_joy_;
  
    void joy_callback(const Joy::SharedPtr msg)
    {

        // timestamp logo à entrada, para as métricas
        auto start_time = this->now();

        // o G923 tem 6 eixos e 24 botões
        if (msg->axes.size() < 6 || msg->buttons.size() < 24) {
            RCLCPP_WARN_ONCE(this->get_logger(), "JOY message incomplete.");
            return;
        }
        
        // engage: L2 e R2 premidos ao mesmo tempo
        bool engage_button_1 = msg->buttons[BUTTON_ENGAGE_1];
        bool engage_button_2 = msg->buttons[BUTTON_ENGAGE_2];
        
        bool change_engage_state = engage_button_1 && engage_button_2;
        
        float throttle_input = msg->axes[AXIS_THROTTLE];
        float brake_input = msg->axes[AXIS_BRAKE];

        // de [-1 repouso, 1 fundo] para [0, 1]
        double normalized_throttle = (throttle_input + 1.0) / 2.0;
        double normalized_brake = (brake_input + 1.0) / 2.0;
        
        float target_vlc = 0.0f;
        
        // com o travão carregado a velocidade alvo é 0
        if (normalized_brake > 0.05) {
            target_vlc = 0.0f;
        } 
        else {
            target_vlc = static_cast<float>(normalized_throttle);
        }
        
        float steering_input = msg->axes[AXIS_STEERING];
        float target_steering_angle = steering_input * MAX_STEERING_RAD;

        bool drive_button = msg->buttons[GEAR_DRIVE];
        bool reverse_button = msg->buttons[GEAR_REVERSE];
        int parking_axes = msg->axes[GEAR_PARKED_AXIS];
        float clutch = msg->axes[AXIS_CLUTCH];

        double normalized_clutch = (clutch + 1.0) / 2.0;

        int new_gear = CmdEnums::GEAR_NONE;

        // mudanças só com a embraiagem quase a fundo
        if (normalized_clutch >= 0.9){
            // D-pad cima: sai de Parking para Drive
            if (parking_axes == 1) {
                new_gear = CmdEnums::GEAR_DRIVE;
            }
            // D-pad baixo: Parking
            else if (parking_axes == -1) {
                new_gear = CmdEnums::GEAR_PARK;
            }
            // patilhas: Drive / Reverse
           
            else if (drive_button) {
                    new_gear = CmdEnums::GEAR_DRIVE;
            }
            
            else if (reverse_button) {
                    new_gear = CmdEnums::GEAR_REVERSE;
            }
        }

        bool turn_right = msg->buttons[BUTTON_TURN_SIGNAL_RIGHT];
        bool turn_left = msg->buttons[BUTTON_TURN_SIGNAL_LEFT];
        bool hazard_signal = msg->buttons[BUTTON_HAZARD_SIGNAL];
        bool uplink_mode = msg->buttons[BUTTON_UPLINK_MODE];

        int turn_signal = CmdEnums::TURN_OFF;

        if (turn_right) {
            turn_signal = CmdEnums::TURN_RIGHT;

        }
        else if (turn_left){
            turn_signal = CmdEnums::TURN_LEFT;
        }
        else if (hazard_signal){
            turn_signal = CmdEnums::TURN_HAZARD;
        }

        auto teleop_msg = std::make_unique<TeleopCommand>();

        teleop_msg->header.stamp = start_time;
        teleop_msg->header.frame_id = "g923_teleop";
        teleop_msg->origin_stamp = start_time;
        teleop_msg->id = seq_num_++;
        teleop_msg->target_velocity = target_vlc;
        teleop_msg->brake_factor = static_cast<float>(normalized_brake);
        teleop_msg->target_steering_angle = target_steering_angle;
        teleop_msg->engage_command = change_engage_state;
        teleop_msg->uplink_mode = uplink_mode;
        teleop_msg->gear = new_gear;
        teleop_msg->turn_signal = turn_signal;

        pub_raw_command_->publish(std::move(teleop_msg));
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