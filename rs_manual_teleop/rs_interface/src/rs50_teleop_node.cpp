// Lê o volante Logitech RS50 (/joy_throttled) e publica os comandos crus em /teleop/raw_command.

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <algorithm>
#include <memory>

#include "teleop_msgs/msg/teleop_command.hpp"
#include "teleop_msgs/msg/command_enums.hpp"


using Joy = sensor_msgs::msg::Joy;
using TeleopCommand = teleop_msgs::msg::TeleopCommand;
using CmdEnums = teleop_msgs::msg::CommandEnums;

class RS50TeleopNode : public rclcpp::Node
{
public:
    RS50TeleopNode() : Node("rs50_teleop_node")
    {
        pub_raw_command_ = this->create_publisher<TeleopCommand>("/teleop/raw_command", 10);

        sub_joy_ = this->create_subscription<Joy>(
            "/joy_throttled", 10, 
            std::bind(&RS50TeleopNode::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Nó RS50 Teleop iniciado. Mapeamento de controlo ativo. Publicando em /teleop/raw_command.");
    }

private:
    // mapeamento do RS50 (índices do joy_node)

    // eixos entre -1.0 e 1.0
    const int AXIS_STEERING =  0;   // volante: 0 ao centro, -1 direita, 1 esquerda
    const int AXIS_THROTTLE = 3;   // acelerador, 1 em repouso
    const int AXIS_BRAKE = 4;      // travão, 1 em repouso
    
    const int BUTTON_ENGAGE_1 = 6; // R2
    const int BUTTON_ENGAGE_2 = 7; // L2
    const int BUTTON_TURN_SIGNAL_RIGHT = 10; // R3
    const int BUTTON_TURN_SIGNAL_LEFT = 11; // L3
    const int BUTTON_HAZARD_SIGNAL = 27; // botão Enter

    const int GEAR_REVERSE = 5; // patilha esquerda
    const int GEAR_DRIVE = 4; // patilha direita
    const int PARKING = 2; // círculo

    const int UPLINK_MODE = 0; // X, alterna vídeo / pointcloud

    const float MAX_STEERING_RAD = 0.5f; // ~28.6 graus

    u_int16_t seq_num_ = 1;
    
    rclcpp::Publisher<TeleopCommand>::SharedPtr pub_raw_command_;
    rclcpp::Subscription<Joy>::SharedPtr sub_joy_;
  
    void joy_callback(const Joy::SharedPtr msg)
    {

        // timestamp logo à entrada, para as métricas
        auto start_time = this->now();

        // o RS50 tem 10 eixos e 79 botões
        if (msg->axes.size() < 10 || msg->buttons.size() < 79) {
            RCLCPP_WARN_ONCE(this->get_logger(), "JOY message incomplete.");
            return;
        }
        
        // engage: L2 e R2 premidos ao mesmo tempo
        bool engage_button_1 = msg->buttons[BUTTON_ENGAGE_1];
        bool engage_button_2 = msg->buttons[BUTTON_ENGAGE_2];
        
        bool change_engage_state = engage_button_1 && engage_button_2;
        
        float throttle_input = msg->axes[AXIS_THROTTLE];
        float brake_input = msg->axes[AXIS_BRAKE];

        // pedais de [1 repouso, -1 fundo] para [0, 1]
        double normalized_throttle = abs((throttle_input - 1.0) / 2.0);
        double normalized_brake = abs((brake_input - 1.0) / 2.0);
        
        float target_vlc = 0.0f;
        
        // com travão carregado ou acelerador solto a velocidade alvo é 0
        if (normalized_brake > 0.05 || normalized_throttle < 0.05) {
            target_vlc = 0.0f;
        } 
        else {
            target_vlc = static_cast<float>(normalized_throttle);
        }
        
        float steering_input = msg->axes[AXIS_STEERING];
        float target_steering_angle = steering_input * MAX_STEERING_RAD;

        bool drive = msg->buttons[GEAR_DRIVE];
        bool reverse = msg->buttons[GEAR_REVERSE];
        bool park = msg->buttons[PARKING];

        int new_gear = CmdEnums::GEAR_NONE;

        if (park) {
            new_gear = CmdEnums::GEAR_PARK;
        }           
        else if (drive) {
            new_gear = CmdEnums::GEAR_DRIVE;
        }
        else if (reverse) {
            new_gear = CmdEnums::GEAR_REVERSE;
        }

        bool turn_right = msg->buttons[BUTTON_TURN_SIGNAL_RIGHT];
        bool turn_left = msg->buttons[BUTTON_TURN_SIGNAL_LEFT];
        bool hazard_signal = msg->buttons[BUTTON_HAZARD_SIGNAL];

        int turn_signal = CmdEnums::TURN_OFF;

        if (turn_left) {
            turn_signal = CmdEnums::TURN_LEFT;
        }
        else if (turn_right){
            turn_signal = CmdEnums::TURN_RIGHT;
        }
        else if (hazard_signal){
            turn_signal = CmdEnums::TURN_HAZARD;
        }

        bool uplink_mode = msg->buttons[UPLINK_MODE];

        auto teleop_msg = std::make_unique<TeleopCommand>();

        teleop_msg->header.stamp = start_time;
        teleop_msg->header.frame_id = "rs50_teleop";
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
    auto node = std::make_shared<RS50TeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}