// Lê o comando Xbox (/joy_throttled) e publica os comandos crus em /teleop/raw_command.
// Engage com Back + Start; mudanças com X premido + A (Drive), B (Reverse) ou D-pad cima/baixo (Drive/Parking).

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <algorithm>
#include <cmath>
#include <memory>

#include "teleop_msgs/msg/teleop_command.hpp"
#include "teleop_msgs/msg/command_enums.hpp"

using Joy = sensor_msgs::msg::Joy;
using TeleopCommand = teleop_msgs::msg::TeleopCommand;
using CmdEnums = teleop_msgs::msg::CommandEnums;

class XboxTeleopNode : public rclcpp::Node
{
public:
    XboxTeleopNode() : Node("xbox_teleop_node")
    {
        pub_raw_command_ = this->create_publisher<TeleopCommand>("/teleop/raw_command", 10);

        sub_joy_ = this->create_subscription<Joy>(
            "/joy_throttled", 10,
            std::bind(&XboxTeleopNode::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "Nó Xbox Teleop iniciado. Publicando em /teleop/raw_command.");
    }

private:
    // eixos
    const int AXIS_STEERING  = 0;  // Left Stick X
    const int AXIS_BRAKE     = 2;  // LT, travão
    const int AXIS_THROTTLE  = 5;  // RT, acelerador
    const int AXIS_DPAD_X    = 6;  // D-Pad horizontal
    const int AXIS_DPAD_Y    = 7;  // D-Pad vertical

    // botões
    const int BUTTON_A       = 0;  // Drive
    const int BUTTON_B       = 1;  // Reverse
    const int BUTTON_X       = 2;  // modificador de mudanças (faz de embraiagem)
    const int BUTTON_Y       = 3;  // Hazard
    const int BUTTON_LB      = 4;  // Pisca esquerdo
    const int BUTTON_RB      = 5;  // Pisca direito
    const int BUTTON_BACK    = 6;  // Engage parte 1
    const int BUTTON_START   = 7;  // Engage parte 2
    const int BUTTON_LS      = 9;  // Alternar vídeo / pointcloud

    const float MAX_STEERING_RAD    = 0.5f;  // ~28.6 graus
    const double TRIGGER_REST_VALUE = 1.0;   // gatilhos em repouso a +1.0, confirmar com ros2 topic echo /joy

    u_int32_t seq_num_ = 1;

    rclcpp::Publisher<TeleopCommand>::SharedPtr pub_raw_command_;
    rclcpp::Subscription<Joy>::SharedPtr sub_joy_;

    void joy_callback(const Joy::SharedPtr msg)
    {
        auto start_time = this->now();

        if (msg->axes.size() < 8 || msg->buttons.size() < 10) {
            RCLCPP_WARN_ONCE(this->get_logger(),
                "Mensagem JOY incompleta. Esperados >= 8 eixos e >= 10 botões.");
            return;
        }

        // engage: Back + Start ao mesmo tempo
        bool engage_button_1    = msg->buttons[BUTTON_BACK];
        bool engage_button_2    = msg->buttons[BUTTON_START];
        bool change_engage_state = engage_button_1 && engage_button_2;

        float throttle_input = msg->axes[AXIS_THROTTLE];
        float brake_input    = msg->axes[AXIS_BRAKE];

        // gatilhos de [1 repouso, -1 fundo] para [0, 1]
        double normalized_throttle = std::abs((throttle_input - TRIGGER_REST_VALUE) / 2.0);
        double normalized_brake    = std::abs((brake_input    - TRIGGER_REST_VALUE) / 2.0);

        float target_vlc = 0.0f;

        // com travão carregado ou acelerador solto a velocidade alvo é 0
        if (normalized_brake > 0.05 || normalized_throttle < 0.05) {
            target_vlc = 0.0f;
        } else {
            target_vlc = static_cast<float>(normalized_throttle);
        }

        float steering_input        = msg->axes[AXIS_STEERING];
        float target_steering_angle = steering_input * MAX_STEERING_RAD;

        // mudanças só com X premido
        bool gear_modifier  = msg->buttons[BUTTON_X];
        bool drive_button   = msg->buttons[BUTTON_A];
        bool reverse_button = msg->buttons[BUTTON_B];
        int  dpad_y         = static_cast<int>(msg->axes[AXIS_DPAD_Y]);

        int new_gear = CmdEnums::GEAR_NONE;

        if (gear_modifier) {
            if (dpad_y == 1) {
                new_gear = CmdEnums::GEAR_DRIVE;
            } else if (dpad_y == -1) {
                new_gear = CmdEnums::GEAR_PARK;
            } else if (drive_button) {
                new_gear = CmdEnums::GEAR_DRIVE;
            } else if (reverse_button) {
                new_gear = CmdEnums::GEAR_REVERSE;
            }
        }

        bool turn_right    = msg->buttons[BUTTON_RB];
        bool turn_left     = msg->buttons[BUTTON_LB];
        bool hazard_signal = msg->buttons[BUTTON_Y];
        bool uplink_mode   = msg->buttons[BUTTON_LS];

        int turn_signal = CmdEnums::TURN_OFF;

        if (turn_right) {
            turn_signal = CmdEnums::TURN_RIGHT;
        } else if (turn_left) {
            turn_signal = CmdEnums::TURN_LEFT;
        } else if (hazard_signal) {
            turn_signal = CmdEnums::TURN_HAZARD;
        }

        auto teleop_msg = std::make_unique<TeleopCommand>();

        teleop_msg->header.stamp = start_time;
        teleop_msg->header.frame_id = "xbox_teleop";
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
    auto node = std::make_shared<XboxTeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}