#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <algorithm>
#include <memory>

#include "msg_manual_teleop/msg/teleop_command.hpp"

using Joy = sensor_msgs::msg::Joy;
using TeleopCommand = msg_manual_teleop::msg::TeleopCommand;

// ============================================================
//  Mapeamento do Comando Xbox (via joy_node / driver xpad)
// ============================================================
//
//  EIXOS (axes[])
//  ─────────────────────────────────────────────────────────
//  0  │ Left Stick  X    │ -1.0 = esquerda, +1.0 = direita
//  1  │ Left Stick  Y    │ +1.0 = cima,     -1.0 = baixo
//  2  │ LT (Travão)      │  0.0 = solto,    +1.0 = fundo
//  3  │ Right Stick X    │ -1.0 = esquerda, +1.0 = direita
//  4  │ Right Stick Y    │ +1.0 = cima,     -1.0 = baixo
//  5  │ RT (Acelerador)  │  0.0 = solto,    +1.0 = fundo
//  6  │ D-Pad X          │ -1.0 = esq,      +1.0 = dir
//  7  │ D-Pad Y          │ +1.0 = cima,     -1.0 = baixo
//
//  BOTÕES (buttons[])
//  ─────────────────────────────────────────────────────────
//  0  │ A      │ Mudar para Drive
//  1  │ B      │ Mudar para Reverse
//  2  │ X      │ Modificador de mudança (equivalente à embraiagem)
//  3  │ Y      │ Luzes de perigo (hazard)
//  4  │ LB     │ Pisca esquerdo
//  5  │ RB     │ Pisca direito
//  6  │ Back   │ Engage (parte 1 — pressionar Back + Start em simultâneo)
//  7  │ Start  │ Engage (parte 2 — pressionar Back + Start em simultâneo)
//  8  │ Guide  │ (não utilizado)
//  9  │ LS     │ (não utilizado)
// 10  │ RS     │ (não utilizado)
//
//  MUDANÇAS DE MARCHA
//  ─────────────────────────────────────────────────────────
//  Manter X pressionado (modificador de embraiagem), depois:
//    D-Pad Cima  → Sair do Parking (entrar em Drive)
//    D-Pad Baixo → Entrar em Parking
//    A           → Drive
//    B           → Reverse
// ============================================================

class XboxTeleopNode : public rclcpp::Node
{
public:
    XboxTeleopNode() : Node("xbox_teleop_node")
    {
        pub_filtered_command_ = this->create_publisher<TeleopCommand>("/teleop/filtered_command", 10);

        sub_joy_ = this->create_subscription<Joy>(
            "/joy", 10,
            std::bind(&XboxTeleopNode::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "Nó Xbox Teleop iniciado. Publicando em /teleop/filtered_command.");
    }

private:
    // --- Eixos ---
    const int AXIS_STEERING  = 0;  // Left Stick X
    const int AXIS_BRAKE     = 2;  // LT  — 0.0=solto, 1.0=fundo
    const int AXIS_THROTTLE  = 5;  // RT  — 0.0=solto, 1.0=fundo
    const int AXIS_DPAD_X    = 6;  // D-Pad horizontal
    const int AXIS_DPAD_Y    = 7;  // D-Pad vertical

    // --- Botões ---
    const int BUTTON_A       = 0;  // Drive
    const int BUTTON_B       = 1;  // Reverse
    const int BUTTON_X       = 2;  // Modificador de marcha (embraiagem)
    const int BUTTON_Y       = 3;  // Hazard
    const int BUTTON_LB      = 4;  // Pisca esquerdo
    const int BUTTON_RB      = 5;  // Pisca direito
    const int BUTTON_BACK    = 6;  // Engage parte 1
    const int BUTTON_START   = 7;  // Engage parte 2

    // --- Constantes ---
    const float MAX_VLC           = 10.0f;  // Velocidade máxima em m/s
    const float MAX_STEERING_RAD  =  0.5f;  // Ângulo máximo de direção (~28.6°)

    // --- ROS 2 ---
    rclcpp::Publisher<TeleopCommand>::SharedPtr pub_filtered_command_;
    rclcpp::Subscription<Joy>::SharedPtr sub_joy_;

    void joy_callback(const Joy::SharedPtr msg)
    {
        // O comando Xbox necessita de pelo menos 8 eixos e 8 botões
        if (msg->axes.size() < 8 || msg->buttons.size() < 8) {
            RCLCPP_WARN_ONCE(this->get_logger(),
                "Mensagem JOY incompleta. Esperados >= 8 eixos e >= 8 botões.");
            return;
        }

        // ── 1. ENGAGE ──────────────────────────────────────────────────────────
        // Pressionar Back + Start em simultâneo para alternar o estado de engage
        bool engage_button_1    = msg->buttons[BUTTON_BACK];
        bool engage_button_2    = msg->buttons[BUTTON_START];
        bool change_engage_state = engage_button_1 && engage_button_2;

        // ── 2. VELOCIDADE (Acelerador e Travão) ────────────────────────────────
        // Os gatilhos do Xbox vão de 0.0 (solto) a 1.0 (fundo pressionado),
        // ao contrário dos pedais G923 que vão de -1.0 a 1.0.
        // Normalização: valor já está em [0.0 – 1.0], não é necessário converter.
        double normalized_throttle = static_cast<double>(msg->axes[AXIS_THROTTLE]);
        double normalized_brake    = static_cast<double>(msg->axes[AXIS_BRAKE]);

        float target_vlc = 0.0f;

        if (normalized_brake > 0.05) {
            // Travão pressionado → velocidade alvo = 0
            target_vlc = 0.0f;
        } else {
            target_vlc = static_cast<float>(normalized_throttle) * MAX_VLC;
        }

        target_vlc = std::clamp(target_vlc, 0.0f, MAX_VLC);

        // ── 3. DIREÇÃO ─────────────────────────────────────────────────────────
        // Left Stick X: -1.0 = esquerda, +1.0 = direita.
        // Negamos para manter a convenção do G923 (positivo = virar à esquerda).
        float steering_input        = msg->axes[AXIS_STEERING];
        float target_steering_angle = -steering_input * MAX_STEERING_RAD;

        // ── 4. MUDANÇAS DE MARCHA ──────────────────────────────────────────────
        // O botão X funciona como modificador (equivalente à embraiagem).
        // Com X pressionado, usar A/B ou D-Pad para selecionar a marcha.
        //
        //  Gear: 0 = sem alteração, 1 = Parking,
        //        2 = Drive,         3 = Reverse
        bool gear_modifier  = msg->buttons[BUTTON_X];
        bool drive_button   = msg->buttons[BUTTON_A];
        bool reverse_button = msg->buttons[BUTTON_B];
        int  dpad_y         = static_cast<int>(msg->axes[AXIS_DPAD_Y]);

        int new_gear = 0;

        if (gear_modifier) {
            if (dpad_y == 1) {
                // D-Pad Cima + X → sair de Parking (entrar em Drive)
                new_gear = 2;
            } else if (dpad_y == -1) {
                // D-Pad Baixo + X → entrar em Parking
                new_gear = 1;
            } else if (drive_button) {
                // A + X → Drive
                new_gear = 2;
            } else if (reverse_button) {
                // B + X → Reverse
                new_gear = 3;
            }
        }

        // ── 5. PISCAS E LUZES DE PERIGO ────────────────────────────────────────
        // Right(1), Left(2), Hazard(3)
        bool turn_right   = msg->buttons[BUTTON_RB];
        bool turn_left    = msg->buttons[BUTTON_LB];
        bool hazard_signal = msg->buttons[BUTTON_Y];

        int turn_signal = 0;

        if (turn_right) {
            turn_signal = 1;
        } else if (turn_left) {
            turn_signal = 2;
        } else if (hazard_signal) {
            turn_signal = 3;
        }

        // ── 6. PUBLICAÇÃO ──────────────────────────────────────────────────────
        auto teleop_msg = std::make_unique<TeleopCommand>();

        teleop_msg->header.stamp    = this->now();
        teleop_msg->header.frame_id = "xbox_teleop";

        teleop_msg->id = 0;

        teleop_msg->target_velocity      = target_vlc;
        teleop_msg->brake_factor         = static_cast<float>(normalized_brake);
        teleop_msg->target_steering_angle = target_steering_angle;
        teleop_msg->engage_command       = change_engage_state;
        teleop_msg->gear                 = new_gear;
        teleop_msg->turn_signal          = turn_signal;

        pub_filtered_command_->publish(std::move(teleop_msg));
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