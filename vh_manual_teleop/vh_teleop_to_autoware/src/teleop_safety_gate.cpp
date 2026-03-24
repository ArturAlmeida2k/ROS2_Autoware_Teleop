#include "rclcpp/rclcpp.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_command.hpp"
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include "std_msgs/msg/int8.hpp"
#include <algorithm>

using Control = autoware_control_msgs::msg::Control;
using GearCommand = autoware_vehicle_msgs::msg::GearCommand;
using TurnIndicatorsCommand = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using HazardLightsCommand = autoware_vehicle_msgs::msg::HazardLightsCommand;

using Int8 = std_msgs::msg::Int8;

class TeleopSafetyGateNode : public rclcpp::Node {
public:
    // Definição dos estados de segurança
    static constexpr int8_t STATE_OK = 0;
    static constexpr int8_t STATE_WARN = 1;
    static constexpr int8_t STATE_ERROR = 2;

    TeleopSafetyGateNode() : Node("teleop_safety_gate_node") {
        // Parâmetros de segurança
        this->declare_parameter<float>("warning_velocity_limit", 2.77f); // ~10 km/h
        this->declare_parameter<float>("emergency_deceleration", -3.0f); // Travagem a 3 m/s²
        
        warning_velocity_limit_ = this->get_parameter("warning_velocity_limit").as_double();
        emergency_deceleration_ = this->get_parameter("emergency_deceleration").as_double();

        // Subscrições (Recebem dados do seu AutowareControllerNode)
        sub_control_cmd_ = this->create_subscription<Control>(
            "/teleop/internal/control_cmd", 10,
            std::bind(&TeleopSafetyGateNode::control_cmd_callback, this, std::placeholders::_1));

        sub_gear_cmd_ = this->create_subscription<GearCommand>(
            "/teleop/internal/gear_cmd", 10,
            std::bind(&TeleopSafetyGateNode::gear_cmd_callback, this, std::placeholders::_1));

        sub_turn_indicators_ = this->create_subscription<TurnIndicatorsCommand>(
            "/teleop/internal/turn_indicators_cmd", 10,
            std::bind(&TeleopSafetyGateNode::turn_indicators_callback, this, std::placeholders::_1));

        sub_hazard_lights_ = this->create_subscription<HazardLightsCommand>(
            "/teleop/internal/hazard_lights_cmd", 10,
            std::bind(&TeleopSafetyGateNode::hazard_lights_callback, this, std::placeholders::_1));    

        // Subscrição do estado de segurança (Virá do futuro nó Monitor)
        sub_safety_state_ = this->create_subscription<Int8>(
            "/teleop/safety_state", 10,
            std::bind(&TeleopSafetyGateNode::safety_state_callback, this, std::placeholders::_1));

        // Publicadores (Enviam dados finais para o Autoware)
        pub_control_cmd_ = this->create_publisher<Control>("/external/selected/control_cmd", rclcpp::QoS(1));
        pub_gear_cmd_ = this->create_publisher<GearCommand>("/external/selected/gear_cmd", 1);
        pub_turn_indicators_ = this->create_publisher<TurnIndicatorsCommand>("/external/selected/turn_indicators_cmd", 1);
        pub_hazard_lights_ = this->create_publisher<HazardLightsCommand>("/external/selected/hazard_lights_cmd", 1);

        RCLCPP_INFO(this->get_logger(), "Teleop Safety Gate Node started.");
    }

private:
    int8_t current_state_ = STATE_ERROR; // Inicia em estado de erro por segurança
    float warning_velocity_limit_;
    float emergency_deceleration_;

    rclcpp::Subscription<Control>::SharedPtr sub_control_cmd_;
    rclcpp::Subscription<GearCommand>::SharedPtr sub_gear_cmd_;
    rclcpp::Subscription<TurnIndicatorsCommand>::SharedPtr sub_turn_indicators_;
    rclcpp::Subscription<Int8>::SharedPtr sub_safety_state_;
    rclcpp::Subscription<HazardLightsCommand>::SharedPtr sub_hazard_lights_;


    rclcpp::Publisher<Control>::SharedPtr pub_control_cmd_;
    rclcpp::Publisher<GearCommand>::SharedPtr pub_gear_cmd_;
    rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr pub_turn_indicators_;
    rclcpp::Publisher<HazardLightsCommand>::SharedPtr pub_hazard_lights_;

    // Atualiza o estado atual com base na mensagem do Monitor
    void safety_state_callback(const Int8::SharedPtr msg) {
        current_state_ = msg->data;
    }

    // Processa os comandos de controlo primário
    void control_cmd_callback(const Control::SharedPtr msg) {
        Control out_msg = *msg; // Copia a mensagem original

        switch (current_state_) {
            case STATE_OK:
                // Passa o comando sem alterações
                break;
            case STATE_WARN:
                // Limita a velocidade máxima (frente e marcha-atrás)
                out_msg.longitudinal.velocity = std::clamp(
                    out_msg.longitudinal.velocity, 
                    -warning_velocity_limit_, 
                    warning_velocity_limit_
                );
                RCLCPP_WARN_ONCE(this->get_logger(), "Warning state: Limiting velocity.");
                break;
            case STATE_ERROR:
            default:
                // Força paragem do veículo
                out_msg.longitudinal.velocity = 0.0;
                out_msg.longitudinal.acceleration = emergency_deceleration_;
                out_msg.lateral.steering_tire_angle = 0.0;
                RCLCPP_ERROR_ONCE(this->get_logger(), "Error state: Stopping vehicle.");
                break;
        }

        // Publica o comando final para o Autoware
        pub_control_cmd_->publish(out_msg);
    }

    // Processa mudanças de caixa
    void gear_cmd_callback(const GearCommand::SharedPtr msg) {
        if (current_state_ == STATE_OK || current_state_ == STATE_WARN) {
            pub_gear_cmd_->publish(*msg);
        }
    }

    // Processa indicadores de mudança de direção (piscas)
    void turn_indicators_callback(const TurnIndicatorsCommand::SharedPtr msg) {
        if (current_state_ == STATE_OK || current_state_ == STATE_WARN) {
            pub_turn_indicators_->publish(*msg);
        }
    }

    void hazard_lights_callback(const HazardLightsCommand::SharedPtr msg){
        if (current_state_ == STATE_OK || current_state_ == STATE_WARN) {
            pub_hazard_lights_->publish(*msg);
        }
        else if (current_state_ == STATE_ERROR) {

            HazardLightsCommand hazard_cmd;
            hazard_cmd.stamp = this->now();
            hazard_cmd.command = HazardLightsCommand::ENABLE;

            pub_hazard_lights_->publish(hazard_cmd);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopSafetyGateNode>());
    rclcpp::shutdown();
    return 0;
}