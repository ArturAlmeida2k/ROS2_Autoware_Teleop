#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <algorithm>
#include <memory>

#include "msg_manual_teleop/msg/teleop_command.hpp"

using Joy = sensor_msgs::msg::Joy;
using TeleopCommand = msg_manual_teleop::msg::TeleopCommand;

class G923TeleopNode : public rclcpp::Node
{
public:
    G923TeleopNode() : Node("g923_teleop_node")
    {
        // Pub to the Autoware Controller Node
        pub_filtered_command_ = this->create_publisher<TeleopCommand>("/teleop/filtered_command", 10);

        // Sub to the Joystick
        sub_joy_ = this->create_subscription<Joy>(
            "/joy_throttled", 10, 
            std::bind(&G923TeleopNode::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Nó G923 Teleop iniciado. Mapeamento de controlo ativo. Publicando em /teleop/filtered_command.");
    }

private:
    // --- Mapping of the Axis and Buttons (Logitech G923) ---

    // Axis (vals from -1.0 a 1.0)
    const int AXIS_STEERING = 0;   // Sterring Wheel, 0 repose, -1 right, 1 left
    const int AXIS_CLUTCH = 1;     // Clutch, -1 repose
    const int AXIS_THROTTLE = 2;   // Accelaration, -1 repose
    const int AXIS_BRAKE = 3;      // Brake, -1 repose 
    
    const int BUTTON_ENGAGE_1 = 6; // Engage Button 1 -> R2
    const int BUTTON_ENGAGE_2 = 7; // Engage Button 2 -> L2
    const int BUTTON_TURN_SIGNAL_RIGHT = 10; // Turn Signal Right -> R3
    const int BUTTON_TURN_SIGNAL_LEFT = 11; // Turn Signal Left -> L3
    const int BUTTON_HAZARD_SIGNAL = 23; // Turn Hazard Lights -> "Enter" Button

    const int GEAR_DRIVE = 4; // Drive_Button -> Right padle
    const int GEAR_REVERSE = 5; // Reverse Button -> Left padle
    const int GEAR_PARKED_AXIS = 5; // Parking Button 2 -> D-PAD (UP/DOWN)
    
    // --- Constants ---
    const float MAX_VLC = 10.0f; // Maximum Velocity in m/s, 5m/s -> 18Km/h
    const float MAX_STEERING_RAD = 0.5f; // Maximum steering angle (~28.6 graus)

    // --- Extra variables ---

    u_int16_t seq_num_ = 1;
    
    // --- ROS 2 ---
    rclcpp::Publisher<TeleopCommand>::SharedPtr pub_filtered_command_;
    rclcpp::Subscription<Joy>::SharedPtr sub_joy_;
  
    void joy_callback(const Joy::SharedPtr msg)
    {

        // Get the time as soon has possible for metrics
        auto start_time = this->now();

        // Check if the joy mensage is complete (it should have 4 axis and 8 buttons)
        if (msg->axes.size() < 6 || msg->buttons.size() < 24) {
            RCLCPP_WARN_ONCE(this->get_logger(), "JOY message incomplete.");
            return;
        }
        
        // --- 1. ENGAGE OF THE CAR ---
    
        // To engage both the L2 and R2 need to be pressed at the same time 
        bool engage_button_1 = msg->buttons[BUTTON_ENGAGE_1];
        bool engage_button_2 = msg->buttons[BUTTON_ENGAGE_2];
        
        bool change_engage_state = engage_button_1 && engage_button_2;
        
        // --- 2. VELOCITY CONTROL (ACCEL & BRAKE) ---

        float throttle_input = msg->axes[AXIS_THROTTLE];
        float brake_input = msg->axes[AXIS_BRAKE];

        // Change from [-1.0 (Repose) to 1.0 (Fully Pressed)] to [0.0 to 1.0]
        double normalized_throttle = (throttle_input + 1.0) / 2.0;
        double normalized_brake = (brake_input + 1.0) / 2.0;
        
        float target_vlc = 0.0f;
        
        // If the brake is pressed the target velocity is 0.0
        if (normalized_brake > 0.05) {
            target_vlc = 0.0f;
        } 
        else {
            // Otherwise the target velocity is controlled by the accelarator pedal.
            target_vlc = normalized_throttle * MAX_VLC;
        }
        
        target_vlc = std::clamp(target_vlc, 0.0f, MAX_VLC);

        // --- 3. STEERING CONTROL ---
        
        float steering_input = msg->axes[AXIS_STEERING];
        float target_steering_angle = steering_input * MAX_STEERING_RAD;

        // --- 4. GEAR CONTROL ---
        bool drive_button = msg->buttons[GEAR_DRIVE];
        bool reverse_button = msg->buttons[GEAR_REVERSE];
        int parking_axes = msg->axes[GEAR_PARKED_AXIS];
        float clutch = msg->axes[AXIS_CLUTCH];

        // Change from [-1.0 (Repose) to 1.0 (Fully Pressed)] to [0.0 to 1.0]
        double normalized_clutch = (clutch + 1.0) / 2.0;

        int new_gear = 0;
        // Enter Parking(1), Leave Parking(2), Drive(2), Reverse(3)

        if (normalized_clutch >= 0.9){
            // Lógica para sair de Parking (0) para Drive (1)
            if (parking_axes == 1) {
                new_gear = 2;
            }
            // Lógica para entrar em Parking (0)
            else if (parking_axes == -1) {
                new_gear = 1;
            }
            // Troca entre Drive (1) e Reverse (2) - apenas se não estiver em Parking
           
            else if (drive_button) {
                    new_gear = 2;
            }
            
            else if (reverse_button) {
                    new_gear = 3;
            }
        }

        // --- 5. TURN SIGNAL ----

        bool turn_right = msg->buttons[BUTTON_TURN_SIGNAL_RIGHT];
        bool turn_left = msg->buttons[BUTTON_TURN_SIGNAL_LEFT];
        bool hazard_signal = msg->buttons[BUTTON_HAZARD_SIGNAL];

        int turn_signal = 0;

        // Right(1), Left(2), Hazard(3)
        if (turn_right) {
            turn_signal = 1;

        }
        else if (turn_left){
            turn_signal = 2;
        }
        else if (hazard_signal){
            turn_signal = 3;
        }

        // --- 6. PUBLISHING ---  
        auto teleop_msg = std::make_unique<TeleopCommand>();

        teleop_msg->header.stamp = start_time;
        teleop_msg->header.frame_id = "g923_teleop";
        teleop_msg->origin_stamp = start_time;
        teleop_msg->id = seq_num_++;
        teleop_msg->target_velocity = target_vlc;
        teleop_msg->brake_factor = static_cast<float>(normalized_brake);
        teleop_msg->target_steering_angle = target_steering_angle;
        teleop_msg->engage_command = change_engage_state;
        teleop_msg->gear = new_gear;
        teleop_msg->turn_signal = turn_signal;

        pub_filtered_command_->publish(std::move(teleop_msg));
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