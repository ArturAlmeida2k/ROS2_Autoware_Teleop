#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include <algorithm>
#include <memory>

using Joy = sensor_msgs::msg::Joy;
using Float32 = std_msgs::msg::Float32;
using Bool = std_msgs::msg::Bool;
using Int32 = std_msgs::msg::Int32;

class G923TeleopNode : public rclcpp::Node
{
public:
    G923TeleopNode() : Node("teleop_g923_node")
    {
        // Pub to the Autoware Controller Node
        pub_vlc_ = this->create_publisher<Float32>("/teleop/target_velocity", 10);
        pub_brake_factor_ = this->create_publisher<Float32>("/teleop/brake_factor", 10);
        pub_steering_ = this->create_publisher<Float32>("/teleop/target_steering_angle", 10);
        pub_engage_ = this->create_publisher<Bool>("/teleop/engage_command", 1);
        pub_gear_ = this->create_publisher<Int32>("/teleop/gear_change", 1);

        // Sub to the Joystick
        sub_joy_ = this->create_subscription<Joy>(
            "/joy", 10, 
            std::bind(&G923TeleopNode::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Nó G923 Teleop iniciado. Mapeamento de controlo ativo.");
    }

private:
    // --- Mapping of the Axis and Buttons (Logitech G923) ---

    // Axis (vals from -1.0 a 1.0)
    const int AXIS_STEERING = 0;   // Sterring Wheel, 0 repose, -1 right, 1 left
    const int AXIS_THROTTLE = 2;   // Accelaration, -1 repose
    const int AXIS_BRAKE = 3;      // Brake -1 em repose 
    
    const int BUTTON_ENGAGE_1 = 6; // Engage Button 1 -> R2
    const int BUTTON_ENGAGE_2 = 7; // Engage Button 2 -> L2

    const int GEAR_DRIVE = 4; // Drive_Button -> Right padle
    const int GEAR_REVERSE = 5; // Reverse Button -> Left padle
    
    // --- Constants ---
    const float MAX_VLC = 10.0f; // Maximum Velocity in m/s, 5m/s -> 18Km/h
    const float MAX_STEERING_RAD = 0.5f; // Maximum steering angle (~28.6 graus)
    
    // --- ROS 2 ---
    rclcpp::Publisher<Float32>::SharedPtr pub_vlc_;
    rclcpp::Publisher<Float32>::SharedPtr pub_brake_factor_; 
    rclcpp::Publisher<Float32>::SharedPtr pub_steering_;
    rclcpp::Publisher<Bool>::SharedPtr pub_engage_;
    rclcpp::Publisher<Int32>::SharedPtr pub_gear_;
    rclcpp::Subscription<Joy>::SharedPtr sub_joy_;

  
    // Variables to publish
    // --- For Engage ---
    bool last_engage_state = false; 
    bool current_engage_state = false; 
    // --- For Velocity and Break ---
    float last_vlc_target_ = 0.0f;
    float last_brake_factor_ = 0.0f;
    // --- For Steering ---
    float last_steering_target_ = 0.0f; 
    // --- For Gear ---
    int current_gear_ = 2;

    // --- Engage Publisher function ---
    void publish_engage(bool state)
    {
        auto msg = std::make_unique<Bool>();
        msg->data = state;
        pub_engage_->publish(std::move(msg));
        RCLCPP_WARN(this->get_logger(), "Engage State Alterado: %s", state ? "TRUE" : "FALSE");
    
    }

    // --- Velocity Publisher function ---
    void publish_vlc(float vlc)
    { 
        if (vlc != last_vlc_target_) {
            auto msg = std::make_unique<Float32>();
            msg->data = vlc;
            pub_vlc_->publish(std::move(msg));
            last_vlc_target_ = vlc;
        }
    }
    // --- Break Publisher function ---
    void publish_brake_factor(float factor) 
    {
        if (factor != last_brake_factor_) {
            auto msg = std::make_unique<Float32>();
            msg->data = factor;
            pub_brake_factor_->publish(std::move(msg));
            last_brake_factor_ = factor;
        }
    }

    // --- Steering Publisher function ---
    void publish_steering(float angle)
    {
        if (angle != last_steering_target_) {
            auto msg = std::make_unique<Float32>();
            msg->data = angle;
            pub_steering_->publish(std::move(msg));
            last_steering_target_ = angle;
        }
    }

    // -- Gear Publisher function --
    void publish_gear(int gear)
    {
        auto msg = std::make_unique<Int32>();
        msg->data = gear;
        pub_gear_->publish(std::move(msg));    
    }

    void joy_callback(const Joy::SharedPtr msg)
    {
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
        
        // Logic to prevent multiple changes in the engage state
        if (change_engage_state && last_engage_state == current_engage_state){
            current_engage_state = !current_engage_state;
            publish_engage(current_engage_state);
        }
        else if(!change_engage_state && last_engage_state != current_engage_state){
            last_engage_state = current_engage_state;
        }
        

        // --- 2. VELOCITY CONTROL (ACCEL & BRAKE) ---

        float throttle_input = msg->axes[AXIS_THROTTLE];
        float brake_input = msg->axes[AXIS_BRAKE];

        // Change from [-1.0 (Repose) to 1.0 (Fully Pressed)] to [0.0 to 1.0]
        double normalized_throttle = (throttle_input + 1.0) / 2.0;
        double normalized_brake = (brake_input + 1.0) / 2.0;
        
        float target_vlc = 0.0f;
        
        // If the brake is pressed the target velocity is 0.0
        if (normalized_brake > 0.01) {
            target_vlc = 0.0f;
        } else {
            // Otherwise the target velocity is controlled by the accelarator pedal.
            target_vlc = normalized_throttle * MAX_VLC;
        }
        
        target_vlc = std::clamp(target_vlc, 0.0f, MAX_VLC);
        publish_vlc(target_vlc);
        publish_brake_factor(normalized_brake); 

        // --- 3. STEERING CONTROL ---
        
        float steering_input = msg->axes[AXIS_STEERING];
        float target_steering_angle = steering_input * MAX_STEERING_RAD;
        publish_steering(target_steering_angle);

        // --- 4. GEAR CONTROL ---
        bool drive_button = msg->buttons[GEAR_DRIVE];
        bool reverse_button = msg->buttons[GEAR_REVERSE];

        if (drive_button && !reverse_button && current_gear_ != 1){
            publish_gear(1);
            current_gear_ = 1;
        }
        else if (!drive_button && reverse_button && current_gear_ != 0){
            publish_gear(0);
            current_gear_ = 0;
        }
        else if (drive_button && reverse_button) {
            RCLCPP_WARN(this->get_logger(), "Don't press both padles at same time");
        }

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