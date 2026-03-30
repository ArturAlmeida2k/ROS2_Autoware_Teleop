#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include <algorithm>
#include <memory>


#include "vh_telemetry/msg/telemetry_state.hpp"

using Float32 = std_msgs::msg::Float32;
using Bool = std_msgs::msg::Bool;
using Int32 = std_msgs::msg::Int32;
using String = std_msgs::msg::String;
using Telemetry = vh_telemetry::msg::TelemetryState;

class ComandGate: public rclcpp::Node
{
public:
    ComandGate() : Node("comand_gate")
    {
        sub_engage_ = this->create_subscription<Bool>("/teleop/engage_command", 1,
        std::bind(&ComandGate::engage_callback, this, std::placeholders::_1));
        
        sub_gear_ = this->create_subscription<Int32>("/teleop/gear_change", 1,
        std::bind(&ComandGate::gear_callback, this, std::placeholders::_1));

        sub_telemetry_ = this->create_subscription<Telemetry>("/teleop/telemetry", 10,
        [this](const Telemetry::SharedPtr msg){
            current_mode_ = msg->mode;
            current_engage_status_ = msg->engaged;
            current_velocity_ = msg->velocity_kmh;
            current_turn_signal_ = msg->turn_signal;
            current_hazard_signal_ = msg->hazard;
            current_gear_ = msg->gear;

            if (!initialized_) {
                desired_engage_ = (current_mode_ == 3 || current_mode_ == 4);
                initialized_    = true;
                RCLCPP_INFO(get_logger(), "Initialized — engage: %s",
                    desired_engage_ ? "ON" : "OFF");
            }
        });



        pub_engage_ = this->create_publisher<Bool>("/teleop/engage_command_2", 1);

    }
 
private:

    int current_mode_ = 0;
    bool current_engage_status_ = false;
    float current_velocity_ = 0.0f;
    int current_turn_signal_ = 0;
    int current_hazard_signal_ = 0;
    int current_gear_ = 0;

    bool initialized_  = false;
    bool last_engage_ = false;
    bool desired_engage_ = false;
    
    rclcpp::Subscription<Bool>::SharedPtr sub_engage_;
    rclcpp::Subscription<Int32>::SharedPtr sub_gear_;
    rclcpp::Subscription<Telemetry>::SharedPtr sub_telemetry_;

    rclcpp::Publisher<Bool>::SharedPtr pub_engage_;

    void engage_callback(const Bool::SharedPtr msg)
    {
        if(!initialized_) return;

        bool request = msg->data;
        if (request && !last_engage_) {
            if (current_engage_status_ && current_gear_ == 22) {               
                desired_engage_ = !desired_engage_; 
                RCLCPP_INFO(get_logger(), "Engage/Disengage allowed");
            } else {
                const char* gear_str = "UNKNOWN";
                switch (current_gear_) {
                    case 2:   gear_str = "DRIVE";   break;
                    case 20:  gear_str = "REVERSE"; break;
                    case 22:  gear_str = "PARK";    break;
                }
                RCLCPP_WARN(get_logger(),
                    "Engage/Disengage blocked — Autoware control: %s | Gear: %s",
                    current_engage_status_ ? "ON" : "OFF",
                    gear_str);
            }}

        last_engage_ = request;

        Bool out;
        out.data = desired_engage_;
        pub_engage_->publish(out);
    }

    void gear_callback(const Int32::SharedPtr msg)
    {
        if(desired_engage_){
            int request = msg->data;



        }

    }
 
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ComandGate>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}