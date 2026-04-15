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

class XboxTeleopNode : public rclcpp::Node
{
public:
    XboxTeleopNode() : Node("xbox_teleop_node")
    {
        // Pub to the Autoware Controller Node
        pub_vlc_ = this->create_publisher<Float32>("/teleop/target_velocity", 10);
        pub_brake_factor_ = this->create_publisher<Float32>("/teleop/brake_factor", 10);
        pub_steering_ = this->create_publisher<Float32>("/teleop/target_steering_angle", 10);
        pub_engage_ = this->create_publisher<Bool>("/teleop/engage_command", 1);
        pub_gear_ = this->create_publisher<Int32>("/teleop/gear_change", 1);
        pub_turn_signal_ = this->create_publisher<Int32>("/teleop/turn_signal", 1);

        // Sub to the Joystick
        sub_joy_ = this->create_subscription<Joy>(
            "/joy", 10,
            std::bind(&XboxTeleopNode::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Nó Xbox Teleop iniciado. Mapeamento de controlo ativo.");
    }

private:
    // --- Mapping of the Axis and Buttons (Xbox Controller - joy_linux driver) ---
    //
    //  AXES:
    //    0 = Left Stick X     (left: +1.0, right: -1.0)
    //    1 = Left Stick Y     (up: +1.0, down: -1.0)
    //    2 = Left Trigger LT  (-1.0 = released, +1.0 = fully pressed)
    //    3 = Right Stick X
    //    4 = Right Stick Y
    //    5 = Right Trigger RT (-1.0 = released, +1.0 = fully pressed)
    //    6 = D-Pad X          (-1.0 = left, +1.0 = right)
    //    7 = D-Pad Y          (-1.0 = down, +1.0 = up)
    //
    //  BUTTONS:
    //    0  = A
    //    1  = B
    //    2  = X
    //    3  = Y
    //    4  = LB
    //    5  = RB
    //    6  = Back / View
    //    7  = Start / Menu
    //    8  = Xbox / Guide
    //    9  = Left Stick Click  (L3)
    //   10  = Right Stick Click (R3)

    // Axes
    const int AXIS_STEERING  = 0;  // Left Stick X: +1 = left, -1 = right
    const int AXIS_THROTTLE  = 5;  // Right Trigger RT: -1 = released, +1 = fully pressed
    const int AXIS_BRAKE     = 2;  // Left Trigger LT:  -1 = released, +1 = fully pressed

    // D-Pad (used for parking)
    const int AXIS_DPAD_Y    = 7;  // +1 = up (exit parking), -1 = down (enter parking)

    // Buttons
    const int BUTTON_ENGAGE_1        = 4;  // LB
    const int BUTTON_ENGAGE_2        = 5;  // RB
    const int BUTTON_GEAR_DRIVE      = 1;  // B  → Drive
    const int BUTTON_GEAR_REVERSE    = 2;  // X  → Reverse
    const int BUTTON_TURN_RIGHT      = 10; // R3 → Right turn signal
    const int BUTTON_TURN_LEFT       = 9;  // L3 → Left turn signal
    const int BUTTON_HAZARD          = 3;  // Y  → Hazard lights

    // --- Constants ---
    const float MAX_VLC          = 10.0f;  // Maximum velocity in m/s
    const float MAX_STEERING_RAD = 0.5f;   // Maximum steering angle (~28.6 degrees)

    // --- ROS 2 ---
    rclcpp::Publisher<Float32>::SharedPtr pub_vlc_;
    rclcpp::Publisher<Float32>::SharedPtr pub_brake_factor_;
    rclcpp::Publisher<Float32>::SharedPtr pub_steering_;
    rclcpp::Publisher<Bool>::SharedPtr pub_engage_;
    rclcpp::Publisher<Int32>::SharedPtr pub_gear_;
    rclcpp::Publisher<Int32>::SharedPtr pub_turn_signal_;
    rclcpp::Subscription<Joy>::SharedPtr sub_joy_;

    // --- State variables ---
    // Engage
    bool last_engage_state_   = false;
    bool current_engage_state_ = false;
    // Gear
    int current_gear_ = 0; // 0 = Parked, 1 = Drive, 2 = Reverse
    // Turn signal
    int  turn_signal_       = 1; // 1 = Off, 2 = Left, 3 = Right, 4 = Hazard
    bool last_turn_right_   = false;
    bool last_turn_left_    = false;
    bool last_hazard_signal_ = false;

    // --- Publisher helpers ---
    void publish_engage(bool state)
    {
        auto msg = std::make_unique<Bool>();
        msg->data = state;
        pub_engage_->publish(std::move(msg));
    }

    void publish_vlc(float vlc)
    {
        auto msg = std::make_unique<Float32>();
        msg->data = vlc;
        pub_vlc_->publish(std::move(msg));
    }

    void publish_brake_factor(float factor)
    {
        auto msg = std::make_unique<Float32>();
        msg->data = factor;
        pub_brake_factor_->publish(std::move(msg));
    }

    void publish_steering(float angle)
    {
        auto msg = std::make_unique<Float32>();
        msg->data = angle;
        pub_steering_->publish(std::move(msg));
    }

    void publish_gear(int gear)
    {
        auto msg = std::make_unique<Int32>();
        msg->data = gear;
        pub_gear_->publish(std::move(msg));
    }

    void publish_turn_signal(int turn_signal)
    {
        auto msg = std::make_unique<Int32>();
        msg->data = turn_signal;
        pub_turn_signal_->publish(std::move(msg));
    }

    void joy_callback(const Joy::SharedPtr msg)
    {
        // Xbox controller: 8 axes, 11 buttons (joy_linux driver)
        if (msg->axes.size() < 8 || msg->buttons.size() < 11) {
            RCLCPP_WARN_ONCE(this->get_logger(), "JOY message incomplete.");
            return;
        }

        // --- 1. ENGAGE (LB + RB simultaneously) ---
        bool engage_button_1 = msg->buttons[BUTTON_ENGAGE_1];
        bool engage_button_2 = msg->buttons[BUTTON_ENGAGE_2];
        bool change_engage   = engage_button_1 && engage_button_2;

        if (change_engage && !last_engage_state_) {
            current_engage_state_ = !current_engage_state_;
            RCLCPP_INFO(this->get_logger(), "Engage State Alterado: %s",
                        current_engage_state_ ? "TRUE" : "FALSE");
        }
        last_engage_state_ = change_engage;

        // --- 2. VELOCITY CONTROL (RT = throttle, LT = brake) ---
        // Triggers: -1.0 = released, +1.0 = fully pressed  →  normalise to [0.0, 1.0]
        float rt_raw = msg->axes[AXIS_THROTTLE];
        float lt_raw = msg->axes[AXIS_BRAKE];

        double normalized_throttle = (rt_raw + 1.0) / 2.0;
        double normalized_brake    = (lt_raw + 1.0) / 2.0;

        float target_vlc = 0.0f;
        if (normalized_brake > 0.05) {
            target_vlc = 0.0f;
        } else {
            target_vlc = static_cast<float>(normalized_throttle * MAX_VLC);
        }
        target_vlc = std::clamp(target_vlc, 0.0f, MAX_VLC);

        // --- 3. STEERING (Left Stick X) ---
        float steering_input       = msg->axes[AXIS_STEERING];
        float target_steering_angle = steering_input * MAX_STEERING_RAD;

        // --- 4. GEAR CONTROL ---
        // D-Pad Y: +1 = up → exit Parking to Drive; -1 = down → enter Parking
        // B  = Drive, X = Reverse  (only effective when engaged)
        bool drive_button   = msg->buttons[BUTTON_GEAR_DRIVE];
        bool reverse_button = msg->buttons[BUTTON_GEAR_REVERSE];
        int  dpad_y         = static_cast<int>(msg->axes[AXIS_DPAD_Y]);

        if (current_engage_state_) {
            // Enter Parking from any gear
            if (dpad_y == -1) {
                current_gear_ = 0;
            }
            // Exit Parking to Drive
            else if (current_gear_ == 0 && dpad_y == 1) {
                current_gear_ = 1;
            }
            // Switch Drive / Reverse (only when not Parked)
            else if (current_gear_ != 0) {
                if (drive_button && !reverse_button) {
                    current_gear_ = 1;
                } else if (!drive_button && reverse_button) {
                    current_gear_ = 2;
                }
            }
        }

        // --- 5. TURN SIGNALS ---
        bool turn_right    = msg->buttons[BUTTON_TURN_RIGHT];
        bool turn_left     = msg->buttons[BUTTON_TURN_LEFT];
        bool hazard_signal = msg->buttons[BUTTON_HAZARD];

        if (turn_right && !last_turn_right_) {
            turn_signal_ = (turn_signal_ == 3) ? 1 : 3; // toggle RIGHT
        }
        if (turn_left && !last_turn_left_) {
            turn_signal_ = (turn_signal_ == 2) ? 1 : 2; // toggle LEFT
        }
        if (hazard_signal && !last_hazard_signal_) {
            turn_signal_ = (turn_signal_ == 4) ? 1 : 4; // toggle HAZARD
        }
        last_turn_right_    = turn_right;
        last_turn_left_     = turn_left;
        last_hazard_signal_ = hazard_signal;

        // --- 6. PUBLISH ---
        publish_engage(current_engage_state_);
        publish_vlc(target_vlc);
        publish_brake_factor(static_cast<float>(normalized_brake));
        publish_steering(target_steering_angle);
        publish_gear(current_gear_);
        publish_turn_signal(turn_signal_);
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