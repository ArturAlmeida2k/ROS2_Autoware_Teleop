#pragma once
#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <msg_manual_teleop/msg/telemetry_state.hpp>
#include <thread>

using CompressedImage = sensor_msgs::msg::CompressedImage;
using TelemetryState  = msg_manual_teleop::msg::TelemetryState;

// RosBridge vive na thread ROS e emite sinais Qt thread-safe
class RosBridge : public QObject, public rclcpp::Node {
    Q_OBJECT
public:
    explicit RosBridge(QObject* parent = nullptr);
    void spin();
    void stop();

signals:
    void telemetryReceived(TelemetryState msg);
    void imageReceived(CompressedImage::SharedPtr msg);

private:
    rclcpp::Subscription<TelemetryState>::SharedPtr   sub_telemetry_;
    rclcpp::Subscription<CompressedImage>::SharedPtr  sub_image_;
    rclcpp::executors::SingleThreadedExecutor         executor_;
    std::thread                                        spin_thread_;
};
