#pragma once
#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <msg_manual_teleop/msg/telemetry_state.hpp>
#include <thread>

using CompressedImage = sensor_msgs::msg::CompressedImage;
using TelemetryState  = msg_manual_teleop::msg::TelemetryState;

class RosBridge : public QObject, public rclcpp::Node {
    Q_OBJECT
public:
    explicit RosBridge(QObject* parent = nullptr);
    void spin();
    void stop();

signals:
    void telemetryReceived(TelemetryState msg);
    // 1. Declarar um sinal específico para cada câmara
    void imageFrontReceived(CompressedImage::SharedPtr msg);
    void imageLeftReceived(CompressedImage::SharedPtr msg);
    void imageRightReceived(CompressedImage::SharedPtr msg);
    void imageBackReceived(CompressedImage::SharedPtr msg);

private:
    rclcpp::Subscription<TelemetryState>::SharedPtr   sub_telemetry_;
    
    // 2. Declarar as quatro variáveis de subscrição
    rclcpp::Subscription<CompressedImage>::SharedPtr  sub_front_;
    rclcpp::Subscription<CompressedImage>::SharedPtr  sub_left_;
    rclcpp::Subscription<CompressedImage>::SharedPtr  sub_right_;
    rclcpp::Subscription<CompressedImage>::SharedPtr  sub_back_;
    
    rclcpp::executors::SingleThreadedExecutor         executor_;
    std::thread                                       spin_thread_;
};