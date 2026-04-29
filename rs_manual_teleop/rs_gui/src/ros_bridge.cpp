#include "ros_bridge.hpp"

RosBridge::RosBridge(QObject* parent)
: QObject(parent), rclcpp::Node("rs_gui_node")
{
    sub_telemetry_ = create_subscription<TelemetryState>(
        "/telemetry/state", 10,
        [this](const TelemetryState::SharedPtr msg) {
            emit telemetryReceived(*msg);
        });

    sub_front_ = create_subscription<CompressedImage>(
        "/camera/front/compressed", rclcpp::SensorDataQoS(), // Correção do QoS
        [this](const CompressedImage::SharedPtr msg) {
            emit imageFrontReceived(msg); // Sinal específico
        });
    
    sub_left_ = create_subscription<CompressedImage>(
        "/camera/left/compressed", rclcpp::SensorDataQoS(),
        [this](const CompressedImage::SharedPtr msg) {
            emit imageLeftReceived(msg);
        });
    
    sub_right_ = create_subscription<CompressedImage>(
        "/camera/right/compressed", rclcpp::SensorDataQoS(),
        [this](const CompressedImage::SharedPtr msg) {
            emit imageRightReceived(msg);
        });
    
    sub_back_ = create_subscription<CompressedImage>(
        "/camera/back/compressed", rclcpp::SensorDataQoS(),
        [this](const CompressedImage::SharedPtr msg) {
            emit imageBackReceived(msg);
        });

    executor_.add_node(get_node_base_interface());
}

void RosBridge::spin()
{
    spin_thread_ = std::thread([this]() {
        executor_.spin();
    });
}

void RosBridge::stop()
{
    executor_.cancel();
    if (spin_thread_.joinable())
        spin_thread_.join();
}
