#include "ros_bridge.hpp"

RosBridge::RosBridge(QObject* parent)
: QObject(parent), rclcpp::Node("rs_gui_node")
{
    sub_telemetry_ = create_subscription<TelemetryState>(
        "/telemetry/state", 10,
        [this](const TelemetryState::SharedPtr msg) {
            // Qt::QueuedConnection garante thread safety ROS→Qt
            emit telemetryReceived(*msg);
        });

    sub_image_ = create_subscription<CompressedImage>(
        "/camera/front/compressed", 10,
        [this](const CompressedImage::SharedPtr msg) {
            emit imageReceived(msg);
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
