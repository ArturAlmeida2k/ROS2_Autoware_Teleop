#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <QMetaType>
#include "ros_bridge.hpp"
#include "main_window.hpp"

int main(int argc, char* argv[])
{
    // ROS e Qt partilham argc/argv
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    qRegisterMetaType<TelemetryState>("TelemetryState");

    qRegisterMetaType<CompressedImage::SharedPtr>("CompressedImage::SharedPtr");
    
    auto* bridge = new RosBridge();
    bridge->spin();  // ROS numa thread separada

    MainWindow window(bridge);
    window.show();

    int ret = app.exec();

    bridge->stop();
    rclcpp::shutdown();
    return ret;
}
