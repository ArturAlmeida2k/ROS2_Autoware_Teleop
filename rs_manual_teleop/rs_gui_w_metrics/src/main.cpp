#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <QMetaType>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp> 
#include "ros_bridge.hpp"
#include "main_window.hpp"


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    qRegisterMetaType<int64_t>("int64_t");

    // Registos para o Qt::QueuedConnection
    qRegisterMetaType<TelemetryState>("TelemetryState");
    qRegisterMetaType<CompressedImage::SharedPtr>("CompressedImage::SharedPtr");
    
    qRegisterMetaType<PointCloud2::SharedPtr>("PointCloud2::SharedPtr"); 

    auto* bridge = new RosBridge();
    bridge->spin();  

    MainWindow window(bridge);
    window.show();

    int ret = app.exec();

    bridge->stop();
    rclcpp::shutdown();
    return ret;
}