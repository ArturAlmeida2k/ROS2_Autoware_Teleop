#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <QMetaType>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp> 
#include "ros_bridge.hpp"
#include "main_window.hpp"

// 2. Adicionar os mesmos 'using' para os nomes baterem certo
using CompressedImage = sensor_msgs::msg::CompressedImage;
using PointCloud2     = sensor_msgs::msg::PointCloud2; 

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    // Registos para o Qt::QueuedConnection
    qRegisterMetaType<TelemetryState>("TelemetryState");
    qRegisterMetaType<CompressedImage::SharedPtr>("CompressedImage::SharedPtr");
    
    // 3. ALTERAR AQUI: Usar exatamente o nome curto que o erro pediu
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