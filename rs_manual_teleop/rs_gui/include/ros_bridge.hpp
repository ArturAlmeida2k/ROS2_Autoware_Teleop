#pragma once
#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <msg_manual_teleop/msg/telemetry_state.hpp>
#include <msg_manual_teleop/msg/node_metrics.hpp>
#include <thread>


using PointCloud2 = sensor_msgs::msg::PointCloud2;
using TelemetryState  = msg_manual_teleop::msg::TelemetryState;
using Metrics = msg_manual_teleop::msg::NodeMetrics;

class RosBridge : public QObject, public rclcpp::Node {
    Q_OBJECT
public:
    explicit RosBridge(QObject* parent = nullptr);
    void spin();
    void stop();

    void publishTelemetryGuiMetrics(uint32_t id, const builtin_interfaces::msg::Time &origin_stamp, double e2e_command, int64_t rx_time_ns, int64_t display_time_ns);
    void publishFrontCameraMetrics(uint32_t frame_id, double latency_ms);
    void publishFrontCameraNetwork(uint32_t frame_id, double latency_ms);
    void publishFrontCameraDecode(uint32_t frame_id, double latency_ms);
    
    signals:
    void telemetryReceived(TelemetryState msg, int64_t receive_time_ns);

    void pointCloudReceived(PointCloud2::SharedPtr msg);
private:
    rclcpp::Subscription<TelemetryState>::SharedPtr   sub_telemetry_;

    rclcpp::Subscription<PointCloud2>::SharedPtr      sub_pointcloud_;

    rclcpp::Publisher<Metrics>::SharedPtr pub_telemetry_decoder_;
    rclcpp::Publisher<Metrics>::SharedPtr pub_telemetry_gui_;
    rclcpp::Publisher<Metrics>::SharedPtr pub_e2e_telemetry_;
    rclcpp::Publisher<Metrics>::SharedPtr pub_full_latency_;    
    rclcpp::Publisher<Metrics>::SharedPtr pub_front_camera_;
    rclcpp::Publisher<Metrics>::SharedPtr pub_front_camera_network_;
    rclcpp::Publisher<Metrics>::SharedPtr pub_front_camera_decode_;

    rclcpp::executors::SingleThreadedExecutor         executor_;
    std::thread                                       spin_thread_;

    void publish_metric(const rclcpp::Publisher<Metrics>::SharedPtr& pub, uint32_t id, const rclcpp::Time &rx_time, const rclcpp::Time &tx_time);
};