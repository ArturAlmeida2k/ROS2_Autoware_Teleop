#pragma once
#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <teleop_msgs/msg/telemetry_state.hpp>
#include <teleop_msgs/msg/teleop_command.hpp>
#include <teleop_msgs/msg/command_enums.hpp>
#include <teleop_msgs/msg/node_metrics.hpp>
#include <thread>
#include <mutex>
#include <atomic>


using PointCloud2 = sensor_msgs::msg::PointCloud2;
using TelemetryState  = teleop_msgs::msg::TelemetryState;
using TeleopCommand = teleop_msgs::msg::TeleopCommand;
using CmdEnums = teleop_msgs::msg::CommandEnums;
using Metrics = teleop_msgs::msg::NodeMetrics;

class RosBridge : public QObject, public rclcpp::Node {
    Q_OBJECT
public:
    explicit RosBridge(QObject* parent = nullptr);
    void spin();
    void stop();

    int64_t nowNanoseconds() { return this->now().nanoseconds(); }

    bool latestTelemetry(TelemetryState &out, int64_t &rx_time_ns_out);

    uint8_t currentUplinkMode() const { return uplink_mode_.load(std::memory_order_relaxed); }

    double publishTelemetryGuiMetrics(uint32_t id, const builtin_interfaces::msg::Time &origin_stamp, double e2e_command, int64_t rx_time_ns, int64_t display_time_ns);
    void publishFrontCameraMetrics(uint32_t frame_id, double latency_ms);
    void publishFrontCameraNetwork(uint32_t frame_id, double latency_ms);
    void publishFrontCameraDecode(uint32_t frame_id, double latency_ms);
    void publishPointCloudMetrics(uint32_t id, double latency_ms);

    signals:
    void pointCloudReceived(PointCloud2::SharedPtr msg);
private:
    rclcpp::Subscription<TelemetryState>::SharedPtr   sub_telemetry_;
    rclcpp::Subscription<TeleopCommand>::SharedPtr    sub_command_;

    rclcpp::Subscription<PointCloud2>::SharedPtr      sub_pointcloud_;

    rclcpp::Publisher<Metrics>::SharedPtr pub_telemetry_decoder_;
    rclcpp::Publisher<Metrics>::SharedPtr pub_telemetry_gui_;
    rclcpp::Publisher<Metrics>::SharedPtr pub_e2e_telemetry_;
    rclcpp::Publisher<Metrics>::SharedPtr pub_full_latency_;    
    rclcpp::Publisher<Metrics>::SharedPtr pub_front_camera_;
    rclcpp::Publisher<Metrics>::SharedPtr pub_front_camera_network_;
    rclcpp::Publisher<Metrics>::SharedPtr pub_front_camera_decode_;
    rclcpp::Publisher<Metrics>::SharedPtr pub_pointcloud_;

    rclcpp::executors::SingleThreadedExecutor         executor_;
    std::thread                                       spin_thread_;

    std::mutex     telemetry_mutex_;
    TelemetryState latest_telemetry_{};
    int64_t        latest_telemetry_rx_time_ns_ = 0;
    bool           has_telemetry_ = false;

    std::atomic<uint8_t> uplink_mode_{CmdEnums::UPLINK_VIDEO};

    void publish_metric(const rclcpp::Publisher<Metrics>::SharedPtr& pub, uint32_t id, const rclcpp::Time &rx_time, const rclcpp::Time &tx_time);
};