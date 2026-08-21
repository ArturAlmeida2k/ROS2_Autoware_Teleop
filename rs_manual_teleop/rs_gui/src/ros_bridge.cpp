#include "ros_bridge.hpp"

RosBridge::RosBridge(QObject* parent)
: QObject(parent), rclcpp::Node("rs_gui_node")
{
    rclcpp::QoS metrics_qos(10);       
    metrics_qos.best_effort();    
    metrics_qos.durability_volatile();

    pub_telemetry_decoder_ = this->create_publisher<Metrics>("/metrics/telemetry_decoder", metrics_qos);
    pub_telemetry_gui_     = this->create_publisher<Metrics>("/metrics/telemetry_gui", metrics_qos);
    pub_e2e_telemetry_     = this->create_publisher<Metrics>("/metrics/e2e_telemetry_latency", metrics_qos);
    pub_full_latency_      = this->create_publisher<Metrics>("/metrics/full_latency", metrics_qos);
    pub_front_camera_      = this->create_publisher<Metrics>("/metrics/front_camera", metrics_qos);
    pub_front_camera_network_ = this->create_publisher<Metrics>("/metrics/front_camera_network", metrics_qos);
    pub_front_camera_decode_  = this->create_publisher<Metrics>("/metrics/front_camera_decode", metrics_qos);
    
    sub_telemetry_ = create_subscription<TelemetryState>(
        "/telemetry/state", 10,
        [this](const TelemetryState::SharedPtr msg) {
            rclcpp::Time tempo_rececao = this->now();
            
            // 1. Publica DIRETAMENTE para o tópico de rede
            publish_metric(pub_telemetry_decoder_, msg->id, tempo_rececao, rclcpp::Time(msg->header.stamp));

            emit telemetryReceived(*msg, tempo_rececao.nanoseconds());
        });
    
    sub_pointcloud_ = create_subscription<PointCloud2>(
        "/teleop/pointcloud",
        rclcpp::SensorDataQoS(),
        [this](const PointCloud2::SharedPtr msg) {
            emit pointCloudReceived(msg);
        });

    executor_.add_node(get_node_base_interface());
}

void RosBridge::publish_metric(const rclcpp::Publisher<Metrics>::SharedPtr& pub, uint32_t id, const rclcpp::Time &rx_time, const rclcpp::Time &tx_time)
{
    auto msg = std::make_unique<Metrics>();
    msg->id = id;
    msg->tx = tx_time; 
    msg->rx = rx_time;
    
    rclcpp::Duration latency = rx_time - tx_time;

    msg->latency_ms = latency.seconds() * 1000.0;
    
    pub->publish(std::move(msg));
}

// Tópicos 2 e 3 (Telemetria GUI e End-to-End)
double RosBridge::publishTelemetryGuiMetrics(uint32_t id, const builtin_interfaces::msg::Time &origin_stamp, double e2e_command_ms, int64_t rx_time_ns, int64_t display_time_ns)
{
    rclcpp::Time rx_time(rx_time_ns, RCL_ROS_TIME);
    rclcpp::Time display_time(display_time_ns, RCL_ROS_TIME);
    rclcpp::Time origin_time(origin_stamp, RCL_ROS_TIME);

    // Tópico 2: Latência apenas da interface (Display - Rx)
    publish_metric(pub_telemetry_gui_, id, display_time, rx_time);

    // Tópico 3: E2E Telemetry = (Display - Origin)
    // Usamos a função genérica para calcular automaticamente e fazer o publish
    publish_metric(pub_e2e_telemetry_, id, display_time, origin_time);

    // Tópico 4: Full Latency = E2E Telemetry + E2E Command
    // Como a full latency é uma soma e não uma simples diferença de tempos, preenchemos o msg à mão
    double e2e_telemetry_ms = (display_time - origin_time).seconds() * 1000.0;
    const double full_latency_ms = e2e_telemetry_ms + e2e_command_ms;

    auto msg_full = std::make_unique<Metrics>();
    msg_full->id = id;
    msg_full->tx = origin_stamp;
    msg_full->rx = display_time;
    msg_full->latency_ms = full_latency_ms;

    pub_full_latency_->publish(std::move(msg_full));
    return full_latency_ms;
}

// Tópico 5 (Câmara Frontal)
void RosBridge::publishFrontCameraMetrics(uint32_t frame_id, double latency_ms)
{
    auto msg = std::make_unique<Metrics>();
    msg->id = frame_id;
    msg->tx = this->now(); 
    msg->rx = this->now(); 
    msg->latency_ms = latency_ms;
    
    pub_front_camera_->publish(std::move(msg));
}

void RosBridge::publishFrontCameraNetwork(uint32_t frame_id, double latency_ms)
{
    auto msg = std::make_unique<Metrics>();
    msg->id = frame_id;
    msg->tx = this->now();
    msg->rx = this->now();
    msg->latency_ms = latency_ms;
    pub_front_camera_network_->publish(std::move(msg));
}
 
void RosBridge::publishFrontCameraDecode(uint32_t frame_id, double latency_ms)
{
    auto msg = std::make_unique<Metrics>();
    msg->id = frame_id;
    msg->tx = this->now();
    msg->rx = this->now();
    msg->latency_ms = latency_ms;
    pub_front_camera_decode_->publish(std::move(msg));
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
