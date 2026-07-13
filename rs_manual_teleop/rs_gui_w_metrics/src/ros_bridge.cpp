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
    
    sub_telemetry_ = create_subscription<TelemetryState>(
        "/telemetry/state", 10,
        [this](const TelemetryState::SharedPtr msg) {
            rclcpp::Time tempo_rececao = this->now();
            
            // 1. Publica DIRETAMENTE para o tópico de rede
            publish_metric(pub_telemetry_decoder_, msg->id, tempo_rececao, rclcpp::Time(msg->header.stamp));

            emit telemetryReceived(*msg, tempo_rececao.nanoseconds());
        });
    
    sub_pointcloud_ = create_subscription<PointCloud2>(
        "/sensing/lidar/concatenated/pointcloud", 
        rclcpp::SensorDataQoS(),
        [this](const PointCloud2::SharedPtr msg) {
            emit pointCloudReceived(msg);
        });
        
    sub_front_ = create_subscription<CompressedImage>(
        "/camera/front/compressed", rclcpp::SensorDataQoS(),
        [this](const CompressedImage::SharedPtr msg) {
            rclcpp::Time tempo_rececao = this->now();

            emit imageFrontReceived(msg, tempo_rececao.nanoseconds());
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
void RosBridge::publishTelemetryGuiMetrics(uint32_t id, double original_e2e, int64_t rx_time_ns, int64_t display_time_ns)
{
    rclcpp::Time rx_time(rx_time_ns);
    rclcpp::Time display_time(display_time_ns);

    // Tópico 2
    publish_metric(pub_telemetry_gui_, id, display_time, rx_time);

    // Tópico 3 (Float64 publicado à parte)
    double gui_latency_ms = (display_time - rx_time).seconds() * 1000.0;
    auto msg_e2e = std::make_unique<Float64>();
    msg_e2e->data = original_e2e + gui_latency_ms; 
    pub_e2e_telemetry_->publish(std::move(msg_e2e));
}

// Tópico 4 (Câmara Frontal)
void RosBridge::publishFrontCameraMetrics(int64_t rx_time_ns, int64_t display_time_ns)
{
    // Como a imagem comprimida nativa não tem campo ID adaptado na frame_id para int, usa 0
    publish_metric(pub_front_camera_, 0, rclcpp::Time(display_time_ns), rclcpp::Time(rx_time_ns));
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
