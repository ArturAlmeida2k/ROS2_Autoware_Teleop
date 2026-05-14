#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

class VideoEncoderTX : public rclcpp::Node {
public:
    VideoEncoderTX() : Node("video_encoder_tx_cpp"), writer_initialized_(false) {
        this->declare_parameter<std::string>("ip_address", "10.0.0.2");
        this->declare_parameter<int>("port", 5007);
        
        ip_address_ = this->get_parameter("ip_address").as_string();
        port_ = this->get_parameter("port").as_int();        
        
        rclcpp::QoS qos_profile(1);
        qos_profile.best_effort();
        qos_profile.keep_last(1);

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/sensing/camera/traffic_light/image_raw",
            qos_profile,
            std::bind(&VideoEncoderTX::image_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Nó C++ Encoder iniciado. A aguardar imagens... (Destino: %s:%d)", 
                    ip_address_.c_str(), port_);
    }

    ~VideoEncoderTX() {
        if (writer_.isOpened()) {
            writer_.release();
        }
    }

private:

    std::string ip_address_;
    int port_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::VideoWriter writer_;
    bool writer_initialized_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Conversão direta de ROS 2 para OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;

            if (frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "Aviso: Frame vazia recebida do AWSIM. A ignorar...");
                return;
            }

            if (!writer_initialized_) {
                int width = frame.cols;
                int height = frame.rows;
                int fps = 30;
                
                // Pipeline 
                std::string pipeline = 
                    "appsrc is-live=true do-timestamp=true ! "
                    "videoconvert ! "
                    "queue ! "
                    "video/x-raw,format=I420 ! "
                    "x264enc tune=zerolatency speed-preset=superfast "
                    "sliced-threads=true threads=1 "
                    "key-int-max=15 intra-refresh=true "
                    "bitrate=4000 ! "
                    "h264parse config-interval=-1 ! "
                    "rtph264pay pt=96 mtu=1400 aggregate-mode=zero-latency ! "
                    "udpsink host=" + ip_address_ + " port=" + std::to_string(port_) + " sync=false";
                    
                // O 0 significa API backend preferencial, mas vamos forçar explicitamente o CAP_GSTREAMER
                writer_.open(pipeline, cv::CAP_GSTREAMER, 0, fps, cv::Size(width, height), true);
                
                if (!writer_.isOpened()) {
                    RCLCPP_ERROR(this->get_logger(), "Falha ao abrir o pipeline GStreamer via OpenCV C++ (Resolucao: %dx%d)", width, height);
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Pipeline GStreamer aberto. A enviar para %s:%d", ip_address_.c_str(), port_);
                writer_initialized_ = true;
            }

            // Escreve a frame na placa gráfica
            writer_.write(frame);
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Erro no cv_bridge: %s", e.what());
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoEncoderTX>());
    rclcpp::shutdown();
    return 0;
}