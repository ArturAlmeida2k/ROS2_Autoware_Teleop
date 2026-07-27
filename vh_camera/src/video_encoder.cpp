#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

class VideoEncoderTX : public rclcpp::Node {
public:
    VideoEncoderTX() : Node("video_encoder_tx_cpp"), writer_initialized_(false) {
        this->declare_parameter<std::string>("ip_address", "10.0.0.1");
        this->declare_parameter<int>("port", 5007);
        this->declare_parameter<int>("bitrate", 5000); 
        
        ip_address_ = this->get_parameter("ip_address").as_string();
        port_ = this->get_parameter("port").as_int();        
        bitrate_ = this->get_parameter("bitrate").as_int();

        rclcpp::QoS qos_profile(1);
        qos_profile.best_effort();
        qos_profile.keep_last(1);

        // Subscreve ao tópico que o nó de captura está a publicar
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/real/image_raw",
            qos_profile,
            std::bind(&VideoEncoderTX::image_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Encoder iniciado. A aguardar imagens... (Destino: %s:%d)", 
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
    int bitrate_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::VideoWriter writer_;
    bool writer_initialized_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;

            if (frame.empty()) return;

            if (!writer_initialized_) {
                int width = frame.cols;
                int height = frame.rows;
                int fps = 30;
                
                std::string pipeline = 
                    "appsrc is-live=true do-timestamp=true ! "
                    "videoconvert ! "
                    "queue ! "
                    "video/x-raw,format=I420 ! "
                    "x264enc tune=zerolatency speed-preset=superfast "
                    "sliced-threads=true threads=1 "
                    "key-int-max=15 intra-refresh=true "
                    "bitrate=" + std::to_string(bitrate_) + " ! "
                    "h264parse config-interval=-1 ! "
                    "rtph264pay pt=96 mtu=1400 aggregate-mode=zero-latency ! "
                    "udpsink host=" + ip_address_ + " port=" + std::to_string(port_) + " sync=false";
                    
                writer_.open(pipeline, cv::CAP_GSTREAMER, 0, fps, cv::Size(width, height), true);
                
                if (!writer_.isOpened()) {
                    RCLCPP_ERROR(this->get_logger(), "Falha ao abrir GStreamer");
                    return;
                }
                writer_initialized_ = true;
            }

            writer_.write(frame);
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Erro cv_bridge: %s", e.what());
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoEncoderTX>());
    rclcpp::shutdown();
    return 0;
}