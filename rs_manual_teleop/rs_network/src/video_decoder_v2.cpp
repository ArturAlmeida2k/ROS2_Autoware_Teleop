#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <thread>
#include <vector>

class VideoDecoderRX : public rclcpp::Node {
public:
    VideoDecoderRX() : Node("video_decoder_rx_cpp") {

        this->declare_parameter<std::string>("ip_address", "10.0.0.1");
        this->declare_parameter<int>("port", 5007);

        int port = this->get_parameter("port").as_int();
        
        RCLCPP_INFO(this->get_logger(), "A inicializar descodificador C++...");

        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/camera/front/compressed", 10);

        video_thread_ = std::thread(&VideoDecoderRX::receive_and_publish, this, port);
    }

    ~VideoDecoderRX() {
        if (video_thread_.joinable()) {
            video_thread_.join();
        }
    }

private:
    void receive_and_publish(int port) {

        std::string pipeline =
            "udpsrc port=" + std::to_string(port) + " buffer-size=2129920 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! "
            "rtpjitterbuffer latency=10 ! "
            "rtph264depay ! h264parse ! avdec_h264 output-corrupt=false ! "
            "videoconvert ! appsink sync=false drop=true max-buffers=1";

        cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao abrir a porta UDP %d via GStreamer.", port);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "A escutar vídeo em UDP (Porta %d)... A publicar em /camera/front/compressed.", port);
        
        cv::Mat frame;
        std::vector<uint8_t> jpeg_buffer;
        std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, 90};

        while (rclcpp::ok()) {
            if (!cap.read(frame) || frame.empty()) {
                continue;
            }

            // Codifica o frame como JPEG em memória
            if (!cv::imencode(".jpg", frame, jpeg_buffer, encode_params)) {
                RCLCPP_WARN(this->get_logger(), "Falha ao codificar frame JPEG.");
                continue;
            }

            // Preenche a mensagem CompressedImage
            auto msg = sensor_msgs::msg::CompressedImage();
            msg.header.stamp = this->now();
            msg.header.frame_id = "camera_front";
            msg.format = "jpeg";
            msg.data = jpeg_buffer;

            publisher_->publish(msg);
        }

        cap.release();
    }

    std::thread video_thread_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoDecoderRX>());
    rclcpp::shutdown();
    return 0;
}