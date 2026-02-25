#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

class VideoEncoderTX : public rclcpp::Node {
public:
    VideoEncoderTX() : Node("video_encoder_tx_cpp"), writer_initialized_(false) {
        // Configuração de QoS idêntica à do AWSIM para evitar a fila de atraso
        rclcpp::QoS qos_profile(1);
        qos_profile.best_effort();
        qos_profile.keep_last(1);

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/sensing/camera/traffic_light/image_raw",
            qos_profile,
            std::bind(&VideoEncoderTX::image_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Nó C++ Encoder iniciado. A aguardar imagens do AWSIM...");
    }

    ~VideoEncoderTX() {
        if (writer_.isOpened()) {
            writer_.release();
        }
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Conversão direta de ROS 2 para OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;

            if (!writer_initialized_) {
                int width = frame.cols;
                int height = frame.rows;
                int fps = 30;
                
                // Pipeline de compressão NVENC a apontar para 127.0.0.1 (Loopback)
                std::string pipeline = 
                    "appsrc is-live=true do-timestamp=true ! "
                    "videoconvert ! video/x-raw,format=I420 ! "
                    "nvh264enc preset=low-latency rc-mode=cbr bitrate=8000 zerolatency=true gop-size=30 insert-sps-pps=true ! "
                    "rtph264pay config-interval=1 pt=96 mtu=1300 ! "
                    "udpsink host=127.0.0.1 port=5006 sync=false";

                writer_.open(pipeline, cv::CAP_GSTREAMER, 0, fps, cv::Size(width, height), true);
                
                if (!writer_.isOpened()) {
                    RCLCPP_ERROR(this->get_logger(), "Falha ao abrir o pipeline GStreamer via OpenCV C++.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Pipeline GStreamer (NVENC) aberto. A enviar para 127.0.0.1:5006");
                writer_initialized_ = true;
            }

            // Escreve a frame na placa gráfica
            writer_.write(frame);
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Erro no cv_bridge: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::VideoWriter writer_;
    bool writer_initialized_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoEncoderTX>());
    rclcpp::shutdown();
    return 0;
}