#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

class VideoDecoderRX : public rclcpp::Node {
public:
    VideoDecoderRX() : Node("video_decoder_rx_cpp") {
        RCLCPP_INFO(this->get_logger(), "A inicializar descodificador C++...");
        
        // Iniciar a leitura de vídeo numa thread separada para latência zero
        video_thread_ = std::thread(&VideoDecoderRX::receive_and_display, this);
    }

    ~VideoDecoderRX() {
        if (video_thread_.joinable()) {
            video_thread_.join();
        }
    }

private:
    void receive_and_display() {
        // Pipeline equivalente à lógica da TUM, mas adaptado para UDP e appsink do OpenCV
    
        std::string pipeline =
            "udpsrc port=5006 buffer-size=2129920 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! "
            "rtpjitterbuffer latency=50 ! "
            "rtph264depay ! h264parse ! avdec_h264 output-corrupt=false ! "
            "videoconvert ! appsink sync=false drop=true max-buffers=1";

        cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao abrir a porta UDP 5006 via GStreamer.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "A escutar vídeo em UDP (Porta 5006)... A janela abrirá quando o veículo transmitir.");

        cv::Mat frame;
        // Loop contínuo independente dos timers do ROS 2
        while (rclcpp::ok()) {
            if (cap.read(frame) && !frame.empty()) {
                cv::imshow("PC B - Teleop Stream (Zero Latency)", frame);
                
                // Atualiza o ecrã a cada 1 milissegundo
                if (cv::waitKey(1) == 27) { // Pressionar 'ESC' para sair
                    break;
                }
            }
        }

        cap.release();
        cv::destroyAllWindows();
    }

    std::thread video_thread_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoDecoderRX>());
    rclcpp::shutdown();
    return 0;
}