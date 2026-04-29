#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

class VideoDecoderRX : public rclcpp::Node {
public:
    VideoDecoderRX() : Node("video_decoder_rx_cpp") {
        RCLCPP_INFO(this->get_logger(), "A inicializar descodificador C++...");
    }

    // Movemos a lógica da janela para uma função pública que correrá na Main Thread
    void run_gui_loop() {
        std::string pipeline =
            "udpsrc port=5007 buffer-size=2129920 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! "
            "rtpjitterbuffer latency=50 ! "
            "rtph264depay ! h264parse ! avdec_h264 output-corrupt=false ! "
            "videoconvert ! appsink sync=false drop=true max-buffers=1";

        RCLCPP_INFO(this->get_logger(), "A tentar abrir GStreamer... (À espera do emissor)");
        cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao abrir a porta UDP 5007 via GStreamer.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "A escutar vídeo em UDP... A janela vai abrir.");

        cv::Mat frame;
        while (rclcpp::ok()) {
            if (cap.read(frame) && !frame.empty()) {
                cv::imshow("PC B - Teleop Stream (Zero Latency)", frame);
            }
            
            // O waitKey fica na main thread, que processa os eventos da janela e do teclado
            if (cv::waitKey(1) == 27) { // Pressionar 'ESC' para sair
                break;
            }
        }

        cap.release();
        cv::destroyAllWindows();
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<VideoDecoderRX>();

    // Colocamos o ROS 2 (Spin) numa thread separada para não bloquear o OpenCV
    std::thread ros_thread([&node]() {
        rclcpp::spin(node);
    });

    // Corremos a interface gráfica do OpenCV na Thread Principal!
    node->run_gui_loop();

    // Se sairmos do loop do GUI (ex: clicou ESC), encerramos o ROS
    rclcpp::shutdown();
    if (ros_thread.joinable()) {
        ros_thread.join();
    }
    
    return 0;
}