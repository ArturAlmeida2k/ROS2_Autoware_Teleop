#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <thread>
#include <vector>
#include <array>
#include <string>

class VideoDecoderRX : public rclcpp::Node {
public:
    VideoDecoderRX() : Node("video_decoder_rx_multi_cpp") {
        RCLCPP_INFO(this->get_logger(), "A inicializar descodificador C++ para 4 camaras...");

        // Estrutura para organizar os parâmetros de cada fluxo de vídeo
        struct CameraConfig {
            int port;
            std::string topic_name;
            std::string frame_id;
        };

        // Mapeamento das 4 câmaras com base nas portas e tópicos do transmissor
        std::array<CameraConfig, 4> configs = {{
            {5006, "/camera/back/compressed", "camera_back"},
            {5007, "/camera/front/compressed", "camera_front"},
            {5008, "/camera/left/compressed", "camera_left"},
            {5009, "/camera/right/compressed", "camera_right"}
        }};

        for (size_t i = 0; i < configs.size(); ++i) {
            // Inicializar o publicador de cada câmara
            publishers_[i] = this->create_publisher<sensor_msgs::msg::CompressedImage>(
                configs[i].topic_name, 10);

            // Lançar uma thread dedicada para o loop de receção e publicação de cada câmara
            video_threads_[i] = std::thread(&VideoDecoderRX::receive_and_publish, this, 
                                            configs[i].port, configs[i].topic_name, configs[i].frame_id, publishers_[i]);
        }
    }

    ~VideoDecoderRX() {
        // Garantir que as threads encerram corretamente antes de destruir o nó
        for (auto& t : video_threads_) {
            if (t.joinable()) {
                t.join();
            }
        }
    }

private:
    void receive_and_publish(int port, const std::string& topic_name, const std::string& frame_id, 
                             rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher) {
        
        // Pipeline construída dinamicamente para escutar na porta UDP correta
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

        RCLCPP_INFO(this->get_logger(), "A escutar vídeo em UDP (Porta %d)... A publicar em %s", port, topic_name.c_str());

        cv::Mat frame;
        std::vector<uint8_t> jpeg_buffer;
        std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, 90};

        // Loop contínuo independente para esta porta específica
        while (rclcpp::ok()) {
            if (!cap.read(frame) || frame.empty()) {
                continue;
            }

            // Codifica o frame como JPEG em memória
            if (!cv::imencode(".jpg", frame, jpeg_buffer, encode_params)) {
                RCLCPP_WARN(this->get_logger(), "Falha ao codificar frame JPEG na porta %d.", port);
                continue;
            }

            // Preenche a mensagem CompressedImage
            auto msg = sensor_msgs::msg::CompressedImage();
            msg.header.stamp = this->now();
            msg.header.frame_id = frame_id;
            msg.format = "jpeg";
            msg.data = jpeg_buffer;

            // Publica no tópico correspondente
            publisher->publish(msg);
        }

        cap.release();
    }

    // Arrays para guardar as referências às threads e publicadores
    std::array<std::thread, 4> video_threads_;
    std::array<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr, 4> publishers_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    
    // Neste caso, o MultiThreadedExecutor não é estritamente necessário no main(), 
    // porque criámos nós as threads manualmente usando o std::thread na classe.
    // O rclcpp::spin() regular será suficiente para manter o nó vivo.
    rclcpp::spin(std::make_shared<VideoDecoderRX>());
    
    rclcpp::shutdown();
    return 0;
}