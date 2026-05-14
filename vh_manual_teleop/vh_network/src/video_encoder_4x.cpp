#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <array>

class VideoEncoderTX : public rclcpp::Node {
public:
    VideoEncoderTX() : Node("video_encoder_tx_cpp") {

        this->declare_parameter<std::string>("ip_address", "10.0.0.2");
        this->declare_parameter<int>("port", 5007);

        ip_address = this->get_parameter("ip_address").as_string();
        int base_port = this->get_parameter("port").as_int();


        rclcpp::QoS qos_profile(1);
        qos_profile.best_effort();
        qos_profile.keep_last(1);

        // Definir os tópicos e as portas correspondentes
        std::vector<std::string> topics = {
            "/sensing/camera/CAM_BACK/image_raw",
            "/sensing/camera/CAM_FRONT/image_raw",
            "/sensing/camera/CAM_FRONT_LEFT/image_raw",
            "/sensing/camera/CAM_FRONT_RIGHT/image_raw"
        };
        
        RCLCPP_INFO(this->get_logger(), "Configurado para enviar vídeo para o IP: %s (Porta base: %d)", 
                    ip_address.c_str(), base_port);

        for (size_t i = 0; i < topics.size(); ++i) {
            streams_[i].topic_name = topics[i];
            streams_[i].port = base_port + i;
            streams_[i].writer_initialized = false;

            // Criação do subscritor usando uma expressão lambda para capturar o índice (i) da câmara
            streams_[i].subscription = this->create_subscription<sensor_msgs::msg::Image>(
                topics[i],
                qos_profile,
                [this, i](const sensor_msgs::msg::Image::SharedPtr msg) {
                    this->image_callback(msg, i);
                }
            );

            RCLCPP_INFO(this->get_logger(), "Subscrito em: %s -> Stream alocada para porta UDP: %d", topics[i].c_str(), streams_[i].port);
        }

        RCLCPP_INFO(this->get_logger(), "Nó C++ Encoder iniciado. A aguardar imagens do AWSIM...");
    }

    ~VideoEncoderTX() {
        for (auto& stream : streams_) {
            if (stream.writer.isOpened()) {
                stream.writer.release();
            }
        }
    }

private:
    struct CameraStream {
        std::string topic_name;
        int port;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
        cv::VideoWriter writer;
        bool writer_initialized;
    };

    // Array para armazenar o estado das 4 câmaras
    std::array<CameraStream, 4> streams_;

    std::string ip_address;


    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg, size_t camera_index) {
        try {
            // Conversão direta de ROS 2 para OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;

            if (frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "Aviso: Frame vazia recebida do tópico %s. A ignorar...", streams_[camera_index].topic_name.c_str());
                return;
            }

            if (!streams_[camera_index].writer_initialized) {
                int width = frame.cols;
                int height = frame.rows;
                int fps = 30;
                
                // Pipeline dinâmico que atribui a porta correta à udpsink
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
                    "udpsink host=" + ip_address + " port=" + std::to_string(streams_[camera_index].port) + " sync=false";
                
                // Abre o escritor associado a esta câmara específica
                streams_[camera_index].writer.open(pipeline, cv::CAP_GSTREAMER, 0, fps, cv::Size(width, height), true);
                
                if (!streams_[camera_index].writer.isOpened()) {
                    RCLCPP_ERROR(this->get_logger(), "Falha ao abrir pipeline GStreamer para %s (Resolucao: %dx%d)", 
                                 streams_[camera_index].topic_name.c_str(), width, height);
                    return;
                }
                
                RCLCPP_INFO(this->get_logger(), "Pipeline GStreamer aberto para %s. A enviar para 10.0.0.2:%d", 
                            streams_[camera_index].topic_name.c_str(), streams_[camera_index].port);
                
                streams_[camera_index].writer_initialized = true;
            }

            // Escreve a frame
            streams_[camera_index].writer.write(frame);
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Erro no cv_bridge (%s): %s", streams_[camera_index].topic_name.c_str(), e.what());
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    // Usar um MultiThreadedExecutor garante que, se várias câmaras publicarem ao mesmo tempo, 
    // os callbacks não encravam à espera uns dos outros.
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<VideoEncoderTX>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
