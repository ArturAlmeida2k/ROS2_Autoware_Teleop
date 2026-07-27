#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;

class CameraCaptureNode : public rclcpp::Node {
public:
    CameraCaptureNode() : Node("camera_capture_node") {
        // Parâmetros para flexibilidade
        this->declare_parameter<int>("camera_id", 0);
        this->declare_parameter<int>("width", 1280);
        this->declare_parameter<int>("height", 720);
        this->declare_parameter<int>("fps", 30);

        int camera_id = this->get_parameter("camera_id").as_int();
        int width = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();
        int fps = this->get_parameter("fps").as_int();

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/real/image_raw", 10);

        // Abre a câmara usando o backend V4L2 (nativo do Linux, o mais rápido)
        cap_.open(camera_id, cv::CAP_V4L2);
        
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao abrir a câmara com o ID: %d", camera_id);
            return;
        }

        // Forçar resolução e FPS na própria câmara
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        cap_.set(cv::CAP_PROP_FPS, fps);

        RCLCPP_INFO(this->get_logger(), "Câmara RealSense aberta: %dx%d @ %dfps", width, height, fps);

        // Criar timer para capturar frames à frequência do FPS
        int timer_ms = 1000 / fps;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_ms),
            std::bind(&CameraCaptureNode::capture_and_publish, this)
        );
    }

    ~CameraCaptureNode() {
        if (cap_.isOpened()) {
            cap_.release();
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;

    void capture_and_publish() {
        cv::Mat frame;
        cap_ >> frame; // Captura do buffer de hardware

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Frame vazio ignorado.");
            return;
        }

        // Converter OpenCV Mat para ROS 2 Image e publicar
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "realsense_rgb_frame";

        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        publisher_->publish(*msg);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraCaptureNode>());
    rclcpp::shutdown();
    return 0;
}