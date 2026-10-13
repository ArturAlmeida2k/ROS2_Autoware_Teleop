#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>

#include <atomic>
#include <cstring>
#include <string>
#include <thread>

// Lê uma webcam (v4l2, MJPEG) e publica as imagens num tópico ROS, para usar com o video_encoder.

class WebcamPublisher : public rclcpp::Node {
public:
    WebcamPublisher() : Node("webcam_publisher") {
        declare_parameter<int>("camera_id", 0);
        declare_parameter<int>("width", 1280);
        declare_parameter<int>("height", 720);
        declare_parameter<int>("fps", 30);
        declare_parameter<std::string>("topic", "/webcam/image_raw");

        const int camera_id = get_parameter("camera_id").as_int();
        const int width     = get_parameter("width").as_int();
        const int height    = get_parameter("height").as_int();
        const int fps       = get_parameter("fps").as_int();
        const auto topic    = get_parameter("topic").as_string();

        cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);

        // appsink com 1 buffer e drop, para ler sempre a imagem mais recente
        const std::string pipeline =
            "v4l2src device=/dev/video" + std::to_string(camera_id) + " ! "
            "image/jpeg,width=" + std::to_string(width) + ",height=" + std::to_string(height) +
            ",framerate=" + std::to_string(fps) + "/1 ! "
            "jpegdec ! videoconvert ! video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false";

        cap_.open(pipeline, cv::CAP_GSTREAMER);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(get_logger(), "Failed to open /dev/video%d at %dx%d@%d (MJPEG).",
                         camera_id, width, height, fps);
            return;
        }

        rclcpp::QoS qos(1);
        qos.best_effort();
        pub_ = create_publisher<sensor_msgs::msg::Image>(topic, qos);

        RCLCPP_INFO(get_logger(), "Publishing /dev/video%d on %s (%dx%d@%d).",
                    camera_id, topic.c_str(), width, height, fps);
        thread_ = std::thread(&WebcamPublisher::loop, this);
    }

    ~WebcamPublisher() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (cap_.isOpened()) cap_.release();
    }

private:
    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    std::thread thread_;
    std::atomic<bool> running_{true};

    void loop() {
        cv::Mat frame;
        while (running_ && rclcpp::ok()) {
            if (!cap_.read(frame) || frame.empty()) continue;

            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp = now();
            msg->header.frame_id = "webcam";
            msg->height = frame.rows;
            msg->width = frame.cols;
            msg->encoding = "bgr8";
            msg->is_bigendian = false;
            msg->step = static_cast<uint32_t>(frame.step);
            msg->data.resize(frame.step * frame.rows);
            std::memcpy(msg->data.data(), frame.data, msg->data.size());
            pub_->publish(std::move(msg));
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WebcamPublisher>());
    rclcpp::shutdown();
    return 0;
}
