#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <cstring>

class CameraStreamerNode : public rclcpp::Node {
public:
    CameraStreamerNode() : Node("camera_streamer_node"),
                           pipeline_(nullptr),
                           appsrc_(nullptr),
                           running_(true),
                           frame_counter_(1) {

        this->declare_parameter<int>("camera_id", 2);
        this->declare_parameter<int>("width", 1280);   // 720p — 1080p raw excede Gigabit
        this->declare_parameter<int>("height", 720);
        this->declare_parameter<int>("fps", 30);
        this->declare_parameter<std::string>("ip_address", "127.0.0.1");
        this->declare_parameter<int>("port", 5007);

        int camera_id = this->get_parameter("camera_id").as_int();
        width_        = this->get_parameter("width").as_int();
        height_       = this->get_parameter("height").as_int();
        int fps       = this->get_parameter("fps").as_int();
        ip_address_   = this->get_parameter("ip_address").as_string();
        port_         = this->get_parameter("port").as_int();

        gst_init(nullptr, nullptr);
        cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);

        std::string cam_pipeline =
            "v4l2src device=/dev/video" + std::to_string(camera_id) + " ! "
            "image/jpeg,width=" + std::to_string(width_) + ",height=" + std::to_string(height_) +
            ",framerate=" + std::to_string(fps) + "/1 ! "
            "jpegdec ! videoconvert ! video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1";

        cap_.open(cam_pipeline, cv::CAP_GSTREAMER);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao abrir a câmara via GStreamer.");
            return;
        }

        init_gstreamer_pipeline(width_, height_, fps);

        capture_thread_ = std::thread(&CameraStreamerNode::capture_loop, this);
    }

    ~CameraStreamerNode() {
        running_ = false;
        if (capture_thread_.joinable()) capture_thread_.join();
        if (cap_.isOpened()) cap_.release();
        if (appsrc_) gst_object_unref(appsrc_);
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Encerramento limpo concluído.");
    }

private:
    cv::VideoCapture cap_;
    std::thread capture_thread_;

    std::string ip_address_;
    int port_;
    int width_, height_;

    GstElement *pipeline_;
    GstElement *appsrc_;
    std::atomic<bool> running_;
    uint64_t frame_counter_;

    void init_gstreamer_pipeline(int width, int height, int fps) {
        std::string caps_str = "video/x-raw,format=BGR,width=" + std::to_string(width) +
                               ",height=" + std::to_string(height) + ",framerate=" + std::to_string(fps) + "/1";

        // SEM x264enc, SEM h264parse — vídeo raw, payload direto via RTP (RFC 4175).
        std::string pipeline_str =
            "appsrc name=mysrc is-live=true do-timestamp=true format=time caps=\"" + caps_str + "\" ! "
            "rtpvrawpay mtu=1400 ! "
            "udpsink host=" + ip_address_ + " port=" + std::to_string(port_) + " sync=false async=false";

        GError *error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
        if (error) {
            RCLCPP_ERROR(this->get_logger(), "Erro ao criar pipeline: %s", error->message);
            g_error_free(error);
            return;
        }

        appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "mysrc");
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        RCLCPP_INFO(this->get_logger(), "A transmitir video RAW (sem encoding) para %s:%d", ip_address_.c_str(), port_);
    }

    void capture_loop() {
        try {
            cv::Mat frame;
            while (running_ && rclcpp::ok()) {

                cap_ >> frame;
                if (!running_) break;
                if (frame.empty()) continue;

                uint64_t current_id = frame_counter_++;
                auto now = std::chrono::system_clock::now().time_since_epoch();
                uint64_t ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

                guint size = frame.total() * frame.elemSize();
                GstBuffer *buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
                GstMapInfo map;
                gst_buffer_map(buffer, &map, GST_MAP_WRITE);
                std::memcpy(map.data, frame.data, size);

                // Header de texto embutido nos primeiros bytes do buffer de pixels
                // (sem NALU/SEI em raw). Fica visível como um pequeno risco no canto
                // superior esquerdo — irrelevante para este teste de diagnóstico.
                std::string header = "ID:" + std::to_string(current_id) + "|TS:" + std::to_string(ts_ns);
                if (map.size > header.size() + 1) {
                    std::memcpy(map.data, header.c_str(), header.size());
                    map.data[header.size()] = '\0';
                }

                gst_buffer_unmap(buffer, &map);

                GstFlowReturn ret;
                g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
                gst_buffer_unref(buffer);
            }
        } catch (...) {
            // Prevenção de exceções durante o encerramento
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraStreamerNode>());
    rclcpp::shutdown();
    return 0;
}