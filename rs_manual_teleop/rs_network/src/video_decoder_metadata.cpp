#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <thread>
#include <vector>
#include <array>
#include <string>

class VideoDecoderRX : public rclcpp::Node {
public:
    VideoDecoderRX() : Node("video_decoder_rx_multi_cpp") {
        gst_init(NULL, NULL);

        this->declare_parameter<std::string>("ip_address", "10.0.0.1");
        this->declare_parameter<int>("port", 5007); 

        int base_port = this->get_parameter("port").as_int();

        struct CameraConfig {
            int port;
            std::string topic_name;
            std::string frame_id;
        };

        std::array<CameraConfig, 4> configs = {{
            {base_port,     "/camera/back/compressed",  "camera_back"},
            {base_port + 1, "/camera/front/compressed", "camera_front"},
            {base_port + 2, "/camera/left/compressed",  "camera_left"},
            {base_port + 3, "/camera/right/compressed", "camera_right"}
        }};

        for (size_t i = 0; i < configs.size(); ++i) {
            publishers_[i] = this->create_publisher<sensor_msgs::msg::CompressedImage>(configs[i].topic_name, 10);
            video_threads_[i] = std::thread(&VideoDecoderRX::receive_and_publish, this, 
                                            configs[i].port, configs[i].frame_id, publishers_[i]);
        }
    }

    ~VideoDecoderRX() {
        for (auto& t : video_threads_) {
            if (t.joinable()) t.join();
        }
    }

private:
    void receive_and_publish(int port, const std::string& frame_id, rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher) 
    {   

        std::string pipeline_str =
            "udpsrc port=" + std::to_string(port) + " buffer-size=2129920 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! "
            "rtpjitterbuffer latency=10 ! "
            "rtph264depay ! h264parse ! avdec_h264 output-corrupt=false ! "
            "videoconvert ! appsink name=sinkis sync=false drop=true max-buffers=1";

        GError *error = nullptr;
        GstElement *pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
        if (error) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao criar pipeline RX na porta %d: %s", port, error->message);
            g_error_free(error);
            return;
        }

        GstElement *appsink = gst_bin_get_by_name(GST_BIN(pipeline), "sinkis");
        
        // Configurar o appsink para expor formato BGR nativo para OpenCV
        GstCaps *caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "BGR", NULL);
        g_object_set(G_OBJECT(appsink), "caps", caps, NULL);
        gst_caps_unref(caps);

        gst_element_set_state(pipeline, GST_STATE_PLAYING);

        std::vector<uint8_t> jpeg_buffer;
        std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, 90};

        while (rclcpp::ok()) {
            GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));
            if (!sample) continue;

            GstBuffer *buffer = gst_sample_get_buffer(sample);
            GstCaps *sample_caps = gst_sample_get_caps(sample);
            GstStructure *structure = gst_caps_get_structure(sample_caps, 0);

            int width = 0, height = 0;
            gst_structure_get_int(structure, "width", &width);
            gst_structure_get_int(structure, "height", &height);

            // Capturar o PTS enviado (reconstruído a partir do relógio RTP de 90kHz)
            GstClockTime tx_pts = GST_BUFFER_PTS(buffer);

            GstMapInfo map;
            if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
                // Embrulhar os dados binários diretamente numa matriz OpenCV
                cv::Mat frame(height, width, CV_8UC3, map.data);

                if (!frame.empty() && GST_CLOCK_TIME_IS_VALID(tx_pts)) {
                    uint64_t rx_time_ns = this->now().nanoseconds();

                    // Conversão matemática para tratar a rotação (overflow) de 32 bits do relógio RTP
                    uint32_t tx_rtp_clk = (uint32_t)((tx_pts * 90000) / 1000000000);
                    uint32_t rx_rtp_clk = (uint32_t)((rx_time_ns * 90000) / 1000000000);

                    // Aritmética de inteiros sem sinal resolve a rotação automaticamente
                    uint32_t diff_clk = rx_rtp_clk - tx_rtp_clk;
                    double latency_ms = (double)diff_clk / 90.0;

                    RCLCPP_INFO(this->get_logger(), "[%s] Latência de Software: %.2f ms", frame_id.c_str(), latency_ms);

                    // Proceder com a codificação para publicação ROS 2
                    if (cv::imencode(".jpg", frame, jpeg_buffer, encode_params)) {
                        auto msg = sensor_msgs::msg::CompressedImage();
                        msg.header.stamp = this->now();
                        msg.header.frame_id = frame_id;
                        msg.format = "jpeg";
                        msg.data = jpeg_buffer;
                        publisher->publish(msg);
                    }
                }
                gst_buffer_unmap(buffer, &map);
            }
            gst_sample_unref(sample);
        }

        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
        gst_object_unref(appsink);
    }

    std::array<std::thread, 4> video_threads_;
    std::array<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr, 4> publishers_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoDecoderRX>());
    rclcpp::shutdown();
    return 0;
}