#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <string>
#include <array>
#include <vector>

class VideoEncoderTX : public rclcpp::Node {
public:
    VideoEncoderTX() : Node("video_encoder_tx_cpp") {
        gst_init(NULL, NULL);

        this->declare_parameter<std::string>("ip_address", "10.0.0.2");
        this->declare_parameter<int>("port", 5007);

        ip_address = this->get_parameter("ip_address").as_string();
        int base_port = this->get_parameter("port").as_int();

        rclcpp::QoS qos_profile(1);
        qos_profile.best_effort();
        qos_profile.keep_last(1);

        std::vector<std::string> topics = {
            "/sensing/camera/CAM_BACK/image_raw",
            "/sensing/camera/CAM_FRONT/image_raw",
            "/sensing/camera/CAM_FRONT_LEFT/image_raw",
            "/sensing/camera/CAM_FRONT_RIGHT/image_raw"
        };

        for (size_t i = 0; i < topics.size(); ++i) {
            streams_[i].topic_name = topics[i];
            streams_[i].port = base_port + i;
            streams_[i].initialized = false;
            streams_[i].pipeline = nullptr;
            streams_[i].appsrc = nullptr;

            streams_[i].subscription = this->create_subscription<sensor_msgs::msg::Image>(
                topics[i],
                qos_profile,
                [this, i](const sensor_msgs::msg::Image::SharedPtr msg) {
                    this->image_callback(msg, i);
                }
            );
            
            RCLCPP_INFO(this->get_logger(), "Subscrito em: %s -> Stream alocada para porta UDP: %d", topics[i].c_str(), streams_[i].port);
        }
    }

    ~VideoEncoderTX() {
        for (auto& stream : streams_) {
            if (stream.pipeline) {
                gst_element_set_state(stream.pipeline, GST_STATE_NULL);
                gst_object_unref(stream.pipeline);
            }
        }
    }

private:
    struct CameraStream {
        std::string topic_name;
        int port;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
        GstElement *pipeline;
        GstElement *appsrc;
        bool initialized;
    };

    std::array<CameraStream, 4> streams_;
    std::string ip_address;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg, size_t i) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;

            if (frame.empty()) return;

            if (!streams_[i].initialized) {
                // Configuração da pipeline GStreamer com timestamp-offset fixado a 0
                std::string pipeline_str = 
                    "appsrc name=srcis live=true do-timestamp=false ! "
                    "videoconvert ! "
                    "queue ! "
                    "video/x-raw,format=I420 ! "
                    "x264enc tune=zerolatency speed-preset=superfast "
                    "sliced-threads=true threads=1 "
                    "key-int-max=15 intra-refresh=true "
                    "bitrate=4000 ! "
                    "h264parse config-interval=-1 ! "
                    "rtph264pay pt=96 mtu=1400 aggregate-mode=zero-latency timestamp-offset=0 ! "
                    "udpsink host=" + ip_address + " port=" + std::to_string(streams_[i].port) + " sync=false";

                GError *error = nullptr;
                streams_[i].pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
                
                if (error) {
                    RCLCPP_ERROR(this->get_logger(), "Erro na pipeline GStreamer: %s", error->message);
                    g_error_free(error);
                    return;
                }

                streams_[i].appsrc = gst_bin_get_by_name(GST_BIN(streams_[i].pipeline), "srcis");

                // Especificar o formato de entrada no appsrc (BGR do OpenCV)
                GstCaps *caps = gst_caps_new_simple("video/x-raw",
                    "format", G_TYPE_STRING, "BGR",
                    "width", G_TYPE_INT, frame.cols,
                    "height", G_TYPE_INT, frame.rows,
                    "framerate", GST_TYPE_FRACTION, 30, 1, NULL);
                
                g_object_set(G_OBJECT(streams_[i].appsrc), "caps", caps, NULL);
                gst_caps_unref(caps);

                gst_element_set_state(streams_[i].pipeline, GST_STATE_PLAYING);
                streams_[i].initialized = true;
            }

            // Alocar Buffer nativo do GStreamer e copiar os dados da frame
            gsize size = frame.total() * frame.elemSize();
            GstBuffer *buffer = gst_buffer_new_allocate(NULL, size, NULL);
            
            GstMapInfo map;
            gst_buffer_map(buffer, &map, GST_MAP_WRITE);
            memcpy(map.data, frame.data, size);
            gst_buffer_unmap(buffer, &map);

            // Injetar o timestamp do ROS 2 no metadado PTS do buffer (em nanossegundos)
            GST_BUFFER_PTS(buffer) = this->now().nanoseconds();
            GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, 30);

            // Empurrar o buffer para dentro do fluxo de processamento
            GstFlowReturn ret;
            g_signal_emit_by_name(streams_[i].appsrc, "push-buffer", buffer, &ret);
            gst_buffer_unref(buffer);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Erro no cv_bridge: %s", e.what());
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<VideoEncoderTX>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}