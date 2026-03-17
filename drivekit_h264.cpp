#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

class H264Streamer : public rclcpp::Node {
public:
    H264Streamer() : Node("h264_streamer"), pipeline_(nullptr), appsrc_(nullptr) {
        // Inicializar GStreamer
        gst_init(nullptr, nullptr);

        // Pipeline — sem encoding, só empacotamento RTP
        std::string pipeline_str =
            "appsrc name=mysrc is-live=true do-timestamp=true format=time "
            "caps=video/x-h264,stream-format=byte-stream,alignment=au ! "
            "h264parse config-interval=-1 ! "
            "rtph264pay pt=96 mtu=1400 aggregate-mode=zero-latency ! "
            "udpsink host=192.168.94.202 port=5006 sync=false";

        GError *error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
        if (error) {
            RCLCPP_ERROR(this->get_logger(), "Erro ao criar pipeline: %s", error->message);
            g_error_free(error);
            return;
        }

        appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "mysrc");
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera_0/compressed_image",
            rclcpp::QoS(1).best_effort().keep_last(1),
            std::bind(&H264Streamer::image_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "H264Streamer iniciado. A enviar para 10.0.0.2:5006");
    }

    ~H264Streamer() {
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
        if (appsrc_) {
            gst_object_unref(appsrc_);
        }
    }

private:
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        if (!appsrc_) return;

        // Copiar o bitstream H264 para um buffer GStreamer
        GstBuffer *buffer = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
        GstMapInfo map;
        gst_buffer_map(buffer, &map, GST_MAP_WRITE);
        memcpy(map.data, msg->data.data(), msg->data.size());
        gst_buffer_unmap(buffer, &map);

        // Timestamp do buffer
        GST_BUFFER_PTS(buffer) = 
            (guint64)msg->header.stamp.sec * GST_SECOND + 
            (guint64)msg->header.stamp.nanosec;

        // Enviar para o pipeline
        GstFlowReturn ret;
        g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
        gst_buffer_unref(buffer);

        if (ret != GST_FLOW_OK) {
            RCLCPP_WARN(this->get_logger(), "Erro ao enviar buffer: %d", ret);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    GstElement *pipeline_;
    GstElement *appsrc_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<H264Streamer>());
    rclcpp::shutdown();
    return 0;
}