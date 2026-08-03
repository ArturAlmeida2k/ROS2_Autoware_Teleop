#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <string>
#include <cstring>
#include <queue>
#include <mutex>
#include <vector>
#include <chrono> 

class VideoEncoderTX : public rclcpp::Node {
public:
    VideoEncoderTX() : Node("video_encoder_tx_cpp"), pipeline_initialized_(false), frame_counter_(1) {
        this->declare_parameter<std::string>("ip_address", "10.0.0.2");
        this->declare_parameter<int>("port", 5007);

        ip_address_ = this->get_parameter("ip_address").as_string();
        port_ = this->get_parameter("port").as_int();

        rclcpp::QoS qos_profile(1);
        qos_profile.best_effort();
        qos_profile.keep_last(1);

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/sensing/camera/traffic_light/image_raw",
            qos_profile,
            std::bind(&VideoEncoderTX::image_callback, this, std::placeholders::_1)
        );

        gst_init(nullptr, nullptr);
        RCLCPP_INFO(this->get_logger(), "Encoder Nativo iniciado com SEI + Chrono. (Destino: %s:%d)",
                    ip_address_.c_str(), port_);
    }

    ~VideoEncoderTX() {
        if (bus_watch_id_ > 0) {
            g_source_remove(bus_watch_id_);
        }
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
    }

private:
    std::string ip_address_;
    int port_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    bool pipeline_initialized_;
    GstElement *pipeline_ = nullptr;
    GstElement *appsrc_ = nullptr;
    guint bus_watch_id_ = 0;

    std::queue<std::pair<uint64_t, uint64_t>> metadata_queue_;
    std::mutex queue_mutex_;
    uint64_t frame_counter_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;

            if (frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "Aviso: Frame vazia recebida do AWSIM. A ignorar...");
                return;
            }

            if (!pipeline_initialized_) {
                init_gstreamer_pipeline(frame.cols, frame.rows);
            }

            // CORREÇÃO DA LATÊNCIA: Usar o tempo real do Sistema Operativo em nanosegundos
            auto now = std::chrono::system_clock::now().time_since_epoch();
            uint64_t ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
            uint64_t current_id = frame_counter_++;

            // Converter OpenCV Mat para GstBuffer
            guint size = frame.total() * frame.elemSize();
            GstBuffer *buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
            GstMapInfo map;

            gst_buffer_map(buffer, &map, GST_MAP_WRITE);
            std::memcpy(map.data, frame.data, size);
            gst_buffer_unmap(buffer, &map);

            // Empurrar o frame para o GStreamer
            GstFlowReturn ret;
            g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);

            if (ret == GST_FLOW_OK) {
                // O GStreamer gerou o buffer com sucesso, vamos registar o SEI na fila
                std::lock_guard<std::mutex> lock(queue_mutex_);
                metadata_queue_.push({current_id, ts_ns});
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "GStreamer rejeitou o buffer (Erro: %d)", ret);
            }
            gst_buffer_unref(buffer);
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Erro no cv_bridge: %s", e.what());
        }
    }

    void init_gstreamer_pipeline(int width, int height) {
        std::string caps_str = "video/x-raw,format=BGR,width=" + std::to_string(width) +
                               ",height=" + std::to_string(height) + ",framerate=30/1";

        // Deixamos o do-timestamp=true para o GStreamer gerir perfeitamente a fluidez RTP
        // e usamos o stream-format=byte-stream para aceitar o nosso SEI sem quebrar a rede.
        std::string pipeline_str =
            "appsrc name=mysrc is-live=true do-timestamp=true format=time caps=\"" + caps_str + "\" ! "
            "videoconvert ! "
            "queue ! "
            "video/x-raw,format=I420 ! "
            "x264enc tune=zerolatency speed-preset=superfast sliced-threads=true threads=1 key-int-max=15 intra-refresh=true bitrate=4000 ! "
            "h264parse config-interval=-1 name=parser ! "
            "video/x-h264,stream-format=byte-stream,alignment=au ! "
            "rtph264pay pt=96 mtu=1400 aggregate-mode=zero-latency ! "
            "udpsink host=" + ip_address_ + " port=" + std::to_string(port_) + " sync=false async=false";

        GError *error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
        if (error) {
            RCLCPP_ERROR(this->get_logger(), "Erro ao criar pipeline: %s", error->message);
            g_error_free(error);
            return;
        }

        appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "mysrc");

        // Instalar a Sonda (Intercetor)
        GstElement *parser = gst_bin_get_by_name(GST_BIN(pipeline_), "parser");
        GstPad *src_pad = gst_element_get_static_pad(parser, "src");
        gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER, (GstPadProbeCallback)pad_probe_callback, this, nullptr);
        gst_object_unref(src_pad);
        gst_object_unref(parser);

        // Monitor de Erros
        GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
        bus_watch_id_ = gst_bus_add_watch(bus, (GstBusFunc)bus_callback, this);
        gst_object_unref(bus);

        // Arrancar
        GstStateChangeReturn ret_state = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        if (ret_state == GST_STATE_CHANGE_FAILURE) {
            RCLCPP_ERROR(this->get_logger(), "FALHA CRÍTICA: A pipeline do GStreamer recusou-se a iniciar!");
        } else {
            RCLCPP_INFO(this->get_logger(), "Pipeline GStreamer iniciada com sucesso. Resolucao: %dx%d -> %s:%d",
                        width, height, ip_address_.c_str(), port_);
        }

        pipeline_initialized_ = true;
    }

    static gboolean bus_callback(GstBus * /*bus*/, GstMessage *msg, gpointer user_data) {
        auto *node = static_cast<VideoEncoderTX*>(user_data);
        switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR: {
                GError *err = nullptr;
                gchar *debug = nullptr;
                gst_message_parse_error(msg, &err, &debug);
                RCLCPP_ERROR(node->get_logger(), "Erro GStreamer: %s (debug: %s)",
                             err->message, debug ? debug : "n/a");
                g_error_free(err);
                g_free(debug);
                break;
            }
            case GST_MESSAGE_WARNING: {
                GError *err = nullptr;
                gchar *debug = nullptr;
                gst_message_parse_warning(msg, &err, &debug);
                RCLCPP_WARN(node->get_logger(), "Aviso GStreamer: %s (debug: %s)",
                            err->message, debug ? debug : "n/a");
                g_error_free(err);
                g_free(debug);
                break;
            }
            default:
                break;
        }
        return TRUE;
    }

    // --- CIRURGIA SEI ---
    static GstPadProbeReturn pad_probe_callback(GstPad * /*pad*/, GstPadProbeInfo *info, gpointer user_data) {
        auto *node = static_cast<VideoEncoderTX*>(user_data);
        GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);

        uint64_t frame_id = 0;
        uint64_t ts_ns = 0;

        {
            std::lock_guard<std::mutex> lock(node->queue_mutex_);
            if (!node->metadata_queue_.empty()) {
                frame_id = node->metadata_queue_.front().first;
                ts_ns = node->metadata_queue_.front().second;
                node->metadata_queue_.pop();
            } else {
                return GST_PAD_PROBE_OK;
            }
        }

        std::vector<uint8_t> sei_nalu;
        sei_nalu.push_back(0x00); sei_nalu.push_back(0x00); sei_nalu.push_back(0x00); sei_nalu.push_back(0x01);
        sei_nalu.push_back(0x06);
        sei_nalu.push_back(0x05);

        std::string payload = "ID:" + std::to_string(frame_id) + "|TS:" + std::to_string(ts_ns) + (char)0x80;
        size_t payload_size = 16 + payload.length();

        size_t s = payload_size;
        while (s >= 255) { sei_nalu.push_back(0xFF); s -= 255; }
        sei_nalu.push_back(s);

        const uint8_t uuid[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
                                  0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11};
        sei_nalu.insert(sei_nalu.end(), uuid, uuid + 16);
        sei_nalu.insert(sei_nalu.end(), payload.begin(), payload.end());

        GstMapInfo old_map;
        gst_buffer_map(buffer, &old_map, GST_MAP_READ);

        GstBuffer *new_buf = gst_buffer_new_allocate(nullptr, sei_nalu.size() + old_map.size, nullptr);
        GstMapInfo new_map;
        gst_buffer_map(new_buf, &new_map, GST_MAP_WRITE);

        std::memcpy(new_map.data, sei_nalu.data(), sei_nalu.size());
        std::memcpy(new_map.data + sei_nalu.size(), old_map.data, old_map.size);

        gst_buffer_unmap(new_buf, &new_map);
        gst_buffer_unmap(buffer, &old_map);

        gst_buffer_copy_into(new_buf, buffer, GST_BUFFER_COPY_METADATA, 0, -1);

        GST_PAD_PROBE_INFO_DATA(info) = new_buf;
        gst_buffer_unref(buffer);

        return GST_PAD_PROBE_OK;
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoEncoderTX>());
    rclcpp::shutdown();
    return 0;
}