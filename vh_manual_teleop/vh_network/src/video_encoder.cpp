#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

using Image = sensor_msgs::msg::Image;

class VideoEncoderTX;

// Um stream = uma câmara = uma pipeline de encode independente.
struct StreamCtx {
    VideoEncoderTX *node = nullptr;
    std::string topic;
    int port = 0;

    rclcpp::Subscription<Image>::SharedPtr sub;

    GstElement *pipeline = nullptr;
    GstElement *appsrc   = nullptr;
    guint bus_watch_id   = 0;
    bool initialized     = false;

    std::queue<std::pair<uint64_t, uint64_t>> metadata_queue;
    std::mutex queue_mutex;
    uint64_t frame_counter = 1;
};

class VideoEncoderTX : public rclcpp::Node {
public:
    VideoEncoderTX() : Node("video_encoder_tx") {
        declare_parameter<std::string>("ip_address", "10.0.0.2");
        declare_parameter<int>("port", 5007);
        declare_parameter<int>("num_cameras", 1);
        declare_parameter<int>("bitrate", 5000);          
        declare_parameter<std::vector<std::string>>("camera_topics",
            std::vector<std::string>{
                "/sensing/camera/CAM_FRONT/image_raw",        
                "/sensing/camera/CAM_FRONT_LEFT/image_raw",  
                "/sensing/camera/CAM_BACK/image_raw",         
                "/sensing/camera/CAM_FRONT_RIGHT/image_raw"   
            });

        ip_address_           = get_parameter("ip_address").as_string();
        bitrate_              = get_parameter("bitrate").as_int();
        base_port_            = get_parameter("port").as_int();
        const auto topics     = get_parameter("camera_topics").as_string_array();
        int num_cameras       = get_parameter("num_cameras").as_int();

        if (num_cameras < 1) num_cameras = 1;
        if (num_cameras > static_cast<int>(topics.size())) {
            RCLCPP_WARN(get_logger(),
                        "num_cameras=%d excede os %zu tópicos configurados; a usar %zu.",
                        num_cameras, topics.size(), topics.size());
            num_cameras = static_cast<int>(topics.size());
        }

        gst_init(nullptr, nullptr);

        rclcpp::QoS qos(1);
        qos.best_effort();
        qos.keep_last(1);

        for (int i = 0; i < num_cameras; ++i) {
            auto ctx   = std::make_unique<StreamCtx>();
            ctx->node  = this;
            ctx->topic = topics[i];
            ctx->port  = base_port_ + i;

            StreamCtx *raw = ctx.get();
            ctx->sub = create_subscription<Image>(
                ctx->topic, qos,
                [this, raw](const Image::SharedPtr msg) { image_callback(msg, raw); });

            RCLCPP_INFO(get_logger(), "%s -> %s:%d",
                        ctx->topic.c_str(), ip_address_.c_str(), ctx->port);

            streams_.push_back(std::move(ctx));
        }

        RCLCPP_INFO(get_logger(), "Encoder iniciado: %d câmara(s), %d kbit/s.",
                    num_cameras, bitrate_);
    }

    ~VideoEncoderTX() override {
        for (auto &ctx : streams_) {
            if (ctx->bus_watch_id > 0) g_source_remove(ctx->bus_watch_id);
            if (ctx->appsrc)   gst_object_unref(ctx->appsrc);
            if (ctx->pipeline) {
                gst_element_set_state(ctx->pipeline, GST_STATE_NULL);
                gst_object_unref(ctx->pipeline);
            }
        }
    }

private:
    std::string ip_address_;
    int bitrate_ = 5000;
    std::vector<std::unique_ptr<StreamCtx>> streams_;

    // -----------------------------------------------------------------
    void image_callback(const Image::SharedPtr msg, StreamCtx *ctx) {
        try {
            cv_bridge::CvImagePtr cv_ptr =
                cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;

            if (frame.empty()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "Frame vazia em %s. A ignorar.", ctx->topic.c_str());
                return;
            }

            if (!ctx->initialized) {
                init_pipeline(ctx, frame.cols, frame.rows);
                if (!ctx->initialized) return;
            }

            const auto now = std::chrono::system_clock::now().time_since_epoch();
            const uint64_t ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
            const uint64_t frame_id = ctx->frame_counter++;

            const guint size = frame.total() * frame.elemSize();
            GstBuffer *buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
            GstMapInfo map;
            gst_buffer_map(buffer, &map, GST_MAP_WRITE);
            std::memcpy(map.data, frame.data, size);
            gst_buffer_unmap(buffer, &map);

            GstFlowReturn ret;
            g_signal_emit_by_name(ctx->appsrc, "push-buffer", buffer, &ret);

            if (ret == GST_FLOW_OK) {
                std::lock_guard<std::mutex> lock(ctx->queue_mutex);
                ctx->metadata_queue.push({frame_id, ts_ns});
            } else {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "GStreamer rejeitou o buffer em %s (erro %d)",
                                     ctx->topic.c_str(), ret);
            }
            gst_buffer_unref(buffer);

        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge (%s): %s", ctx->topic.c_str(), e.what());
        }
    }

    // -----------------------------------------------------------------
    void init_pipeline(StreamCtx *ctx, int width, int height) {
        const std::string caps =
            "video/x-raw,format=BGR,width=" + std::to_string(width) +
            ",height=" + std::to_string(height);

        const std::string pipeline_str =
            "appsrc name=mysrc is-live=true do-timestamp=true format=time caps=\"" + caps + "\" ! "
            "videoconvert ! "
            "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
            "video/x-raw,format=I420 ! "
            "x264enc tune=zerolatency speed-preset=ultrafast sliced-threads=true threads=4 "
            "key-int-max=15 bitrate=" + std::to_string(bitrate_) + " ! "
            "h264parse config-interval=1 name=parser ! "
            "video/x-h264,stream-format=byte-stream,alignment=au ! "
            "rtph264pay pt=96 mtu=1400 aggregate-mode=zero-latency ! "
            "udpsink host=" + ip_address_ + " port=" + std::to_string(ctx->port) +
            " sync=false async=false";

        GError *error = nullptr;
        ctx->pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
        if (error) {
            RCLCPP_ERROR(get_logger(), "Erro ao criar pipeline (porta %d): %s",
                         ctx->port, error->message);
            g_error_free(error);
            return;
        }

        ctx->appsrc = gst_bin_get_by_name(GST_BIN(ctx->pipeline), "mysrc");

        if (ctx->port == base_port_) {
            GstElement *parser = gst_bin_get_by_name(GST_BIN(ctx->pipeline), "parser");
            GstPad *src_pad = gst_element_get_static_pad(parser, "src");
            gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER,
                            (GstPadProbeCallback)pad_probe_callback, ctx, nullptr);
            gst_object_unref(src_pad);
            gst_object_unref(parser);
        }

        GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(ctx->pipeline));
        ctx->bus_watch_id = gst_bus_add_watch(bus, (GstBusFunc)bus_callback, ctx);
        gst_object_unref(bus);

        if (gst_element_set_state(ctx->pipeline, GST_STATE_PLAYING) ==
            GST_STATE_CHANGE_FAILURE) {
            RCLCPP_ERROR(get_logger(), "A pipeline da porta %d recusou-se a iniciar.",
                         ctx->port);
            return;
        }

        RCLCPP_INFO(get_logger(), "Pipeline ativa: %s %dx%d -> %s:%d",
                    ctx->topic.c_str(), width, height, ip_address_.c_str(), ctx->port);
        ctx->initialized = true;
    }

    // -----------------------------------------------------------------
    static gboolean bus_callback(GstBus * /*bus*/, GstMessage *msg, gpointer user_data) {
        auto *ctx = static_cast<StreamCtx *>(user_data);
        GError *err = nullptr;
        gchar *debug = nullptr;

        switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR:
                gst_message_parse_error(msg, &err, &debug);
                RCLCPP_ERROR(ctx->node->get_logger(), "GStreamer (porta %d): %s (%s)",
                             ctx->port, err->message, debug ? debug : "n/a");
                g_error_free(err);
                g_free(debug);
                break;
            case GST_MESSAGE_WARNING:
                gst_message_parse_warning(msg, &err, &debug);
                RCLCPP_WARN(ctx->node->get_logger(), "GStreamer (porta %d): %s (%s)",
                            ctx->port, err->message, debug ? debug : "n/a");
                g_error_free(err);
                g_free(debug);
                break;
            default:
                break;
        }
        return TRUE;
    }

    // -----------------------------------------------------------------
    // Insere o SEI com ID, timestamp de captura e timestamp de saída do encoder.
    // -----------------------------------------------------------------
    static GstPadProbeReturn pad_probe_callback(GstPad * /*pad*/, GstPadProbeInfo *info,
                                                gpointer user_data) {
        auto *ctx = static_cast<StreamCtx *>(user_data);
        GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);

        uint64_t frame_id = 0;
        uint64_t ts_ns = 0;

        {
            std::lock_guard<std::mutex> lock(ctx->queue_mutex);
            // Se o encoder descartou frames, a fila desalinha-se de forma
            // permanente. Purgar mantém o carimbo associado ao frame certo.
            while (ctx->metadata_queue.size() > 2) ctx->metadata_queue.pop();

            if (ctx->metadata_queue.empty()) return GST_PAD_PROBE_OK;

            frame_id = ctx->metadata_queue.front().first;
            ts_ns    = ctx->metadata_queue.front().second;
            ctx->metadata_queue.pop();
        }

        const auto out_now = std::chrono::system_clock::now().time_since_epoch();
        const uint64_t encode_ts_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(out_now).count();

        std::vector<uint8_t> sei;
        sei.push_back(0x00); sei.push_back(0x00); sei.push_back(0x00); sei.push_back(0x01);
        sei.push_back(0x06);   // nal_unit_type = SEI
        sei.push_back(0x05);   // payload_type  = user_data_unregistered

        const std::string payload =
            "ID:" + std::to_string(frame_id) +
            "|TS:" + std::to_string(ts_ns) +
            "|TS2:" + std::to_string(encode_ts_ns) + static_cast<char>(0x80);

        size_t remaining = 16 + payload.length();
        while (remaining >= 255) { sei.push_back(0xFF); remaining -= 255; }
        sei.push_back(static_cast<uint8_t>(remaining));

        const uint8_t uuid[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
                                  0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11};
        sei.insert(sei.end(), uuid, uuid + 16);
        sei.insert(sei.end(), payload.begin(), payload.end());

        GstMapInfo old_map;
        gst_buffer_map(buffer, &old_map, GST_MAP_READ);

        GstBuffer *new_buf =
            gst_buffer_new_allocate(nullptr, sei.size() + old_map.size, nullptr);
        GstMapInfo new_map;
        gst_buffer_map(new_buf, &new_map, GST_MAP_WRITE);
        std::memcpy(new_map.data, sei.data(), sei.size());
        std::memcpy(new_map.data + sei.size(), old_map.data, old_map.size);
        gst_buffer_unmap(new_buf, &new_map);
        gst_buffer_unmap(buffer, &old_map);

        gst_buffer_copy_into(new_buf, buffer, GST_BUFFER_COPY_METADATA, 0, -1);
        GST_PAD_PROBE_INFO_DATA(info) = new_buf;
        gst_buffer_unref(buffer);

        return GST_PAD_PROBE_OK;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    // MultiThreadedExecutor: com várias câmaras, os callbacks não bloqueiam
    // uns aos outros à espera do encode.
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<VideoEncoderTX>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}