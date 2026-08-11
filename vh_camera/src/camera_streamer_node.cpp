#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <string>
#include <queue>
#include <mutex>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <atomic>

// Um stream = uma pipeline de encode+envio independente, alimentada
// pela mesma captura. Permite testar N enc/dec com uma só câmara.
struct StreamCtx {
    int port = 0;
    GstElement *pipeline = nullptr;
    GstElement *appsrc = nullptr;
    std::queue<std::pair<uint64_t, uint64_t>> metadata_queue;
    std::mutex queue_mutex;
};

class CameraStreamerNode : public rclcpp::Node {
public:
    CameraStreamerNode() : Node("camera_streamer_node"),
                           running_(true),
                           frame_counter_(1) {

        this->declare_parameter<int>("camera_id", 2);
        this->declare_parameter<int>("width", 1920);
        this->declare_parameter<int>("height", 1080);
        this->declare_parameter<int>("fps", 30);
        this->declare_parameter<std::string>("ip_address", "127.0.0.1");
        this->declare_parameter<int>("port", 5007);
        this->declare_parameter<int>("bitrate", 80000);
        // N streams em paralelo a partir da mesma captura: portas port..port+N-1
        this->declare_parameter<int>("num_streams", 1);

        int camera_id  = this->get_parameter("camera_id").as_int();
        int width      = this->get_parameter("width").as_int();
        int height     = this->get_parameter("height").as_int();
        int fps        = this->get_parameter("fps").as_int();
        ip_address_    = this->get_parameter("ip_address").as_string();
        int base_port  = this->get_parameter("port").as_int();
        bitrate_       = this->get_parameter("bitrate").as_int();
        int num_streams = this->get_parameter("num_streams").as_int();

        gst_init(nullptr, nullptr);
        cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);

        std::string cam_pipeline =
            "v4l2src device=/dev/video" + std::to_string(camera_id) + " ! "
            "image/jpeg,width=" + std::to_string(width) + ",height=" + std::to_string(height) +
            ",framerate=" + std::to_string(fps) + "/1 ! "
            "jpegdec ! videoconvert ! video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1";

        cap_.open(cam_pipeline, cv::CAP_GSTREAMER);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao abrir a camara via GStreamer.");
            return;
        }

        for (int i = 0; i < num_streams; ++i) {
            auto ctx = std::make_unique<StreamCtx>();
            ctx->port = base_port + i;
            init_stream(ctx.get(), width, height, fps, bitrate_);
            streams_.push_back(std::move(ctx));
        }

        RCLCPP_INFO(this->get_logger(), "%d stream(s) a transmitir para %s, portas %d-%d",
                    num_streams, ip_address_.c_str(), base_port, base_port + num_streams - 1);

        capture_thread_ = std::thread(&CameraStreamerNode::capture_loop, this);
    }

    ~CameraStreamerNode() {
        running_ = false;
        if (capture_thread_.joinable()) capture_thread_.join();
        if (cap_.isOpened()) cap_.release();

        for (auto &ctx : streams_) {
            if (ctx->appsrc) gst_object_unref(ctx->appsrc);
            if (ctx->pipeline) {
                gst_element_set_state(ctx->pipeline, GST_STATE_NULL);
                gst_object_unref(ctx->pipeline);
            }
        }
    }

private:
    cv::VideoCapture cap_;
    std::thread capture_thread_;

    std::string ip_address_;
    int bitrate_;

    std::vector<std::unique_ptr<StreamCtx>> streams_;
    std::atomic<bool> running_;
    std::atomic<uint64_t> frame_counter_;

    void init_stream(StreamCtx *ctx, int width, int height, int fps, int bitrate) {
        std::string caps_str = "video/x-raw,format=BGR,width=" + std::to_string(width) +
                               ",height=" + std::to_string(height) +
                               ",framerate=" + std::to_string(fps) + "/1";

        // intra-refresh removido: sem IDR real o decoder nunca purga o DPB.
        // key-int-max=15 dá IDRs a cada 0.5s, que limpam a lista de referencias.
        std::string pipeline_str =
            "appsrc name=mysrc is-live=true do-timestamp=true format=time caps=\"" + caps_str + "\" ! "
            "videoconvert ! "
            "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
            "video/x-raw,format=I420 ! "
            "x264enc tune=zerolatency speed-preset=ultrafast sliced-threads=true threads=4 "
            "key-int-max=15 bitrate=" + std::to_string(bitrate) + " ! "
            "h264parse config-interval=1 name=parser ! "
            "video/x-h264,stream-format=byte-stream,alignment=au ! "
            "rtph264pay pt=96 mtu=1400 aggregate-mode=zero-latency ! "
            "udpsink host=" + ip_address_ + " port=" + std::to_string(ctx->port) +
            " sync=false async=false";

        GError *error = nullptr;
        ctx->pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
        if (error) {
            RCLCPP_ERROR(this->get_logger(), "Erro ao criar pipeline (porta %d): %s",
                         ctx->port, error->message);
            g_error_free(error);
            return;
        }

        ctx->appsrc = gst_bin_get_by_name(GST_BIN(ctx->pipeline), "mysrc");

        GstElement *parser = gst_bin_get_by_name(GST_BIN(ctx->pipeline), "parser");
        GstPad *src_pad = gst_element_get_static_pad(parser, "src");
        gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER,
                          (GstPadProbeCallback)pad_probe_callback, ctx, nullptr);
        gst_object_unref(src_pad);
        gst_object_unref(parser);

        gst_element_set_state(ctx->pipeline, GST_STATE_PLAYING);
    }

    void capture_loop() {
        try {
            cv::Mat frame;
            while (running_ && rclcpp::ok()) {

                cap_ >> frame;
                if (!running_) break;
                if (frame.empty()) continue;

                auto now = std::chrono::system_clock::now().time_since_epoch();
                uint64_t ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
                uint64_t current_id = frame_counter_++;

                guint size = frame.total() * frame.elemSize();

                // Mesmo frame empurrado para todas as pipelines de encode
                for (auto &ctx : streams_) {
                    if (!ctx->appsrc) continue;

                    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
                    GstMapInfo map;
                    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
                    std::memcpy(map.data, frame.data, size);
                    gst_buffer_unmap(buffer, &map);

                    GstFlowReturn ret;
                    g_signal_emit_by_name(ctx->appsrc, "push-buffer", buffer, &ret);

                    if (ret == GST_FLOW_OK) {
                        std::lock_guard<std::mutex> lock(ctx->queue_mutex);
                        ctx->metadata_queue.push({current_id, ts_ns});
                    }
                    gst_buffer_unref(buffer);
                }
            }
        } catch (...) {
        }
    }

    static GstPadProbeReturn pad_probe_callback(GstPad * /*pad*/, GstPadProbeInfo *info, gpointer user_data) {
        auto *ctx = static_cast<StreamCtx*>(user_data);
        GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);

        uint64_t frame_id = 0;
        uint64_t ts_ns = 0;

        {
            std::lock_guard<std::mutex> lock(ctx->queue_mutex);
            while (ctx->metadata_queue.size() > 2) {
                ctx->metadata_queue.pop();
            }
            if (!ctx->metadata_queue.empty()) {
                frame_id = ctx->metadata_queue.front().first;
                ts_ns = ctx->metadata_queue.front().second;
                ctx->metadata_queue.pop();
            } else {
                return GST_PAD_PROBE_OK;
            }
        }

        auto out_now = std::chrono::system_clock::now().time_since_epoch();
        uint64_t encode_ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(out_now).count();

        std::vector<uint8_t> sei_nalu;
        sei_nalu.push_back(0x00); sei_nalu.push_back(0x00); sei_nalu.push_back(0x00); sei_nalu.push_back(0x01);
        sei_nalu.push_back(0x06);
        sei_nalu.push_back(0x05);

        std::string payload = "ID:" + std::to_string(frame_id) +
                              "|TS:" + std::to_string(ts_ns) +
                              "|TS2:" + std::to_string(encode_ts_ns) + (char)0x80;
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
    rclcpp::spin(std::make_shared<CameraStreamerNode>());
    rclcpp::shutdown();
    return 0;
}