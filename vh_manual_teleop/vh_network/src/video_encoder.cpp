#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int8.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include "teleop_msgs/msg/command_enums.hpp"
#include "teleop_msgs/msg/node_metrics.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using Image    = sensor_msgs::msg::Image;
using Int8     = std_msgs::msg::Int8;
using CmdEnums = teleop_msgs::msg::CommandEnums;
using Metrics  = teleop_msgs::msg::NodeMetrics;

static constexpr int OUT_W = 1280;
static constexpr int OUT_H = 720;

static inline uint64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

class VideoEncoderTX;

// Instantes (relógio do VH, ns) de um frame dentro do encoder.
struct FrameTimes {
    uint64_t id = 0;
    uint64_t rx = 0;       // entrada no callback ROS (TS do SEI)
    uint64_t pre = 0;      // redimensionado e convertido para I420
    uint64_t enc_in = 0;   // entrada no x264enc
    uint64_t enc_out = 0;  // saída do x264enc
};

// Um stream = uma câmara = uma pipeline de encode independente.
struct StreamCtx {
    VideoEncoderTX *node = nullptr;
    std::string topic;
    int port = 0;
    bool instrumented = false;   // só a câmara frontal leva SEI e métricas

    rclcpp::Subscription<Image>::SharedPtr sub;
    GstElement *pipeline = nullptr;
    GstElement *appsrc   = nullptr;
    bool initialized     = false;
    uint64_t frame_counter = 1;

    // Seguimento de cada frame: até ao encoder pelo PTS (que a queue leaky
    // não desalinha quando descarta frames); depois do encoder pela ordem,
    // porque o GstVideoEncoder altera o PTS e o x264 em zerolatency produz
    // exatamente um frame por cada frame de entrada, pela mesma ordem.
    std::mutex mtx;
    std::deque<FrameTimes> pending;               // entregues ao appsrc, sem PTS
    std::map<GstClockTime, FrameTimes> by_pts;    // appsrc -> entrada do encoder
    std::deque<FrameTimes> in_enc, after_enc;     // dentro / depois do encoder
};

class VideoEncoderTX : public rclcpp::Node {
public:
    VideoEncoderTX() : Node("video_encoder_tx") {
        declare_parameter<std::string>("ip_address", "10.0.0.2");
        declare_parameter<int>("port", 5007);
        declare_parameter<int>("num_cameras", 1);
        declare_parameter<int>("bitrate", 5000);
        // Intra-refresh: em vez de um keyframe inteiro a cada key-int-max frames,
        // o x264 renova a imagem por colunas ao longo desses frames. Os frames
        // ficam com tamanho mais uniforme e os picos dos keyframes desaparecem.
        declare_parameter<bool>("intra_refresh", false);
        declare_parameter<std::vector<std::string>>("camera_topics",
            std::vector<std::string>{
                "/sensing/camera/CAM_FRONT/image_raw",
                "/sensing/camera/CAM_FRONT_LEFT/image_raw",
                "/sensing/camera/CAM_BACK/image_raw",
                "/sensing/camera/CAM_FRONT_RIGHT/image_raw"});

        ip_address_       = get_parameter("ip_address").as_string();
        bitrate_          = get_parameter("bitrate").as_int();
        intra_refresh_    = get_parameter("intra_refresh").as_bool();
        const int port    = get_parameter("port").as_int();
        const auto topics = get_parameter("camera_topics").as_string_array();
        const int n = std::clamp(static_cast<int>(get_parameter("num_cameras").as_int()),
                                 1, static_cast<int>(topics.size()));

        gst_init(nullptr, nullptr);

        rclcpp::QoS mq(10);
        mq.best_effort().durability_volatile();
        pub_preprocess_ = create_publisher<Metrics>("/metrics/video_encoder/preprocess", mq);
        pub_x264_       = create_publisher<Metrics>("/metrics/video_encoder/x264", mq);
        pub_total_      = create_publisher<Metrics>("/metrics/video_encoder/total", mq);

        sub_mode_ = create_subscription<Int8>("/teleop/uplink_mode", 10,
            [this](const Int8::SharedPtr msg) {
                active_.store(msg->data == CmdEnums::UPLINK_VIDEO, std::memory_order_relaxed);
            });

        rclcpp::QoS qos(1);
        qos.best_effort().keep_last(1);
        for (int i = 0; i < n; ++i) {
            auto ctx = std::make_unique<StreamCtx>();
            ctx->node = this;
            ctx->topic = topics[i];
            ctx->port = port + i;
            ctx->instrumented = (i == 0);
            StreamCtx *raw = ctx.get();
            ctx->sub = create_subscription<Image>(ctx->topic, qos,
                [this, raw](const Image::SharedPtr msg) { image_callback(msg, raw); });
            RCLCPP_INFO(get_logger(), "%s -> %s:%d", ctx->topic.c_str(), ip_address_.c_str(), ctx->port);
            streams_.push_back(std::move(ctx));
        }
        RCLCPP_INFO(get_logger(), "Encoder iniciado: %d câmara(s), %d kbit/s, intra-refresh %s.",
                    n, bitrate_, intra_refresh_ ? "on" : "off");
    }

    ~VideoEncoderTX() override {
        for (auto &ctx : streams_) {
            if (ctx->appsrc) gst_object_unref(ctx->appsrc);
            if (ctx->pipeline) {
                gst_element_set_state(ctx->pipeline, GST_STATE_NULL);
                gst_object_unref(ctx->pipeline);
            }
        }
    }

    void publish_metrics(const FrameTimes &f, uint64_t out) {
        auto pub = [&](const rclcpp::Publisher<Metrics>::SharedPtr &p, uint64_t a, uint64_t b) {
            if (a == 0 || b < a) return;
            auto m = std::make_unique<Metrics>();
            m->id = static_cast<uint32_t>(f.id);
            m->tx.sec = static_cast<int32_t>(a / 1000000000ULL);
            m->tx.nanosec = static_cast<uint32_t>(a % 1000000000ULL);
            m->rx.sec = static_cast<int32_t>(b / 1000000000ULL);
            m->rx.nanosec = static_cast<uint32_t>(b % 1000000000ULL);
            m->latency_ms = (b - a) / 1e6;
            p->publish(std::move(m));
        };
        pub(pub_preprocess_, f.rx, f.pre);
        pub(pub_x264_, f.enc_in, f.enc_out);
        pub(pub_total_, f.rx, out);
    }

private:
    std::string ip_address_;
    int bitrate_ = 5000;
    bool intra_refresh_ = false;
    std::vector<std::unique_ptr<StreamCtx>> streams_;
    rclcpp::Publisher<Metrics>::SharedPtr pub_preprocess_, pub_x264_, pub_total_;
    rclcpp::Subscription<Int8>::SharedPtr sub_mode_;
    std::atomic<bool> active_{true};

    // -----------------------------------------------------------------
    void image_callback(const Image::SharedPtr msg, StreamCtx *ctx) {
        if (!active_.load(std::memory_order_relaxed)) return;

        FrameTimes ft;
        ft.rx = now_ns();

        try {
            // Usa a imagem do ROS sem cópia quando o formato é conhecido.
            int code = -1;
            const std::string &enc = msg->encoding;
            if      (enc == sensor_msgs::image_encodings::BGR8)  code = cv::COLOR_BGR2YUV_I420;
            else if (enc == sensor_msgs::image_encodings::RGB8)  code = cv::COLOR_RGB2YUV_I420;
            else if (enc == sensor_msgs::image_encodings::BGRA8) code = cv::COLOR_BGRA2YUV_I420;
            else if (enc == sensor_msgs::image_encodings::RGBA8) code = cv::COLOR_RGBA2YUV_I420;

            cv_bridge::CvImageConstPtr cv_ptr;
            if (code >= 0) {
                cv_ptr = cv_bridge::toCvShare(msg);
            } else {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                code = cv::COLOR_BGR2YUV_I420;
            }
            const cv::Mat &src = cv_ptr->image;
            if (src.empty()) return;

            if (!ctx->initialized) {
                init_pipeline(ctx, src.cols, src.rows);
                if (!ctx->initialized) return;
                ft.rx = now_ns();   // não contar o arranque da pipeline no 1.º frame
            }

            // Redimensiona e converte para I420 diretamente no buffer do GStreamer.
            cv::Mat scaled = src;
            if (src.cols != OUT_W || src.rows != OUT_H)
                cv::resize(src, scaled, cv::Size(OUT_W, OUT_H), 0, 0, cv::INTER_LINEAR);

            GstBuffer *buffer = gst_buffer_new_allocate(nullptr, OUT_W * OUT_H * 3 / 2, nullptr);
            GstMapInfo map;
            gst_buffer_map(buffer, &map, GST_MAP_WRITE);
            cv::Mat yuv(OUT_H * 3 / 2, OUT_W, CV_8UC1, map.data);
            cv::cvtColor(scaled, yuv, code);
            gst_buffer_unmap(buffer, &map);

            ft.pre = now_ns();
            ft.id = ctx->frame_counter++;

            // Registar antes do push: a thread do appsrc pode empurrar o buffer
            // para a pipeline antes de o push-buffer retornar.
            if (ctx->instrumented) {
                std::lock_guard<std::mutex> lock(ctx->mtx);
                ctx->pending.push_back(ft);
            }

            GstFlowReturn ret;
            g_signal_emit_by_name(ctx->appsrc, "push-buffer", buffer, &ret);
            gst_buffer_unref(buffer);

            if (ret != GST_FLOW_OK) {
                if (ctx->instrumented) {
                    std::lock_guard<std::mutex> lock(ctx->mtx);
                    if (!ctx->pending.empty() && ctx->pending.back().id == ft.id)
                        ctx->pending.pop_back();
                }
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "GStreamer rejeitou o buffer em %s (erro %d)", ctx->topic.c_str(), ret);
            }
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge (%s): %s", ctx->topic.c_str(), e.what());
        } catch (const cv::Exception &e) {
            RCLCPP_ERROR(get_logger(), "OpenCV (%s): %s", ctx->topic.c_str(), e.what());
        }
    }

    // -----------------------------------------------------------------
    void init_pipeline(StreamCtx *ctx, int width, int height) {
        const std::string pipeline_str =
            "appsrc name=mysrc is-live=true do-timestamp=true format=time "
            "caps=\"video/x-raw,format=I420,width=" + std::to_string(OUT_W) +
            ",height=" + std::to_string(OUT_H) + ",framerate=30/1\" ! "
            "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
            "x264enc name=enc tune=zerolatency speed-preset=ultrafast sliced-threads=true threads=4 "
            "key-int-max=15 bitrate=" + std::to_string(bitrate_) +
            (intra_refresh_ ? " intra-refresh=true" : "") + " ! "
            "h264parse config-interval=1 name=parser ! "
            "video/x-h264,stream-format=byte-stream,alignment=au ! "
            "rtph264pay pt=96 mtu=1400 aggregate-mode=zero-latency ! "
            "udpsink host=" + ip_address_ + " port=" + std::to_string(ctx->port) +
            " sync=false async=false";

        GError *error = nullptr;
        ctx->pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
        if (error) {
            RCLCPP_ERROR(get_logger(), "Erro ao criar pipeline (porta %d): %s", ctx->port, error->message);
            g_error_free(error);
            return;
        }
        ctx->appsrc = gst_bin_get_by_name(GST_BIN(ctx->pipeline), "mysrc");

        if (ctx->instrumented) {
            add_probe(ctx, "mysrc",  "src",  probe_appsrc);
            add_probe(ctx, "enc",    "sink", probe_enc_in);
            add_probe(ctx, "enc",    "src",  probe_enc_out);
            add_probe(ctx, "parser", "src",  probe_parser_out);
        }

        if (gst_element_set_state(ctx->pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            RCLCPP_ERROR(get_logger(), "A pipeline da porta %d recusou-se a iniciar.", ctx->port);
            return;
        }
        RCLCPP_INFO(get_logger(), "Pipeline ativa: %s %dx%d -> %dx%d -> %s:%d",
                    ctx->topic.c_str(), width, height, OUT_W, OUT_H, ip_address_.c_str(), ctx->port);
        ctx->initialized = true;
    }

    static void add_probe(StreamCtx *ctx, const char *elem, const char *pad, GstPadProbeCallback cb) {
        GstElement *e = gst_bin_get_by_name(GST_BIN(ctx->pipeline), elem);
        GstPad *p = gst_element_get_static_pad(e, pad);
        gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, cb, ctx, nullptr);
        gst_object_unref(p);
        gst_object_unref(e);
    }

    // -----------------------------------------------------------------
    // appsrc entrega o buffer: associa o registo ao PTS.
    static GstPadProbeReturn probe_appsrc(GstPad *, GstPadProbeInfo *info, gpointer ud) {
        auto *ctx = static_cast<StreamCtx *>(ud);
        const GstClockTime pts = GST_BUFFER_PTS(GST_PAD_PROBE_INFO_BUFFER(info));
        std::lock_guard<std::mutex> lock(ctx->mtx);
        if (ctx->pending.empty()) return GST_PAD_PROBE_OK;
        ctx->by_pts[pts] = ctx->pending.front();
        ctx->pending.pop_front();
        // Frames descartados pela queue leaky nunca chegam ao encoder.
        while (ctx->by_pts.size() > 16) ctx->by_pts.erase(ctx->by_pts.begin());
        return GST_PAD_PROBE_OK;
    }

    static GstPadProbeReturn probe_enc_in(GstPad *, GstPadProbeInfo *info, gpointer ud) {
        auto *ctx = static_cast<StreamCtx *>(ud);
        const GstClockTime pts = GST_BUFFER_PTS(GST_PAD_PROBE_INFO_BUFFER(info));
        const uint64_t t = now_ns();
        std::lock_guard<std::mutex> lock(ctx->mtx);
        FrameTimes ft;                        // sem registo: entra vazio para manter a ordem
        auto it = ctx->by_pts.find(pts);
        if (it != ctx->by_pts.end()) { ft = it->second; ctx->by_pts.erase(it); }
        ft.enc_in = t;
        ctx->in_enc.push_back(ft);
        while (ctx->in_enc.size() > 8) ctx->in_enc.pop_front();
        return GST_PAD_PROBE_OK;
    }

    static GstPadProbeReturn probe_enc_out(GstPad *, GstPadProbeInfo *, gpointer ud) {
        auto *ctx = static_cast<StreamCtx *>(ud);
        const uint64_t t = now_ns();
        std::lock_guard<std::mutex> lock(ctx->mtx);
        if (ctx->in_enc.empty()) return GST_PAD_PROBE_OK;
        FrameTimes ft = ctx->in_enc.front();
        ctx->in_enc.pop_front();
        ft.enc_out = t;
        ctx->after_enc.push_back(ft);
        while (ctx->after_enc.size() > 8) ctx->after_enc.pop_front();
        return GST_PAD_PROBE_OK;
    }

    // Saída do h264parse: insere o SEI com ID, TS (chegada da imagem) e
    // TS2 (saída do encoder), e publica as métricas do frame.
    static GstPadProbeReturn probe_parser_out(GstPad *, GstPadProbeInfo *info, gpointer ud) {
        auto *ctx = static_cast<StreamCtx *>(ud);
        GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
        const uint64_t ts2 = now_ns();

        FrameTimes ft;
        {
            std::lock_guard<std::mutex> lock(ctx->mtx);
            if (ctx->after_enc.empty()) return GST_PAD_PROBE_OK;
            ft = ctx->after_enc.front();
            ctx->after_enc.pop_front();
        }
        if (ft.id == 0) return GST_PAD_PROBE_OK;

        const std::string payload =
            "ID:" + std::to_string(ft.id) + "|TS:" + std::to_string(ft.rx) +
            "|TS2:" + std::to_string(ts2) + static_cast<char>(0x80);
        const uint8_t uuid[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
                                  0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11};

        std::vector<uint8_t> sei = {0x00, 0x00, 0x00, 0x01,
                                    0x06,    // nal_unit_type = SEI
                                    0x05};   // payload_type  = user_data_unregistered
        size_t remaining = 16 + payload.size();
        while (remaining >= 255) { sei.push_back(0xFF); remaining -= 255; }
        sei.push_back(static_cast<uint8_t>(remaining));
        sei.insert(sei.end(), uuid, uuid + 16);
        sei.insert(sei.end(), payload.begin(), payload.end());

        GstMapInfo old_map, new_map;
        gst_buffer_map(buffer, &old_map, GST_MAP_READ);
        GstBuffer *new_buf = gst_buffer_new_allocate(nullptr, sei.size() + old_map.size, nullptr);
        gst_buffer_map(new_buf, &new_map, GST_MAP_WRITE);
        std::memcpy(new_map.data, sei.data(), sei.size());
        std::memcpy(new_map.data + sei.size(), old_map.data, old_map.size);
        gst_buffer_unmap(new_buf, &new_map);
        gst_buffer_unmap(buffer, &old_map);

        gst_buffer_copy_into(new_buf, buffer, GST_BUFFER_COPY_METADATA, 0, -1);
        GST_PAD_PROBE_INFO_DATA(info) = new_buf;
        gst_buffer_unref(buffer);

        ctx->node->publish_metrics(ft, ts2);
        return GST_PAD_PROBE_OK;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<VideoEncoderTX>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}