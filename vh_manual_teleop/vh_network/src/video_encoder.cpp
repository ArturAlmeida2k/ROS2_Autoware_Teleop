#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int8.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include "teleop_msgs/msg/command_enums.hpp"
#include "teleop_msgs/msg/node_metrics.hpp"

#include <atomic>
#include <chrono>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using Image = sensor_msgs::msg::Image;
using Int8 = std_msgs::msg::Int8;
using CmdEnums = teleop_msgs::msg::CommandEnums;
using Metrics = teleop_msgs::msg::NodeMetrics;

class VideoEncoderTX;

static inline uint64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

// Instantes (relógio do VH, ns) por onde um frame passa dentro do encoder.
// 0 = etapa que não existe no modo de pré-processamento em uso.
struct FrameTimes {
    uint64_t id      = 0;
    uint64_t rx      = 0;  // entrada no callback ROS
    uint64_t mat     = 0;  // imagem disponível em cv::Mat (cv_bridge) — é o TS do SEI
    uint64_t pre     = 0;  // pré-processamento feito, buffer pronto para o appsrc
    uint64_t src     = 0;  // appsrc entrega o buffer à pipeline
    uint64_t conv    = 0;  // saída do videoconvert   (só modo gstreamer)
    uint64_t scale   = 0;  // saída do videoscale     (só modo gstreamer)
    uint64_t enc_in  = 0;  // entrada no x264enc (depois da queue)
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
    guint bus_watch_id   = 0;
    bool initialized     = false;
    int out_w = 1280, out_h = 720;

    // Seguimento de cada frame ao longo da pipeline:
    //  - pending:   entregues ao appsrc, ainda sem PTS (FIFO: o appsrc não
    //               reordena nem descarta);
    //  - by_pts:    do appsrc até à entrada do x264enc, com o PTS como chave.
    //               O PTS atravessa videoconvert/videoscale/queue inalterado,
    //               e assim os frames que a queue leaky descarta não
    //               desalinham os restantes;
    //  - in_enc / after_enc: depois do encoder o PTS muda (o GstVideoEncoder
    //               soma-lhe um desvio de 1000 h), por isso segue-se a ordem.
    //               O x264 em zerolatency produz exatamente um frame por
    //               cada frame de entrada, pela mesma ordem.
    std::mutex times_mutex;
    std::deque<FrameTimes> pending;
    std::map<GstClockTime, FrameTimes> by_pts;
    std::deque<FrameTimes> in_enc;
    std::deque<FrameTimes> after_enc;
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
        // "opencv":    redimensiona e converte para I420 em OpenCV, diretamente
        //              para o buffer do GStreamer (rápido, multi-thread/SIMD).
        // "gstreamer": pipeline antiga (videoconvert + videoscale), para comparar.
        declare_parameter<std::string>("preprocess", "opencv");
        declare_parameter<int>("convert_threads", 4);   // videoconvert, modo gstreamer
        declare_parameter<int>("out_width", 1280);
        declare_parameter<int>("out_height", 720);
        declare_parameter<bool>("stage_metrics", true);

        ip_address_       = get_parameter("ip_address").as_string();
        bitrate_          = get_parameter("bitrate").as_int();
        base_port_        = get_parameter("port").as_int();
        preprocess_       = get_parameter("preprocess").as_string();
        convert_threads_  = get_parameter("convert_threads").as_int();
        out_w_            = get_parameter("out_width").as_int();
        out_h_            = get_parameter("out_height").as_int();
        stage_metrics_    = get_parameter("stage_metrics").as_bool();
        const auto topics = get_parameter("camera_topics").as_string_array();
        int num_cameras   = get_parameter("num_cameras").as_int();

        if (preprocess_ != "opencv" && preprocess_ != "gstreamer") {
            RCLCPP_WARN(get_logger(), "preprocess='%s' desconhecido — a usar 'opencv'.",
                        preprocess_.c_str());
            preprocess_ = "opencv";
        }
        // I420 exige dimensões pares.
        out_w_ &= ~1;
        out_h_ &= ~1;

        if (num_cameras < 1) num_cameras = 1;
        if (num_cameras > static_cast<int>(topics.size())) {
            RCLCPP_WARN(get_logger(),
                        "num_cameras=%d excede os %zu tópicos configurados; a usar %zu.",
                        num_cameras, topics.size(), topics.size());
            num_cameras = static_cast<int>(topics.size());
        }

        gst_init(nullptr, nullptr);

        if (stage_metrics_) {
            rclcpp::QoS mq(10);
            mq.best_effort();
            mq.durability_volatile();
            for (const char *s : {"ros_to_mat", "preprocess", "handoff", "convert", "scale",
                                  "queue", "x264", "parse", "total"}) {
                stage_pubs_[s] = create_publisher<Metrics>(
                    std::string("/metrics/video_encoder/") + s, mq);
            }
        }

        rclcpp::QoS qos(1);
        qos.best_effort();
        qos.keep_last(1);

        sub_mode_ = create_subscription<Int8>(
            "/teleop/uplink_mode", 10,
            [this](const Int8::SharedPtr msg) {
                active_.store(msg->data == CmdEnums::UPLINK_VIDEO, std::memory_order_relaxed);
            });

        for (int i = 0; i < num_cameras; ++i) {
            auto ctx   = std::make_unique<StreamCtx>();
            ctx->node  = this;
            ctx->topic = topics[i];
            ctx->port  = base_port_ + i;
            ctx->instrumented = (i == 0);
            ctx->out_w = out_w_;
            ctx->out_h = out_h_;

            StreamCtx *raw = ctx.get();
            ctx->sub = create_subscription<Image>(
                ctx->topic, qos,
                [this, raw](const Image::SharedPtr msg) { image_callback(msg, raw); });

            RCLCPP_INFO(get_logger(), "%s -> %s:%d",
                        ctx->topic.c_str(), ip_address_.c_str(), ctx->port);

            streams_.push_back(std::move(ctx));
        }

        RCLCPP_INFO(get_logger(), "Encoder iniciado: %d câmara(s), %d kbit/s, %dx%d, pré-processamento=%s.",
                    num_cameras, bitrate_, out_w_, out_h_, preprocess_.c_str());
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

    // Publica a duração de cada etapa de um frame (só câmara frontal).
    void publish_stages(const FrameTimes &f, uint64_t out) {
        if (!stage_metrics_) return;
        auto pub = [&](const char *name, uint64_t a, uint64_t b) {
            if (a == 0 || b == 0 || b < a) return;
            auto m = std::make_unique<Metrics>();
            m->id = static_cast<uint32_t>(f.id);
            m->tx.sec = static_cast<int32_t>(a / 1000000000ULL);
            m->tx.nanosec = static_cast<uint32_t>(a % 1000000000ULL);
            m->rx.sec = static_cast<int32_t>(b / 1000000000ULL);
            m->rx.nanosec = static_cast<uint32_t>(b % 1000000000ULL);
            m->latency_ms = (b - a) / 1e6;
            stage_pubs_.at(name)->publish(std::move(m));
        };
        const bool gst = (f.conv != 0);
        pub("ros_to_mat", f.rx,  f.mat);
        pub("preprocess", f.mat, f.pre);
        pub("handoff",    f.pre, f.src);
        if (gst) {
            pub("convert", f.src,  f.conv);
            pub("scale",   f.conv, f.scale);
            pub("queue",   f.scale, f.enc_in);
        } else {
            pub("queue",   f.src, f.enc_in);
        }
        pub("x264",  f.enc_in,  f.enc_out);
        pub("parse", f.enc_out, out);
        pub("total", f.rx,      out);
    }

private:
    std::string ip_address_;
    int bitrate_ = 5000;
    int base_port_ = 5007;
    std::string preprocess_ = "opencv";
    int convert_threads_ = 4;
    int out_w_ = 1280, out_h_ = 720;
    bool stage_metrics_ = true;
    std::vector<std::unique_ptr<StreamCtx>> streams_;
    std::map<std::string, rclcpp::Publisher<Metrics>::SharedPtr> stage_pubs_;

    rclcpp::Subscription<Int8>::SharedPtr sub_mode_;
    std::atomic<bool> active_{true};

    // -----------------------------------------------------------------
    void image_callback(const Image::SharedPtr msg, StreamCtx *ctx) {
        if (!active_.load(std::memory_order_relaxed)) return;

        FrameTimes ft;
        ft.rx = now_ns();

        try {
            GstBuffer *buffer = nullptr;

            if (preprocess_ == "gstreamer") {
                // --- Caminho antigo: cópia BGR à resolução original ---
                cv_bridge::CvImagePtr cv_ptr =
                    cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                const cv::Mat &frame = cv_ptr->image;
                if (frame.empty()) return;
                ft.mat = now_ns();

                if (!ctx->initialized) {
                    init_pipeline(ctx, frame.cols, frame.rows);
                    if (!ctx->initialized) return;
                }

                const gsize size = frame.total() * frame.elemSize();
                buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
                GstMapInfo map;
                gst_buffer_map(buffer, &map, GST_MAP_WRITE);
                std::memcpy(map.data, frame.data, size);
                gst_buffer_unmap(buffer, &map);
            } else {
                // --- Caminho novo: sem cópia à entrada; redimensiona e converte
                //     para I420 em OpenCV, escrevendo diretamente no buffer ---
                int code = -1;
                const std::string &enc = msg->encoding;
                if      (enc == sensor_msgs::image_encodings::BGR8)  code = cv::COLOR_BGR2YUV_I420;
                else if (enc == sensor_msgs::image_encodings::RGB8)  code = cv::COLOR_RGB2YUV_I420;
                else if (enc == sensor_msgs::image_encodings::BGRA8) code = cv::COLOR_BGRA2YUV_I420;
                else if (enc == sensor_msgs::image_encodings::RGBA8) code = cv::COLOR_RGBA2YUV_I420;

                cv_bridge::CvImageConstPtr cv_ptr;
                if (code >= 0) {
                    cv_ptr = cv_bridge::toCvShare(msg);   // sem cópia
                } else {
                    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                    code = cv::COLOR_BGR2YUV_I420;
                }
                const cv::Mat &src = cv_ptr->image;
                if (src.empty()) return;
                ft.mat = now_ns();

                if (!ctx->initialized) {
                    init_pipeline(ctx, src.cols, src.rows);
                    if (!ctx->initialized) return;
                }

                cv::Mat scaled;
                if (src.cols != ctx->out_w || src.rows != ctx->out_h) {
                    cv::resize(src, scaled, cv::Size(ctx->out_w, ctx->out_h), 0, 0, cv::INTER_LINEAR);
                } else {
                    scaled = src;
                }

                const gsize size = static_cast<gsize>(ctx->out_w) * ctx->out_h * 3 / 2;
                buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
                GstMapInfo map;
                gst_buffer_map(buffer, &map, GST_MAP_WRITE);
                cv::Mat yuv(ctx->out_h * 3 / 2, ctx->out_w, CV_8UC1, map.data);
                cv::cvtColor(scaled, yuv, code);     // escreve no próprio buffer
                gst_buffer_unmap(buffer, &map);
            }

            ft.pre = now_ns();
            ft.id  = ctx->frame_counter++;

            // Registar ANTES do push: a thread do appsrc pode empurrar o buffer
            // para a pipeline antes de o push-buffer retornar.
            {
                std::lock_guard<std::mutex> lock(ctx->times_mutex);
                ctx->pending.push_back(ft);
            }

            GstFlowReturn ret;
            g_signal_emit_by_name(ctx->appsrc, "push-buffer", buffer, &ret);
            gst_buffer_unref(buffer);

            if (ret != GST_FLOW_OK) {
                std::lock_guard<std::mutex> lock(ctx->times_mutex);
                if (!ctx->pending.empty() && ctx->pending.back().id == ft.id)
                    ctx->pending.pop_back();
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "GStreamer rejeitou o buffer em %s (erro %d)",
                                     ctx->topic.c_str(), ret);
            }

        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge (%s): %s", ctx->topic.c_str(), e.what());
        } catch (cv::Exception &e) {
            RCLCPP_ERROR(get_logger(), "OpenCV (%s): %s", ctx->topic.c_str(), e.what());
        }
    }

    // -----------------------------------------------------------------
    void init_pipeline(StreamCtx *ctx, int width, int height) {
        const std::string fps = ",framerate=30/1";
        const std::string out_caps =
            "video/x-raw,format=I420,width=" + std::to_string(ctx->out_w) +
            ",height=" + std::to_string(ctx->out_h);

        std::string head;
        if (preprocess_ == "gstreamer") {
            const std::string caps =
                "video/x-raw,format=BGR,width=" + std::to_string(width) +
                ",height=" + std::to_string(height) + fps;
            head =
                "appsrc name=mysrc is-live=true do-timestamp=true format=time caps=\"" + caps + "\" ! "
                "videoconvert name=conv n-threads=" + std::to_string(convert_threads_) + " ! "
                "videoscale name=scale ! " + out_caps + " ! ";
        } else {
            // O buffer já chega em I420 à resolução de saída.
            head =
                "appsrc name=mysrc is-live=true do-timestamp=true format=time caps=\"" +
                out_caps + fps + "\" ! ";
        }

        const std::string pipeline_str = head +
            "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
            "x264enc name=enc tune=zerolatency speed-preset=ultrafast sliced-threads=true threads=4 "
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

        if (ctx->instrumented) {
            add_probe(ctx, "mysrc",  "src",  probe_src);
            if (preprocess_ == "gstreamer") {
                add_probe(ctx, "conv",  "src", probe_conv);
                add_probe(ctx, "scale", "src", probe_scale);
            }
            add_probe(ctx, "enc",    "sink", probe_enc_in);
            add_probe(ctx, "enc",    "src",  probe_enc_out);
            add_probe(ctx, "parser", "src",  probe_parser_out);   // SEI + métricas
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

        RCLCPP_INFO(get_logger(), "Pipeline ativa: %s %dx%d -> %dx%d -> %s:%d",
                    ctx->topic.c_str(), width, height, ctx->out_w, ctx->out_h,
                    ip_address_.c_str(), ctx->port);
        ctx->initialized = true;
    }

    static void add_probe(StreamCtx *ctx, const char *elem, const char *pad,
                          GstPadProbeCallback cb) {
        GstElement *e = gst_bin_get_by_name(GST_BIN(ctx->pipeline), elem);
        if (!e) return;
        GstPad *p = gst_element_get_static_pad(e, pad);
        if (p) {
            gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, cb, ctx, nullptr);
            gst_object_unref(p);
        }
        gst_object_unref(e);
    }

    // -----------------------------------------------------------------
    // Probes de tempo. Todas usam o PTS do buffer como chave do frame.
    // -----------------------------------------------------------------
    static GstPadProbeReturn probe_src(GstPad *, GstPadProbeInfo *info, gpointer ud) {
        auto *ctx = static_cast<StreamCtx *>(ud);
        const GstClockTime pts = GST_BUFFER_PTS(GST_PAD_PROBE_INFO_BUFFER(info));
        const uint64_t t = now_ns();
        std::lock_guard<std::mutex> lock(ctx->times_mutex);
        if (ctx->pending.empty()) return GST_PAD_PROBE_OK;
        FrameTimes ft = ctx->pending.front();
        ctx->pending.pop_front();
        ft.src = t;
        ctx->by_pts[pts] = ft;
        // Frames descartados pela queue leaky nunca chegam ao fim: limpar.
        while (ctx->by_pts.size() > 16) ctx->by_pts.erase(ctx->by_pts.begin());
        return GST_PAD_PROBE_OK;
    }

    static void stamp(StreamCtx *ctx, GstPadProbeInfo *info, uint64_t FrameTimes::*field) {
        const GstClockTime pts = GST_BUFFER_PTS(GST_PAD_PROBE_INFO_BUFFER(info));
        const uint64_t t = now_ns();
        std::lock_guard<std::mutex> lock(ctx->times_mutex);
        auto it = ctx->by_pts.find(pts);
        if (it != ctx->by_pts.end()) it->second.*field = t;
    }
    static GstPadProbeReturn probe_conv(GstPad *, GstPadProbeInfo *i, gpointer ud) {
        stamp(static_cast<StreamCtx *>(ud), i, &FrameTimes::conv);    return GST_PAD_PROBE_OK; }
    static GstPadProbeReturn probe_scale(GstPad *, GstPadProbeInfo *i, gpointer ud) {
        stamp(static_cast<StreamCtx *>(ud), i, &FrameTimes::scale);   return GST_PAD_PROBE_OK; }
    static GstPadProbeReturn probe_enc_in(GstPad *, GstPadProbeInfo *info, gpointer ud) {
        auto *ctx = static_cast<StreamCtx *>(ud);
        const GstClockTime pts = GST_BUFFER_PTS(GST_PAD_PROBE_INFO_BUFFER(info));
        const uint64_t t = now_ns();
        std::lock_guard<std::mutex> lock(ctx->times_mutex);
        FrameTimes ft;                       // frame sem registo: entra vazio
        auto it = ctx->by_pts.find(pts);     // para manter a ordem alinhada
        if (it != ctx->by_pts.end()) { ft = it->second; ctx->by_pts.erase(it); }
        ft.enc_in = t;
        ctx->in_enc.push_back(ft);
        while (ctx->in_enc.size() > 8) ctx->in_enc.pop_front();
        return GST_PAD_PROBE_OK;
    }
    static GstPadProbeReturn probe_enc_out(GstPad *, GstPadProbeInfo *, gpointer ud) {
        auto *ctx = static_cast<StreamCtx *>(ud);
        const uint64_t t = now_ns();
        std::lock_guard<std::mutex> lock(ctx->times_mutex);
        if (ctx->in_enc.empty()) return GST_PAD_PROBE_OK;
        FrameTimes ft = ctx->in_enc.front();
        ctx->in_enc.pop_front();
        ft.enc_out = t;
        ctx->after_enc.push_back(ft);
        while (ctx->after_enc.size() > 8) ctx->after_enc.pop_front();
        return GST_PAD_PROBE_OK;
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
    // Saída do h264parse: insere o SEI com ID, timestamp de captura (TS) e
    // timestamp de saída do encoder (TS2), e publica as etapas do frame.
    // -----------------------------------------------------------------
    static GstPadProbeReturn probe_parser_out(GstPad * /*pad*/, GstPadProbeInfo *info,
                                              gpointer user_data) {
        auto *ctx = static_cast<StreamCtx *>(user_data);
        GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
        const uint64_t encode_ts_ns = now_ns();

        FrameTimes ft;
        {
            std::lock_guard<std::mutex> lock(ctx->times_mutex);
            if (ctx->after_enc.empty()) return GST_PAD_PROBE_OK;
            ft = ctx->after_enc.front();
            ctx->after_enc.pop_front();
        }
        if (ft.id == 0) return GST_PAD_PROBE_OK;   // frame sem registo

        std::vector<uint8_t> sei;
        sei.push_back(0x00); sei.push_back(0x00); sei.push_back(0x00); sei.push_back(0x01);
        sei.push_back(0x06);   // nal_unit_type = SEI
        sei.push_back(0x05);   // payload_type  = user_data_unregistered

        const std::string payload =
            "ID:" + std::to_string(ft.id) +
            "|TS:" + std::to_string(ft.mat) +
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

        ctx->node->publish_stages(ft, encode_ts_ns);
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