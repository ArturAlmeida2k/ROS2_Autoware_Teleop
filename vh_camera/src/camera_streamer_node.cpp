#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <string>
#include <queue>
#include <mutex>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>

class CameraStreamerNode : public rclcpp::Node {
public:
    CameraStreamerNode() : Node("camera_streamer_node"), pipeline_initialized_(false), frame_counter_(1), running_(true) {
        // ... (Declaração de parâmetros mantém-se igual) ...
        this->declare_parameter<int>("camera_id", 0);
        this->declare_parameter<int>("width", 1280);
        this->declare_parameter<int>("height", 720);
        this->declare_parameter<int>("fps", 30);
        this->declare_parameter<std::string>("ip_address", "127.0.0.1");
        this->declare_parameter<int>("port", 5007);
        this->declare_parameter<int>("bitrate", 5000);

        int camera_id = this->get_parameter("camera_id").as_int();
        int width     = this->get_parameter("width").as_int();
        int height    = this->get_parameter("height").as_int();
        int fps       = this->get_parameter("fps").as_int();
        ip_address_   = this->get_parameter("ip_address").as_string();
        port_         = this->get_parameter("port").as_int();
        bitrate_      = this->get_parameter("bitrate").as_int();

        gst_init(nullptr, nullptr);

        // 1. Configuração agressiva da Câmara (Zero Buffering)
        cap_.open(camera_id, cv::CAP_V4L2);
        if (!cap_.isOpened()) return;
        
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        cap_.set(cv::CAP_PROP_FPS, fps);
        // OBRIGATÓRIO: Forçar o Linux a guardar apenas 1 frame de cada vez
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
        // Tentar forçar MJPEG para evitar estrangulamento de USB a 1080p
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        // 2. Inicializar Pipeline Nativa
        init_gstreamer_pipeline(width, height, fps, bitrate_);

        RCLCPP_INFO(this->get_logger(), "A transmitir video e SEI para %s:%d", ip_address_.c_str(), port_);

        // 3. Lançar Thread de Captura (Lê à velocidade pura do hardware)
        capture_thread_ = std::thread(&CameraStreamerNode::capture_loop, this);
    }

    ~CameraStreamerNode() {
        running_ = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        if (cap_.isOpened()) cap_.release();
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
    }

private:
    cv::VideoCapture cap_;
    std::thread capture_thread_;
    std::atomic<bool> running_;

    std::string ip_address_;
    int port_;
    int bitrate_;

    bool pipeline_initialized_;
    GstElement *pipeline_ = nullptr;
    GstElement *appsrc_ = nullptr;
    
    std::queue<std::pair<uint64_t, uint64_t>> metadata_queue_;
    std::mutex queue_mutex_;
    uint64_t frame_counter_;

    void init_gstreamer_pipeline(int width, int height, int fps, int bitrate) {
        // ... (Mantém o mesmo código init_gstreamer_pipeline da versão anterior) ...
        std::string caps_str = "video/x-raw,format=BGR,width=" + std::to_string(width) +
                               ",height=" + std::to_string(height) + ",framerate=" + std::to_string(fps) + "/1";

        std::string pipeline_str =
            "appsrc name=mysrc is-live=true do-timestamp=true format=time caps=\"" + caps_str + "\" ! "
            "videoconvert ! "
            "queue ! "
            "video/x-raw,format=I420 ! "
            "x264enc tune=zerolatency speed-preset=ultrafast sliced-threads=true threads=2 key-int-max=15 intra-refresh=true bitrate=" + std::to_string(bitrate) + " ! "
            "h264parse config-interval=1 name=parser ! "
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

        GstElement *parser = gst_bin_get_by_name(GST_BIN(pipeline_), "parser");
        GstPad *src_pad = gst_element_get_static_pad(parser, "src");
        gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER, (GstPadProbeCallback)pad_probe_callback, this, nullptr);
        gst_object_unref(src_pad);
        gst_object_unref(parser);

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        pipeline_initialized_ = true;
    }

    // A NOVA THREAD: Bloqueia na linha "cap_ >> frame" e processa de imediato!
    void capture_loop() {
        cv::Mat frame;
        while (running_ && rclcpp::ok()) {
            cap_ >> frame; // Fica aqui parado à espera do hardware. Zero atraso!
            
            if (frame.empty()) continue;

            auto now = std::chrono::system_clock::now().time_since_epoch();
            uint64_t ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
            uint64_t current_id = frame_counter_++;

            guint size = frame.total() * frame.elemSize();
            GstBuffer *buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
            GstMapInfo map;

            gst_buffer_map(buffer, &map, GST_MAP_WRITE);
            std::memcpy(map.data, frame.data, size);
            gst_buffer_unmap(buffer, &map);

            GstFlowReturn ret;
            g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);

            if (ret == GST_FLOW_OK) {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                metadata_queue_.push({current_id, ts_ns});
            }
            gst_buffer_unref(buffer);
        }
    }

    // ... (Mantém a função pad_probe_callback igual) ...
    static GstPadProbeReturn pad_probe_callback(GstPad * /*pad*/, GstPadProbeInfo *info, gpointer user_data) {
        // ... (código existente da função de callback) ...
        auto *node = static_cast<CameraStreamerNode*>(user_data);
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
    rclcpp::spin(std::make_shared<CameraStreamerNode>());
    rclcpp::shutdown();
    return 0;
}