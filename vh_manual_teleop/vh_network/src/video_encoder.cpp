#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <string>
#include <queue>
#include <mutex>
#include <vector>

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
        RCLCPP_INFO(this->get_logger(), "Encoder Nativo iniciado. A aguardar imagens... (Destino: %s:%d)", 
                    ip_address_.c_str(), port_);
    }

    ~VideoEncoderTX() {
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
    }

private:
    std::string ip_address_;
    int port_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    
    // GStreamer nativo
    bool pipeline_initialized_;
    GstElement *pipeline_ = nullptr;
    GstElement *appsrc_ = nullptr;
    
    // Fila para emparelhar as frames que entram com os metadados
    std::queue<std::pair<uint64_t, uint64_t>> metadata_queue_;
    std::mutex queue_mutex_;
    uint64_t frame_counter_;
    GstClockTime gst_timestamp_ = 0;

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

            // 1. Calcular o timestamp real em nanosegundos
            uint64_t ts_ns = msg->header.stamp.sec * 1000000000ULL + msg->header.stamp.nanosec;
            uint64_t current_id = frame_counter_++;

            // 2. Guardar na fila para a sonda injetar depois da compressão
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                metadata_queue_.push({current_id, ts_ns});
            }

            // 3. Converter cv::Mat para GstBuffer e enviar para a pipeline
            guint size = frame.total() * frame.elemSize();
            GstBuffer *buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
            GstMapInfo map;
            
            gst_buffer_map(buffer, &map, GST_MAP_WRITE);
            std::memcpy(map.data, frame.data, size);
            gst_buffer_unmap(buffer, &map);

            // Gerir o tempo interno do GStreamer para fluidez
            GST_BUFFER_PTS(buffer) = gst_timestamp_;
            GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, 30); // Assume 30 fps
            gst_timestamp_ += GST_BUFFER_DURATION(buffer);

            // Empurrar frame para a calha!
            GstFlowReturn ret;
            g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
            gst_buffer_unref(buffer);
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Erro no cv_bridge: %s", e.what());
        }
    }

    void init_gstreamer_pipeline(int width, int height) {
        std::string caps_str = "video/x-raw,format=BGR,width=" + std::to_string(width) + 
                               ",height=" + std::to_string(height) + ",framerate=30/1";

        std::string pipeline_str = 
            "appsrc name=mysrc is-live=true format=time caps=\"" + caps_str + "\" ! "
            "videoconvert ! "
            "queue ! "
            "video/x-raw,format=I420 ! "
            "x264enc tune=zerolatency speed-preset=superfast sliced-threads=true threads=1 key-int-max=15 intra-refresh=true bitrate=4000 ! "
            "h264parse config-interval=-1 name=parser ! "
            "rtph264pay pt=96 mtu=1400 aggregate-mode=zero-latency ! "
            "udpsink host=" + ip_address_ + " port=" + std::to_string(port_) + " sync=false";

        GError *error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
        if (error) {
            RCLCPP_ERROR(this->get_logger(), "Erro ao criar pipeline: %s", error->message);
            g_error_free(error);
            return;
        }

        appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "mysrc");

        // INJETOR SEI: Colocar a sonda no 'h264parse' para intercetar as frames já comprimidas
        GstElement *parser = gst_bin_get_by_name(GST_BIN(pipeline_), "parser");
        GstPad *src_pad = gst_element_get_static_pad(parser, "src");
        gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER, (GstPadProbeCallback)pad_probe_callback, this, nullptr);
        gst_object_unref(src_pad);
        gst_object_unref(parser);

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        pipeline_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Pipeline GStreamer iniciada. Resolução: %dx%d", width, height);
    }

    // --- O CRIADOR DE SEI NALU ---
    static GstPadProbeReturn pad_probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
        auto *node = static_cast<VideoEncoderTX*>(user_data);
        GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);

        uint64_t frame_id = 0;
        uint64_t ts_ns = 0;

        // 1. Tentar sacar os metadados da fila correspondentes a esta frame
        {
            std::lock_guard<std::mutex> lock(node->queue_mutex_);
            if (!node->metadata_queue_.empty()) {
                frame_id = node->metadata_queue_.front().first;
                ts_ns = node->metadata_queue_.front().second;
                node->metadata_queue_.pop();
            } else {
                return GST_PAD_PROBE_OK; // Se não houver, passa direto
            }
        }

        // 2. Construir o pacote binário H.264 SEI
        std::vector<uint8_t> sei_nalu;
        sei_nalu.push_back(0x00); sei_nalu.push_back(0x00); sei_nalu.push_back(0x00); sei_nalu.push_back(0x01); // H.264 Start Code
        sei_nalu.push_back(0x06); // Tipo NAL: 6 (SEI)
        sei_nalu.push_back(0x05); // Payload: 5 (Unregistered User Data)

        // Criar a string exatamente como o teu recetor Qt espera
        std::string payload = "ID:" + std::to_string(frame_id) + "|TS:" + std::to_string(ts_ns) + (char)0x80;
        size_t payload_size = 16 + payload.length(); // 16 do UUID + tamanho da string

        // Codificar o tamanho do payload no formato H.264
        size_t s = payload_size;
        while (s >= 255) { sei_nalu.push_back(0xFF); s -= 255; }
        sei_nalu.push_back(s);

        // A tua assinatura secreta (UUID)
        const uint8_t uuid[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 
                                  0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11};
        sei_nalu.insert(sei_nalu.end(), uuid, uuid + 16);
        
        // Colocar a string
        sei_nalu.insert(sei_nalu.end(), payload.begin(), payload.end());

        // 3. Colar o SEI à frente do vídeo já codificado
        GstMapInfo old_map;
        gst_buffer_map(buffer, &old_map, GST_MAP_READ);
        
        // Alocar um buffer novo com espaço para o SEI + Vídeo
        GstBuffer *new_buf = gst_buffer_new_allocate(nullptr, sei_nalu.size() + old_map.size, nullptr);
        GstMapInfo new_map;
        gst_buffer_map(new_buf, &new_map, GST_MAP_WRITE);
        
        // Copiar SEI para o início
        std::memcpy(new_map.data, sei_nalu.data(), sei_nalu.size());
        // Copiar o vídeo codificado logo a seguir
        std::memcpy(new_map.data + sei_nalu.size(), old_map.data, old_map.size);
        
        gst_buffer_unmap(new_buf, &new_map);
        gst_buffer_unmap(buffer, &old_map);

        // Copiar metadados internos do GStreamer para o novo buffer
        gst_buffer_copy_into(new_buf, buffer, GST_BUFFER_COPY_METADATA, 0, -1);
        
        // Substituir o buffer original pela nossa versão "injetada"
        GST_PAD_PROBE_INFO_DATA(info) = new_buf;
        gst_buffer_unref(buffer); // Apagar a memória antiga

        return GST_PAD_PROBE_OK;
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoEncoderTX>());
    rclcpp::shutdown();
    return 0;
}