#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

// Classe principal que herda de rclcpp::Node para criar o nó no ecossistema ROS 2
class H264Streamer : public rclcpp::Node {
public:
    H264Streamer() : Node("h264_streamer"), pipeline_(nullptr), appsrc_(nullptr) {
        // 1. Inicializa o motor interno do GStreamer.
        gst_init(nullptr, nullptr);

        // 2. Definição do Pipeline de Rede (Passthrough H.264)
        // appsrc: Ponto de entrada que aceita dados da nossa função em vez de ficheiros ou câmaras.
        // is-live=true: Instrui o GStreamer a não reter dados em cache, focando na latência zero.
        // caps=...: Declara que a entrada é estritamente vídeo H.264 em formato raw byte-stream.
        // h264parse: Extrai os parâmetros estruturais (SPS/PPS) do fluxo de bytes.
        // rtph264pay: Empacota os dados H.264 no protocolo RTP. aggregate-mode=zero-latency evita acumulação de pacotes.
        // udpsink: Envia os pacotes finais via UDP. sync=false diz para despachar imediatamente.
        std::string pipeline_str =
            "appsrc name=mysrc is-live=true do-timestamp=true format=time "
            "caps=video/x-h264,stream-format=byte-stream,alignment=au ! "
            "h264parse config-interval=-1 ! "
            "rtph264pay pt=96 mtu=1400 aggregate-mode=zero-latency ! "
            "udpsink host=192.168.94.202 port=5006 sync=false";

        // Converte a string num objeto de pipeline operacional
        GError *error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
        if (error) {
            RCLCPP_ERROR(this->get_logger(), "Erro ao criar pipeline: %s", error->message);
            g_error_free(error);
            return;
        }

        // Obtém o ponteiro de memória diretamente para o elemento "appsrc" (nomeado mysrc na string)
        appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "mysrc");
        
        // Inicia a execução do pipeline GStreamer
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        // 3. Subscrição ROS 2
        // Configuração QoS: best_effort().keep_last(1) instrui o ROS a manter sempre
        // apenas a mensagem mais recente, descartando as mais antigas em caso de congestionamento.
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera_0/compressed_image",
            rclcpp::QoS(1).best_effort().keep_last(1),
            std::bind(&H264Streamer::image_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "H264Streamer iniciado. A enviar para 192.168.94.202:5006");
    }

    // O destrutor liberta a memória dos objetos do GStreamer quando o nó é encerrado
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
    // Esta função executa automaticamente sempre que uma mensagem chega ao tópico ROS 2
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        if (!appsrc_) return; // Aborta se a porta de entrada não estiver instanciada

        // 4. Operações de Memória Direta (Zero-Copy Transfer)
        // Pede ao sistema um bloco de memória vazia no GStreamer do tamanho da frame recebida
        GstBuffer *buffer = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
        
        // Mapeia o buffer para obtermos um ponteiro de escrita
        GstMapInfo map;
        gst_buffer_map(buffer, &map, GST_MAP_WRITE);
        
        // Copia os dados comprimidos H.264 do ROS 2 diretamente para a memória do GStreamer
        memcpy(map.data, msg->data.data(), msg->data.size());
        
        // Fecha o mapeamento de escrita
        gst_buffer_unmap(buffer, &map);

        // 5. Aplicação do Timestamp
        // Converte os segundos e nanossegundos do cabeçalho do ROS num valor de tempo contínuo.
        // Permite ao recetor reproduzir os frames no tempo exato em que foram gerados.
        GST_BUFFER_PTS(buffer) = 
            (guint64)msg->header.stamp.sec * GST_SECOND + 
            (guint64)msg->header.stamp.nanosec;

        // 6. Injeção de Dados no GStreamer
        // Envia o sinal que empurra o bloco de memória preenchido para dentro do appsrc
        GstFlowReturn ret;
        g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
        
        // Liberta a referência da variável (o motor interno do GStreamer tratará de limpar a memória real no final da transmissão)
        gst_buffer_unref(buffer);

        // Validação de sucesso no envio local
        if (ret != GST_FLOW_OK) {
            RCLCPP_WARN(this->get_logger(), "Erro ao enviar buffer: %d", ret);
        }
    }

    // Variáveis internas da classe
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    GstElement *pipeline_;
    GstElement *appsrc_;
};

int main(int argc, char *argv[]) {
    // Inicializa a infraestrutura ROS 2
    rclcpp::init(argc, argv);
    
    // Inicia o nó num ciclo de execução contínua
    rclcpp::spin(std::make_shared<H264Streamer>());
    
    // Processo de encerramento controlado
    rclcpp::shutdown();
    return 0;
}