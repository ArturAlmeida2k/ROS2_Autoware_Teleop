#include "camera_gl_widget.hpp"
#include <QOpenGLFunctions_3_3_Core>
#include <QDebug>
#include <QTimer>
#include <chrono>
#include <cstring>

static const char *VERT_SRC = R"(#version 330 core
layout(location=0) in vec2 aPos;
layout(location=1) in vec2 aUV;
out vec2 vUV;
void main() {
    gl_Position = vec4(aPos, 0.0, 1.0);
    vUV = aUV;
})";

static const char *FRAG_SRC = R"(#version 330 core
in vec2 vUV;
out vec4 FragColor;
uniform sampler2D uTex;
void main() {
    FragColor = texture(uTex, vUV);
})";

static const float QUAD[] = {
    -1.f,  1.f, 0.f, 0.f,
    -1.f, -1.f, 0.f, 1.f,
     1.f, -1.f, 1.f, 1.f,
    -1.f,  1.f, 0.f, 0.f,
     1.f, -1.f, 1.f, 1.f,
     1.f,  1.f, 1.f, 0.f
};

CameraGLWidget::CameraGLWidget(int port, QWidget *parent)
    : QOpenGLWidget(parent)
{
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    setFormat(fmt);

    gst_init(nullptr, nullptr);
    start_pipeline(port);

    // Repintura contínua: pede update() a ritmo alto, independentemente de
    // quando chegam frames. O vsync trava no refresh real, por isso o custo
    // é baixo. Assim o paintGL apanha cada frame novo no primeiro refresh
    // disponível, em vez de esperar pela fase do update() disparado por evento.
    render_timer_ = new QTimer(this);
    connect(render_timer_, &QTimer::timeout, this, [this]() { update(); });
    render_timer_->start(2);
}

CameraGLWidget::~CameraGLWidget()
{
    if (render_timer_) render_timer_->stop();
    stop_pipeline();

    makeCurrent();
    delete texture_;
    delete shader_;
    if (gl33_) {
        gl33_->glDeleteVertexArrays(1, &vao_);
        gl33_->glDeleteBuffers(1, &vbo_);
    }
    doneCurrent();
}

void CameraGLWidget::initializeGL()
{
    gl33_ = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_3_3_Core>();
    gl33_->initializeOpenGLFunctions();
    setup_shaders();
    setup_quad();

    texture_ = new QOpenGLTexture(QOpenGLTexture::Target2D);
    texture_->setMinificationFilter(QOpenGLTexture::Linear);
    texture_->setMagnificationFilter(QOpenGLTexture::Linear);
    texture_->setWrapMode(QOpenGLTexture::ClampToEdge);
}

void CameraGLWidget::resizeGL(int w, int h)
{
    gl33_->glViewport(0, 0, w, h);
}

void CameraGLWidget::paintGL()
{
    static std::vector<uint8_t> local_frame;
    int local_w = 0, local_h = 0;
    uint64_t id_to_emit = 0;
    uint64_t ts_to_emit = 0;
    bool has_new_frame = false;

    gl33_->glClear(GL_COLOR_BUFFER_BIT);

    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (dirty_ && !pending_frame_.empty()) {
            local_frame.swap(pending_frame_);

            local_w = frame_w_;
            local_h = frame_h_;
            id_to_emit = pending_id_;
            ts_to_emit = pending_ts_;
            dirty_ = false;
            has_new_frame = true;
        }
    }

    // Só faz upload quando há frame novo. Nas restantes passagens redesenha
    // a textura que já lá está — o ecrã mostra sempre a imagem mais recente.
    if (has_new_frame) {
        if (!texture_->isCreated() || texture_->width() != local_w || texture_->height() != local_h) {
            if (texture_->isCreated()) {
                texture_->destroy();
            }
            texture_->create();
            texture_->setSize(local_w, local_h);
            texture_->setFormat(QOpenGLTexture::RGB8_UNorm);
            texture_->allocateStorage(QOpenGLTexture::RGB, QOpenGLTexture::UInt8);
        }
        texture_->setData(QOpenGLTexture::RGB, QOpenGLTexture::UInt8, static_cast<const void *>(local_frame.data()));
    }

    if (!texture_->isCreated()) return;

    shader_->bind();
    texture_->bind();
    gl33_->glBindVertexArray(vao_);
    gl33_->glDrawArrays(GL_TRIANGLES, 0, 6);
    gl33_->glBindVertexArray(0);
    texture_->release();
    shader_->release();

    // METRICA 3 — só emite quando havia frame novo, senão repetiria o mesmo
    // valor a cada tick do timer e enviesava o bag.
    if (has_new_frame && id_to_emit > 0 && ts_to_emit > 0) {
        auto now = std::chrono::system_clock::now().time_since_epoch();
        int64_t render_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
        double full_latency_ms = (render_ns - (int64_t)ts_to_emit) / 1000000.0;
        emit latencyUpdated(id_to_emit, full_latency_ms);
    }
}

void CameraGLWidget::setup_shaders()
{
    shader_ = new QOpenGLShaderProgram(this);
    shader_->addShaderFromSourceCode(QOpenGLShader::Vertex, VERT_SRC);
    shader_->addShaderFromSourceCode(QOpenGLShader::Fragment, FRAG_SRC);
    if (!shader_->link()) {
        qWarning() << "Shader link failed:" << shader_->log();
    }
}

void CameraGLWidget::setup_quad()
{
    gl33_->glGenVertexArrays(1, &vao_);
    gl33_->glGenBuffers(1, &vbo_);
    gl33_->glBindVertexArray(vao_);
    gl33_->glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    gl33_->glBufferData(GL_ARRAY_BUFFER, sizeof(QUAD), QUAD, GL_STATIC_DRAW);

    gl33_->glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);
    gl33_->glEnableVertexAttribArray(0);

    gl33_->glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(2 * sizeof(float)));
    gl33_->glEnableVertexAttribArray(1);
}

void CameraGLWidget::start_pipeline(int port)
{
    std::string pipeline_str =
        "udpsrc port=" + std::to_string(port) + " "
        "caps=\"application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96\" ! "
        "rtpjitterbuffer latency=0 drop-on-latency=true ! "
        "rtph264depay ! "
        "h264parse name=parser ! "
        "video/x-h264,stream-format=byte-stream,alignment=au ! "
        "queue max-size-buffers=5 max-size-bytes=0 max-size-time=0 ! "
        "avdec_h264 max-threads=4 ! "
        "videoconvert n-threads=4 ! "
        "video/x-raw,format=RGB ! "
        "appsink name=mysink sync=false max-buffers=2 emit-signals=true";

    GError *error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);

    if (error) {
        qWarning() << "Erro GStreamer:" << error->message;
        g_error_free(error);
        return;
    }

    GstElement *parser = gst_bin_get_by_name(GST_BIN(pipeline_), "parser");
    GstPad *src_pad = gst_element_get_static_pad(parser, "src");
    gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER,
                      (GstPadProbeCallback)pad_probe_callback, this, nullptr);
    gst_object_unref(src_pad);
    gst_object_unref(parser);

    GstElement *appsink = gst_bin_get_by_name(GST_BIN(pipeline_), "mysink");
    g_signal_connect(appsink, "new-sample", G_CALLBACK(on_new_sample), this);
    gst_object_unref(appsink);

    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
}

void CameraGLWidget::stop_pipeline()
{
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }
}

// Antes do decode. METRICA 1 — rede isolada.
GstPadProbeReturn CameraGLWidget::pad_probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
    auto *widget = static_cast<CameraGLWidget*>(user_data);
    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    GstMapInfo map;

    if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        const uint8_t uuid[16] = {
            0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
            0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11
        };

        size_t search_limit = std::min(map.size - 16, (size_t)128);
        for (size_t i = 0; i < search_limit - 16; ++i) {
            if (std::memcmp(map.data + i, uuid, 16) == 0) {
                size_t str_len = std::min(map.size - i - 16, (size_t)96);
                std::string payload(reinterpret_cast<char*>(map.data + i + 16), str_len);

                size_t end_pos = payload.find((char)0x80);
                if (end_pos == std::string::npos) break;
                payload = payload.substr(0, end_pos);

                uint64_t frame_id = 0, ts_ns = 0, ts2_ns = 0;
                if (sscanf(payload.c_str(), "ID:%lu|TS:%lu|TS2:%lu", &frame_id, &ts_ns, &ts2_ns) != 3) break;
                if (ts_ns == 0 || ts2_ns == 0) break;

                auto now = std::chrono::system_clock::now().time_since_epoch();
                int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

                int64_t diff_ns = now_ns - (int64_t)ts2_ns;
                double network_ms = diff_ns / 1000000.0;
                emit widget->networkLatencyUpdated(frame_id, network_ms);

                {
                    std::lock_guard<std::mutex> lock(widget->queue_mutex_);
                    while (widget->sei_queue_.size() > 3) {
                        widget->sei_queue_.pop();
                    }
                    widget->sei_queue_.push({frame_id, ts_ns});
                }
                break;
            }
        }
        gst_buffer_unmap(buffer, &map);
    }
    return GST_PAD_PROBE_OK;
}

// Depois do decode. METRICA 2 — captura -> pronto para o paintGL.
// Já não chama update(): o timer trata da repintura.
GstFlowReturn CameraGLWidget::on_new_sample(GstElement *sink, gpointer user_data)
{
    auto *widget = static_cast<CameraGLWidget*>(user_data);
    GstSample *sample;
    g_signal_emit_by_name(sink, "pull-sample", &sample);

    if (sample) {
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        GstCaps *caps = gst_sample_get_caps(sample);

        GstStructure *s = gst_caps_get_structure(caps, 0);
        int width, height;
        gst_structure_get_int(s, "width", &width);
        gst_structure_get_int(s, "height", &height);

        GstMapInfo map;
        gst_buffer_map(buffer, &map, GST_MAP_READ);

        uint64_t current_id = 0, current_ts = 0;
        {
            std::lock_guard<std::mutex> lock(widget->queue_mutex_);
            if (!widget->sei_queue_.empty()) {
                current_id = widget->sei_queue_.front().first;
                current_ts = widget->sei_queue_.front().second;
                widget->sei_queue_.pop();
            }
        }

        if (current_id > 0 && current_ts > 0) {
            auto now = std::chrono::system_clock::now().time_since_epoch();
            int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
            double decode_ms = (now_ns - (int64_t)current_ts) / 1000000.0;
            emit widget->decodeLatencyUpdated(current_id, decode_ms);
        }

        {
            std::lock_guard<std::mutex> lock(widget->frame_mutex_);
            widget->frame_w_ = width;
            widget->frame_h_ = height;
            widget->pending_frame_.assign(map.data, map.data + map.size);
            widget->pending_id_ = current_id;
            widget->pending_ts_ = current_ts;
            widget->dirty_ = true;
        }

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }
    return GST_FLOW_ERROR;
}