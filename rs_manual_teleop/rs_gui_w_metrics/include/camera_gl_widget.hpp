#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QTimer>
#include <gst/gst.h>
#include <mutex>
#include <vector>
#include <queue>
#include <utility>
#include <cstdint>

class CameraGLWidget : public QOpenGLWidget {
    Q_OBJECT

public:
    explicit CameraGLWidget(int port, QWidget *parent = nullptr);
    ~CameraGLWidget() override;

signals:
    void latencyUpdated(uint64_t frame_id, double latency_ms);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

private:
    void setup_shaders();
    void setup_quad();
    void start_pipeline(int port);
    void stop_pipeline();

    static GstPadProbeReturn pad_probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data);
    static GstFlowReturn on_new_sample(GstElement *sink, gpointer user_data);

    // OpenGL
    QOpenGLFunctions_3_3_Core *gl33_ = nullptr;
    QOpenGLShaderProgram *shader_ = nullptr;
    QOpenGLTexture *texture_ = nullptr;
    GLuint vao_ = 0, vbo_ = 0;

    // GStreamer
    GstElement *pipeline_ = nullptr;

    // NOVO: timer que desacopla o pedido de repintura do ritmo de chegada
    // de frames do GStreamer — pede update() a um ritmo fixo, em vez de a
    // cada frame recebido.
    QTimer *render_timer_ = nullptr;

    // Frame pendente para render
    std::mutex frame_mutex_;
    std::vector<uint8_t> pending_frame_;
    int frame_w_ = 0, frame_h_ = 0;
    uint64_t pending_id_ = 0;
    uint64_t pending_ts_ = 0;
    bool dirty_ = false;

    // SEI extraído pela sonda, antes do decode
    std::mutex queue_mutex_;
    std::queue<std::pair<uint64_t, uint64_t>> sei_queue_;
};