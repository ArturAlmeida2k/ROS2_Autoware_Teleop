#pragma once
#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLTexture>
#include <QOpenGLShaderProgram>
#include <mutex>
#include <vector>
#include <map>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

class CameraGLWidget : public QOpenGLWidget {
    Q_OBJECT
public:
    explicit CameraGLWidget(int port, QWidget* parent = nullptr);
    ~CameraGLWidget() override;

signals:
    // Sinal para poderes atualizar labels na MainWindow
    void latencyUpdated(uint64_t frame_id, double latency_ms);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

private:
    QOpenGLFunctions_3_3_Core* gl33_  = nullptr;
    QOpenGLTexture* texture_          = nullptr;
    QOpenGLShaderProgram* shader_     = nullptr;
    unsigned int vao_                 = 0;
    unsigned int vbo_                 = 0;

    std::mutex           frame_mutex_;
    std::vector<uint8_t> pending_frame_;
    int                  frame_w_ = 0;
    int                  frame_h_ = 0;
    bool                 dirty_   = false;
    
    std::map<uint64_t, std::pair<uint64_t, uint64_t>> pts_map_;
    std::mutex queue_mutex_;

    uint64_t pending_id_ = 0;
    uint64_t pending_ts_ = 0;

    // Gestão do GStreamer
    GstElement* pipeline_ = nullptr;
    void start_pipeline(int port);
    void stop_pipeline();

    // Callbacks estáticas do GStreamer
    static GstPadProbeReturn pad_probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data);
    static GstFlowReturn on_new_sample(GstElement *sink, gpointer user_data);

    void setup_quad();
    void setup_shaders();
};