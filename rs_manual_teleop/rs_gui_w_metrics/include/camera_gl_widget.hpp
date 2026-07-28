#pragma once
#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLTexture>
#include <QOpenGLShaderProgram>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>

class CameraGLWidget : public QOpenGLWidget {
    Q_OBJECT
public:
    // O construtor agora recebe a porta UDP que esta câmara vai escutar
    explicit CameraGLWidget(int port, QWidget* parent = nullptr);
    ~CameraGLWidget() override;

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

    // Gestão da thread do GStreamer
    std::thread          video_thread_;
    std::atomic<bool>    running_;
    void video_capture_loop(int port);

    void setup_quad();
    void setup_shaders();
};