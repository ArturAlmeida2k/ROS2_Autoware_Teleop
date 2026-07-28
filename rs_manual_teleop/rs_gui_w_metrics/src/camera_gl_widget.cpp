#include "camera_gl_widget.hpp"
#include <opencv2/opencv.hpp>
#include <QOpenGLFunctions_3_3_Core>
#include <QDebug>

// Shaders e QUAD permanecem iguais...
static const char *VERT_SRC = R"(#version 330 core
layout(location=0) in vec2 aPos; layout(location=1) in vec2 aUV;
out vec2 vUV; void main() { gl_Position = vec4(aPos, 0.0, 1.0); vUV = aUV; })";

static const char *FRAG_SRC = R"(#version 330 core
in vec2 vUV; out vec4 FragColor; uniform sampler2D uTex;
void main() { FragColor = texture(uTex, vUV); })";

static const float QUAD[] = {-1.f, 1.f, 0.f, 0.f, -1.f, -1.f, 0.f, 1.f, 1.f, -1.f, 1.f, 1.f, -1.f, 1.f, 0.f, 0.f, 1.f, -1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 0.f};

// Construtor: Inicializa a variável atomic e lança a thread GStreamer
CameraGLWidget::CameraGLWidget(int port, QWidget *parent) 
    : QOpenGLWidget(parent), running_(true)
{
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    setFormat(fmt);

    video_thread_ = std::thread(&CameraGLWidget::video_capture_loop, this, port);
}

// Destrutor: Para a thread antes de destruir o widget
CameraGLWidget::~CameraGLWidget()
{
    running_ = false;
    if (video_thread_.joinable()) {
        video_thread_.join();
    }
    
    makeCurrent();
    delete texture_;
    delete shader_;
    if (gl33_) {
        gl33_->glDeleteVertexArrays(1, &vao_);
        gl33_->glDeleteBuffers(1, &vbo_);
    }
    doneCurrent();
}

// initializeGL, resizeGL, paintGL, setup_shaders e setup_quad permanecem exatamente iguais...
// [Manter o código atual dessas funções sem alterações]

// NOVA FUNÇÃO: Substitui o antigo onImageReceived
void CameraGLWidget::video_capture_loop(int port)
{
    // Pipeline otimizada: Aceleração GPU (nvh264dec) e saída nativa RGB para encaixar no OpenGL
    std::string pipeline =
        "udpsrc port=" + std::to_string(port) + " caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! "
        "rtpjitterbuffer latency=0 ! "
        "rtph264depay ! h264parse ! nvh264dec ! "
        "videoconvert ! video/x-raw,format=RGB ! "
        "appsink sync=false drop=true max-buffers=1";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        qWarning() << "Falha ao abrir stream UDP na porta" << port;
        return;
    }

    cv::Mat frame;
    while (running_) {
        cap >> frame; 
        if (frame.empty()) continue;

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            frame_w_ = frame.cols;
            frame_h_ = frame.rows;
            // A imagem já vem em RGB do GStreamer, elimina a necessidade de cv::cvtColor
            pending_frame_.assign(frame.data, frame.data + frame.total() * frame.elemSize());
            dirty_ = true;
        }
        
        QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);
    }
    cap.release();
}