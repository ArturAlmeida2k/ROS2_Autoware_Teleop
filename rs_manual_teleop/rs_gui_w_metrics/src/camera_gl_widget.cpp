#include "camera_gl_widget.hpp"
#include <opencv2/opencv.hpp>
#include <QOpenGLFunctions_3_3_Core>
#include <QDebug>

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

void CameraGLWidget::resizeGL(int w, int h) { gl33_->glViewport(0, 0, w, h); }

void CameraGLWidget::paintGL()
{
    gl33_->glClear(GL_COLOR_BUFFER_BIT);
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (dirty_ && !pending_frame_.empty())
        {
            if (!texture_->isCreated() || texture_->width() != frame_w_ || texture_->height() != frame_h_)
            {
                texture_->destroy();
                texture_->create();
                texture_->setSize(frame_w_, frame_h_);
                texture_->setFormat(QOpenGLTexture::RGB8_UNorm);
                texture_->allocateStorage(QOpenGLTexture::RGB, QOpenGLTexture::UInt8);
            }
            texture_->setData(QOpenGLTexture::RGB, QOpenGLTexture::UInt8, static_cast<const void *>(pending_frame_.data()));
            dirty_ = false;
        }
    }
    if (!texture_->isCreated())
        return;
    shader_->bind();
    texture_->bind();
    gl33_->glBindVertexArray(vao_);
    gl33_->glDrawArrays(GL_TRIANGLES, 0, 6);
    gl33_->glBindVertexArray(0);
    texture_->release();
    shader_->release();
}

void CameraGLWidget::setup_shaders()
{
    shader_ = new QOpenGLShaderProgram(this);
    shader_->addShaderFromSourceCode(QOpenGLShader::Vertex, VERT_SRC);
    shader_->addShaderFromSourceCode(QOpenGLShader::Fragment, FRAG_SRC);
    shader_->link();
    if (!shader_->link()) 
    {
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

// A função da thread que consome o GStreamer e ignora o ROS
void CameraGLWidget::video_capture_loop(int port)
{
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
            pending_frame_.assign(frame.data, frame.data + frame.total() * frame.elemSize());
            dirty_ = true;
        }
        
        QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);
    }
    cap.release();
}