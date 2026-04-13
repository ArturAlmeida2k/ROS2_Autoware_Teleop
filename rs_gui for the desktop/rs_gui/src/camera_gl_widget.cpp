#include "camera_gl_widget.hpp"
#include <opencv2/opencv.hpp>
#include <QOpenGLFunctions_3_3_Core>  // substitui QOpenGLFunctions

// Shaders minimalistas: quad fullscreen + textura
static const char* VERT_SRC = R"(
#version 330 core
layout(location=0) in vec2 aPos;
layout(location=1) in vec2 aUV;
out vec2 vUV;
void main() { gl_Position = vec4(aPos, 0.0, 1.0); vUV = aUV; }
)";

static const char* FRAG_SRC = R"(
#version 330 core
in vec2 vUV;
out vec4 FragColor;
uniform sampler2D uTex;
void main() { FragColor = texture(uTex, vUV); }
)";

static const float QUAD[] = {
    -1.f,  1.f,   0.f, 0.f,
    -1.f, -1.f,   0.f, 1.f,
     1.f, -1.f,   1.f, 1.f,
    -1.f,  1.f,   0.f, 0.f,
     1.f, -1.f,   1.f, 1.f,
     1.f,  1.f,   1.f, 0.f,
};

// Ponteiro para funções OpenGL 3.3
static QOpenGLFunctions_3_3_Core* gl33 = nullptr;

CameraGLWidget::CameraGLWidget(QWidget* parent)
: QOpenGLWidget(parent)
{
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    setFormat(fmt);
}

CameraGLWidget::~CameraGLWidget()
{
    makeCurrent();
    delete texture_;
    delete shader_;
    if (gl33) {
        gl33->glDeleteVertexArrays(1, &vao_);
        gl33->glDeleteBuffers(1, &vbo_);
    }
    doneCurrent();
}

void CameraGLWidget::initializeGL()
{
    gl33 = QOpenGLContext::currentContext()
               ->versionFunctions<QOpenGLFunctions_3_3_Core>();
    gl33->initializeOpenGLFunctions();
    gl33->glClearColor(0.05f, 0.05f, 0.08f, 1.f);

    setup_shaders();
    setup_quad();

    texture_ = new QOpenGLTexture(QOpenGLTexture::Target2D);
    texture_->setMinificationFilter(QOpenGLTexture::Linear);
    texture_->setMagnificationFilter(QOpenGLTexture::Linear);
    texture_->setWrapMode(QOpenGLTexture::ClampToEdge);
}

void CameraGLWidget::resizeGL(int w, int h)
{
    gl33->glViewport(0, 0, w, h);
}

void CameraGLWidget::paintGL()
{
    gl33->glClear(GL_COLOR_BUFFER_BIT);

    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (dirty_ && !pending_frame_.empty()) {
            if (!texture_->isCreated() ||
                texture_->width() != frame_w_ ||
                texture_->height() != frame_h_)
            {
                texture_->destroy();
                texture_->create();
                texture_->setSize(frame_w_, frame_h_);
                texture_->setFormat(QOpenGLTexture::RGB8_UNorm);
                texture_->allocateStorage(QOpenGLTexture::RGB, QOpenGLTexture::UInt8);
            }
            // API não-deprecated
            texture_->setData(0, QOpenGLTexture::RGB, QOpenGLTexture::UInt8,
                              pending_frame_.data());
            dirty_ = false;
        }
    }

    if (!texture_->isCreated()) return;

    shader_->bind();
    texture_->bind();
    gl33->glBindVertexArray(vao_);
    gl33->glDrawArrays(GL_TRIANGLES, 0, 6);
    gl33->glBindVertexArray(0);
    texture_->release();
    shader_->release();
}

void CameraGLWidget::onImageReceived(sensor_msgs::msg::Image::SharedPtr msg)
{
    // Assume que a imagem raw chega no formato "bgr8"
    cv::Mat frame(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(msg->data.data()), msg->step);
    
    if (frame.empty()) return;
    
    cv::Mat rgb_frame;
    cv::cvtColor(frame, rgb_frame, cv::COLOR_BGR2RGB);

    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        frame_w_ = rgb_frame.cols;
        frame_h_ = rgb_frame.rows;
        pending_frame_.assign(rgb_frame.data,
                              rgb_frame.data + rgb_frame.total() * rgb_frame.elemSize());
        dirty_ = true;
    }
    QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);
}

void CameraGLWidget::setup_shaders()
{
    shader_ = new QOpenGLShaderProgram(this);
    shader_->addShaderFromSourceCode(QOpenGLShader::Vertex,   VERT_SRC);
    shader_->addShaderFromSourceCode(QOpenGLShader::Fragment, FRAG_SRC);
    shader_->link();
}

void CameraGLWidget::setup_quad()
{
    gl33->glGenVertexArrays(1, &vao_);
    gl33->glGenBuffers(1, &vbo_);
    gl33->glBindVertexArray(vao_);
    gl33->glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    gl33->glBufferData(GL_ARRAY_BUFFER, sizeof(QUAD), QUAD, GL_STATIC_DRAW);
    gl33->glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE,
                                4 * sizeof(float), (void*)0);
    gl33->glEnableVertexAttribArray(0);
    gl33->glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE,
                                4 * sizeof(float), (void*)(2 * sizeof(float)));
    gl33->glEnableVertexAttribArray(1);
    gl33->glBindVertexArray(0);
}
