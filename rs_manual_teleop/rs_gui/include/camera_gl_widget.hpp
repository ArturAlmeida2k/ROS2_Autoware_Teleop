#pragma once
#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLTexture>
#include <QOpenGLShaderProgram>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <mutex>
#include <vector>
using CompressedImage = sensor_msgs::msg::CompressedImage;

// Renderiza um frame de câmara via OpenGL — pronto para point clouds depois
class CameraGLWidget : public QOpenGLWidget {
    Q_OBJECT
public:
    explicit CameraGLWidget(QWidget* parent = nullptr);
    ~CameraGLWidget() override;

public slots:
    void onImageReceived(sensor_msgs::msg::CompressedImage::SharedPtr msg);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

private:
    QOpenGLTexture*       texture_      = nullptr;
    QOpenGLShaderProgram* shader_       = nullptr;
    unsigned int          vao_          = 0;
    unsigned int          vbo_          = 0;

    std::mutex            frame_mutex_;
    std::vector<uint8_t>  pending_frame_;
    int                   frame_w_ = 0;
    int                   frame_h_ = 0;
    bool                  dirty_   = false;

    void setup_quad();
    void setup_shaders();
};
