#pragma once
#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QWheelEvent>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mutex>
#include <vector>
#include <cstdint>
 
class PointCloudGLWidget : public QOpenGLWidget {
    Q_OBJECT
public:
    explicit PointCloudGLWidget(QWidget* parent = nullptr);
    ~PointCloudGLWidget() override;
 
public slots:
    void onPointCloudReceived(sensor_msgs::msg::PointCloud2::SharedPtr msg);
 
signals:
    void displayLatencyUpdated(uint32_t id, double latency_ms);
 
protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
 
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
 
private:
    QOpenGLFunctions_3_3_Core* gl33_ = nullptr;
    
    QOpenGLShaderProgram* shader_ = nullptr;
    unsigned int vao_ = 0;
    unsigned int vbo_ = 0;
 
    QOpenGLShaderProgram* car_shader_ = nullptr;
    unsigned int car_vao_ = 0;
    unsigned int car_vbo_ = 0;
 
    std::mutex frame_mutex_;
    std::vector<float> points_; // Guarda sequencialmente X, Y, Z
    bool dirty_ = false;
 
    builtin_interfaces::msg::Time pending_stamp_;
    uint32_t pending_id_ = 0;
 
    // Controlos de Câmara
    float yaw_ = 0.0f;       // Volta a meter a 0
    float pitch_ = -60.0f;     
    float distance_ = 50.0f; // Mantém os 50 para o zoom out
    QPoint last_mouse_pos_;
};
 
