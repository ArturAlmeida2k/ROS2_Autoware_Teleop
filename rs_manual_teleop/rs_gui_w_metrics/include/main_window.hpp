#pragma once
#include <QMainWindow>
#include <QTabWidget> 
#include "telemetry_panel.hpp"
#include "camera_gl_widget.hpp"
#include "ros_bridge.hpp"
#include "pointcloud_gl_widget.hpp"

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(RosBridge* bridge, QWidget* parent = nullptr);

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    QTabWidget*     tab_widget_    = nullptr; // O gestor de abas
    QWidget*        tab_quad_view_ = nullptr; // O conteúdo da Aba 1

    TelemetryPanel* panel_         = nullptr;
    CameraGLWidget* cam_left_      = nullptr;
    CameraGLWidget* cam_right_     = nullptr;
    CameraGLWidget* cam_front_     = nullptr;   
    CameraGLWidget* cam_back_      = nullptr;
    RosBridge*      bridge_        = nullptr;
    PointCloudGLWidget* pc_widget_ = nullptr;
};