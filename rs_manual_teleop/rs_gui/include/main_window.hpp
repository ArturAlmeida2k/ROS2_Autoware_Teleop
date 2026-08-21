#pragma once
#include <QMainWindow>
#include <QTabWidget>
#include <QKeyEvent>
#include "telemetry_panel.hpp"
#include "speed_panel.hpp"
#include "camera_gl_widget.hpp"
#include "ros_bridge.hpp"
#include "pointcloud_gl_widget.hpp"
 
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(RosBridge* bridge, QWidget* parent = nullptr);
 
protected:
    void resizeEvent(QResizeEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;
 
private:
    void setSingleCameraMode(bool single);
    void reposition_overlays();
 
    bool is_single_camera_ = false;
 
    QTabWidget* tab_widget_    = nullptr;
    QWidget*    tab_quad_view_ = nullptr;
 
    TelemetryPanel*     panel_     = nullptr;
    SpeedPanel*         speed_     = nullptr;
    CameraGLWidget*     cam_left_  = nullptr;
    CameraGLWidget*     cam_right_ = nullptr;
    CameraGLWidget*     cam_front_ = nullptr;
    CameraGLWidget*     cam_back_  = nullptr;
    RosBridge*          bridge_    = nullptr;
    PointCloudGLWidget* pc_widget_ = nullptr;
};
 
