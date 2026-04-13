#pragma once
#include <QMainWindow>
#include <QTimer>
#include "telemetry_panel.hpp"
#include "camera_gl_widget.hpp"
#include "ros_bridge.hpp"

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(RosBridge* bridge, QWidget* parent = nullptr);

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    TelemetryPanel*  panel_;
    CameraGLWidget*  cam_left_;
    CameraGLWidget*  cam_right_;
    CameraGLWidget*  cam_front_;   
    CameraGLWidget*  cam_back_;
    RosBridge*       bridge_;
};
