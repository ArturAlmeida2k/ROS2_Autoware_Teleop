#pragma once
#include <QMainWindow>
#include <QTabWidget>
#include <QKeyEvent>
#include <QQuickWidget>
#include "telemetry_panel.hpp"
#include "camera_stream_manager.hpp"   // trocado de camera_gl_widget.hpp
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
bool is_single_camera_ = true;

// Helper: cria um QQuickWidget com o vídeo carregado + o manager GStreamer ligado
QQuickWidget* makeCameraView(int port, QWidget* parent, CameraStreamManager** out_manager);

QTabWidget*     tab_widget_    = nullptr;
QWidget*        tab_quad_view_ = nullptr;

TelemetryPanel* panel_         = nullptr;

// Cada câmara agora é um QQuickWidget (visível, entra no grid) +
// um CameraStreamManager (lógica GStreamer não-visual, ligado ao item QML lá dentro)
QQuickWidget*        cam_left_widget_   = nullptr;
QQuickWidget*        cam_right_widget_  = nullptr;
QQuickWidget*        cam_front_widget_  = nullptr;
QQuickWidget*        cam_back_widget_   = nullptr;

CameraStreamManager*  cam_left_mgr_   = nullptr;
CameraStreamManager*  cam_right_mgr_  = nullptr;
CameraStreamManager*  cam_front_mgr_  = nullptr;
CameraStreamManager*  cam_back_mgr_   = nullptr;

RosBridge*      bridge_        = nullptr;
PointCloudGLWidget* pc_widget_ = nullptr;
};