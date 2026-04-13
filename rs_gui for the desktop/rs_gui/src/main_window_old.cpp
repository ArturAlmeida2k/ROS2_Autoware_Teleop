#include "main_window.hpp"
#include <QHBoxLayout>
#include <QWidget>

MainWindow::MainWindow(RosBridge* bridge, QWidget* parent)
: QMainWindow(parent), bridge_(bridge)
{
    setWindowTitle("Teleoperation HUD");
    setStyleSheet("background-color: #11111b;");
    resize(1440, 900);

    auto* central = new QWidget(this);
    auto* root    = new QHBoxLayout(central);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);
    setCentralWidget(central);

    // Câmara principal (expansível para grid de 4)
    camera_front_ = new CameraGLWidget(this);
    camera_front_->setMinimumSize(854, 480);

    // Painel de telemetria
    panel_ = new TelemetryPanel(this);

    root->addWidget(camera_front_, 1);
    root->addWidget(panel_,        0);

    // Ligações ROS → Widgets (Qt::QueuedConnection = thread-safe)
    connect(bridge_, &RosBridge::telemetryReceived,
            panel_,  &TelemetryPanel::onTelemetryReceived,
            Qt::QueuedConnection);

    connect(bridge_, &RosBridge::imageReceived,
            camera_front_, &CameraGLWidget::onImageReceived,
            Qt::QueuedConnection);
}
