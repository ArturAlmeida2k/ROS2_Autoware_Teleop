#include "main_window.hpp"
#include <QGridLayout>
#include <QVBoxLayout>
#include <QWidget>
#include <QResizeEvent>
#include <QTimer>

MainWindow::MainWindow(RosBridge* bridge, QWidget* parent)
: QMainWindow(parent), bridge_(bridge)
{
    setWindowTitle("Teleoperation HUD");
    setStyleSheet("background-color: #11111b;");

    tab_widget_ = new QTabWidget(this);
    setCentralWidget(tab_widget_);

    tab_widget_->setStyleSheet(
        "QTabBar::tab { background: #1e1e2e; color: #cdd6f4; padding: 8px 16px; "
        "border-radius: 4px; margin: 2px; }"
        "QTabBar::tab:selected { background: #313244; font-weight: bold; }"
        "QTabWidget::pane { border: none; }"
    );

    // =====================================================================
    // ABA 1 — vistas das câmaras
    // =====================================================================
    tab_quad_view_ = new QWidget();
    auto* grid = new QGridLayout(tab_quad_view_);
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setSpacing(2);

    // A porta define a câmara: 5007 front, 5008 left, 5009 back, 5010 right.
    cam_front_ = new CameraGLWidget(5007, tab_quad_view_);
    cam_left_  = new CameraGLWidget(5008, tab_quad_view_);
    cam_back_  = new CameraGLWidget(5009, tab_quad_view_);
    cam_right_ = new CameraGLWidget(5010, tab_quad_view_);

    // Só a câmara frontal transporta SEI, por isso é a única instrumentada.
    connect(cam_front_, &CameraGLWidget::latencyUpdated, this,
            [this](uint64_t frame_id, double latency_ms) {
        bridge_->publishFrontCameraMetrics(static_cast<uint32_t>(frame_id), latency_ms);
        if (panel_) panel_->setVideoLatency(latency_ms);
    }, Qt::QueuedConnection);

    connect(cam_front_, &CameraGLWidget::networkLatencyUpdated, this,
            [this](uint64_t frame_id, double latency_ms) {
        bridge_->publishFrontCameraNetwork(static_cast<uint32_t>(frame_id), latency_ms);
    }, Qt::QueuedConnection);

    connect(cam_front_, &CameraGLWidget::decodeLatencyUpdated, this,
            [this](uint64_t frame_id, double latency_ms) {
        bridge_->publishFrontCameraDecode(static_cast<uint32_t>(frame_id), latency_ms);
    }, Qt::QueuedConnection);

    grid->addWidget(cam_left_,  0, 0, 2, 1);
    grid->addWidget(cam_front_, 0, 1, 1, 1);
    grid->addWidget(cam_back_,  1, 1, 1, 1);
    grid->addWidget(cam_right_, 0, 2, 2, 1);

    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 4);
    grid->setColumnStretch(2, 1);
    grid->setRowStretch(0, 2);
    grid->setRowStretch(1, 1);

    // Sobreposições: painel de estado no canto superior esquerdo da vista
    // frontal, velocímetro ao centro inferior da mesma vista.
    panel_ = new TelemetryPanel(tab_quad_view_);
    panel_->adjustSize();
    panel_->show();
    panel_->raise();

    speed_ = new SpeedPanel(tab_quad_view_);
    speed_->adjustSize();
    speed_->show();
    speed_->raise();

    tab_widget_->addTab(tab_quad_view_, "Visão das Câmaras");

    // =====================================================================
    // ABA 2 — point cloud
    // =====================================================================
    auto* tab_pc = new QWidget();
    auto* layout_pc = new QVBoxLayout(tab_pc);
    layout_pc->setContentsMargins(0, 0, 0, 0);

    pc_widget_ = new PointCloudGLWidget(tab_pc);
    layout_pc->addWidget(pc_widget_);

    tab_widget_->addTab(tab_pc, "LiDAR 3D");

    connect(bridge_, &RosBridge::pointCloudReceived,
        pc_widget_, &PointCloudGLWidget::onPointCloudReceived,
        Qt::QueuedConnection);

    connect(pc_widget_, &PointCloudGLWidget::displayLatencyUpdated, this,
        [this](uint32_t id, double latency_ms) {
        bridge_->publishPointCloudMetrics(id, latency_ms);
    }, Qt::QueuedConnection);

    // =====================================================================
    // Telemetria
    // =====================================================================
    connect(bridge_, &RosBridge::telemetryReceived, this,
            [this](TelemetryState msg, int64_t rx_time_ns) {
        speed_->setVelocity(msg.velocity_kmh);
        panel_->onTelemetryReceived(msg);

        const int64_t display_time_ns = bridge_->nowNanoseconds();
        const double full_ms = bridge_->publishTelemetryGuiMetrics(
            msg.id, msg.origin_stamp, msg.e2e_command_ms, rx_time_ns, display_time_ns);
        panel_->setLoopLatency(full_ms);
    }, Qt::QueuedConnection);

    resize(1440, 900);

    // A geometria da grelha só está resolvida depois do primeiro ciclo de
    // eventos, por isso o posicionamento inicial é adiado.
    QTimer::singleShot(0, this, [this]() { reposition_overlays(); });
}

// ---------------------------------------------------------------------
void MainWindow::reposition_overlays()
{
    if (!cam_front_) return;
    const int padding = 20;

    if (panel_) {
        panel_->move(cam_front_->x() + padding,
                     cam_front_->y() + padding);
    }

    if (speed_) {
        speed_->move(cam_front_->x() + (cam_front_->width() - speed_->width()) / 2,
                     cam_front_->y() + cam_front_->height() - speed_->height() - padding);
    }
}

void MainWindow::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
    reposition_overlays();
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_1 && !is_single_camera_) {
        setSingleCameraMode(true);
    } else if (event->key() == Qt::Key_4 && is_single_camera_) {
        setSingleCameraMode(false);
    }
    QMainWindow::keyPressEvent(event);
}

void MainWindow::setSingleCameraMode(bool single)
{
    is_single_camera_ = single;
    auto* grid = qobject_cast<QGridLayout*>(tab_quad_view_->layout());
    if (!grid) return;

    if (single) {
        cam_left_->hide();
        cam_back_->hide();
        cam_right_->hide();

        grid->setColumnStretch(0, 0);
        grid->setColumnStretch(1, 1);
        grid->setColumnStretch(2, 0);
        grid->setRowStretch(0, 1);
        grid->setRowStretch(1, 0);
    } else {
        cam_left_->show();
        cam_back_->show();
        cam_right_->show();

        grid->setColumnStretch(0, 1);
        grid->setColumnStretch(1, 4);
        grid->setColumnStretch(2, 1);
        grid->setRowStretch(0, 2);
        grid->setRowStretch(1, 1);
    }

    // A posição da vista frontal mudou; reposicionar depois do relayout.
    QTimer::singleShot(50, this, [this]() { reposition_overlays(); });
}