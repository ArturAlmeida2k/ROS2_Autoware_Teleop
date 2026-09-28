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

    stack_widget_ = new QStackedWidget(this);
    setCentralWidget(stack_widget_);

    // =====================================================================
    // PÁGINA 1 — vistas das câmaras
    // =====================================================================
    tab_quad_view_ = new QWidget();
    auto* grid = new QGridLayout(tab_quad_view_);
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setSpacing(2);

    // A porta define a câmara: 5007 front, 5008 left, 5009 back, 5010 right.
    cam_front_ = new CameraGLWidget(5007, "FRONT", tab_quad_view_);
    cam_left_  = new CameraGLWidget(5008, "LEFT", tab_quad_view_);
    cam_back_  = new CameraGLWidget(5009, "BACK", tab_quad_view_);
    cam_right_ = new CameraGLWidget(5010, "RIGHT", tab_quad_view_);

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

    stack_widget_->addWidget(tab_quad_view_);

    // =====================================================================
    // PÁGINA 2 — point cloud
    // =====================================================================
    tab_pointcloud_ = new QWidget();
    auto* layout_pc = new QVBoxLayout(tab_pointcloud_);
    layout_pc->setContentsMargins(0, 0, 0, 0);

    pc_widget_ = new PointCloudGLWidget(tab_pointcloud_);
    layout_pc->addWidget(pc_widget_);

    stack_widget_->addWidget(tab_pointcloud_);

    connect(bridge_, &RosBridge::pointCloudReceived,
        pc_widget_, &PointCloudGLWidget::onPointCloudReceived,
        Qt::QueuedConnection);

    connect(pc_widget_, &PointCloudGLWidget::displayLatencyUpdated, this,
        [this](uint32_t id, double latency_ms) {
        bridge_->publishPointCloudMetrics(id, latency_ms);
    }, Qt::QueuedConnection);

    // =====================================================================
    // Telemetria — sempre visível, independente da página atual
    // =====================================================================
    // Os dois flutuam sobre o stack_widget_ (não sobre uma página
    // específica), por isso ficam por cima tanto das câmaras como do
    // pointcloud, em vez de desaparecerem ao trocar de página.
    panel_ = new TelemetryPanel(stack_widget_);
    panel_->adjustSize();
    panel_->show();
    panel_->raise();

    speed_ = new SpeedPanel(stack_widget_);
    speed_->adjustSize();
    speed_->show();
    speed_->raise();

    // Telemetria orientada a eventos (ver RosBridge: coalescência, sem
    // backlog). Substitui o QTimer de 30 Hz, que mostrava sempre a última
    // amostra com 0–20 ms de idade (média ~10 ms só de espera).
    connect(bridge_, &RosBridge::telemetryUpdated,
            this, &MainWindow::onTelemetryUpdated, Qt::QueuedConnection);

    // A troca vídeo/pointcloud não é sensível à latência e não deve
    // depender de haver telemetria, por isso tem o seu próprio timer.
    auto* page_timer = new QTimer(this);
    connect(page_timer, &QTimer::timeout, this, &MainWindow::updateActivePage);
    page_timer->start(50);

    resize(1440, 900);

    // A geometria só está resolvida depois do primeiro ciclo de eventos,
    // por isso o posicionamento inicial do velocímetro é adiado.
    QTimer::singleShot(0, this, [this]() { reposition_overlays(); });
}

// ---------------------------------------------------------------------
void MainWindow::onTelemetryUpdated()
{
    // Limpar ANTES de ler: uma chegada a partir daqui gera novo evento,
    // por isso nunca se perde a última amostra.
    bridge_->clearTelemetryPending();

    TelemetryState msg;
    int64_t rx_time_ns;
    if (!bridge_->latestTelemetry(msg, rx_time_ns)) return;

    // Pode haver um evento extra com a mesma amostra (corrida entre o clear
    // e a leitura) — não redesenhar nem duplicar a métrica.
    if (msg.id == last_displayed_telemetry_id_) return;
    last_displayed_telemetry_id_ = msg.id;

    speed_->setVelocity(msg.velocity_kmh);
    speed_->setGear(msg.gear);
    speed_->setTurnSignal(msg.turn_signal);
    panel_->onTelemetryReceived(msg);

    const int64_t display_time_ns = bridge_->nowNanoseconds();
    const double full_ms = bridge_->publishTelemetryGuiMetrics(
        msg.id, msg.origin_stamp, msg.e2e_command_ms, rx_time_ns, display_time_ns);
    panel_->setLoopLatency(full_ms);
}

// ---------------------------------------------------------------------
void MainWindow::updateActivePage()
{
    QWidget* target = (bridge_->currentUplinkMode() == CmdEnums::UPLINK_POINTCLOUD)
                           ? tab_pointcloud_ : tab_quad_view_;
    if (stack_widget_->currentWidget() == target) return;

    stack_widget_->setCurrentWidget(target);
    // setCurrentWidget traz a página nova para cima de tudo o que está no
    // stack_widget_ — incluindo o panel_/speed_. Sem isto, a telemetria
    // fica tapada assim que troca de página.
    panel_->raise();
    speed_->raise();
    // A posição depende de qual página está ativa, por isso recalcula.
    reposition_overlays();
}

// ---------------------------------------------------------------------
void MainWindow::reposition_overlays()
{
    if (!speed_ || !panel_ || !stack_widget_) return;
    const int padding = 20;

    const bool on_pointcloud = (stack_widget_->currentWidget() == tab_pointcloud_);

    // --- Painel de telemetria (modo, latência, etc.) ---
    if (on_pointcloud) {
        panel_->move(stack_widget_->width() / 3, padding);
    } else if (is_single_camera_) {
        panel_->move(padding, padding);
    } else {
        panel_->move(stack_widget_->width() / 6 + padding, padding);
    }

    // --- Velocímetro ---
    int speed_y;
    if (!on_pointcloud && !is_single_camera_) {
        speed_y = static_cast<int>(stack_widget_->height() * 2/3) - speed_->height() - padding;
    } else {
        speed_y = stack_widget_->height() - speed_->height() - padding;
    }
    speed_->move((stack_widget_->width() - speed_->width()) / 2, speed_y);
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