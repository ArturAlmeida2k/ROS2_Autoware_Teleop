#include "main_window.hpp"
#include <QGridLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QWidget>
#include <QResizeEvent>
#include <QTimer>
#include <QQuickItem>
#include <QUrl>

// Helper que monta um QQuickWidget (carrega o QML), extrai o QQuickItem
// "videoItem" lá dentro, e cria o CameraStreamManager ligado a ele.
QQuickWidget* MainWindow::makeCameraView(int port, QWidget* parent, CameraStreamManager** out_manager)
{
    auto* widget = new QQuickWidget(parent);
    widget->setSource(QUrl("qrc:/qml/CameraVideoItem.qml"));
    widget->setResizeMode(QQuickWidget::SizeRootObjectToView);

    QQuickItem *root = widget->rootObject();
    QQuickItem *videoItem = root ? root->findChild<QQuickItem*>("videoItem") : nullptr;

    *out_manager = new CameraStreamManager(videoItem, port, widget);

    return widget;
}

MainWindow::MainWindow(RosBridge* bridge, QWidget* parent)
: QMainWindow(parent), bridge_(bridge)
{
    setWindowTitle("Teleoperation HUD - Multi-Tab");
    setStyleSheet("background-color: #11111b;");

    tab_widget_ = new QTabWidget(this);
    setCentralWidget(tab_widget_);

    tab_widget_->setStyleSheet(
        "QTabBar::tab { background: #1e1e2e; color: #cdd6f4; padding: 8px 16px; border-radius: 4px; margin: 2px; }"
        "QTabBar::tab:selected { background: #313244; font-weight: bold; }"
        "QTabWidget::pane { border: none; }"
    );

    // ==========================================
    // ABA 1: O QUAD VIEW (As 4 Câmaras)
    // ==========================================
    tab_quad_view_ = new QWidget();
    auto* grid = new QGridLayout(tab_quad_view_);
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setSpacing(2);

    // TROCADO: cada câmara agora é um QQuickWidget + CameraStreamManager
    cam_left_widget_  = makeCameraView(5008, tab_quad_view_, &cam_left_mgr_);
    cam_front_widget_ = makeCameraView(5007, tab_quad_view_, &cam_front_mgr_);
    cam_back_widget_  = makeCameraView(5009, tab_quad_view_, &cam_back_mgr_);
    cam_right_widget_ = makeCameraView(5010, tab_quad_view_, &cam_right_mgr_);

    // ==========================================
    // Sinal de latência — agora vem do CameraStreamManager, não do widget
    // ==========================================
    connect(cam_front_mgr_, &CameraStreamManager::latencyUpdated, this, [this](uint64_t frame_id, double latency_ms) {
        bridge_->publishFrontCameraMetrics(static_cast<uint32_t>(frame_id), latency_ms);
        if (panel_) {
            panel_->setVideoLatency(latency_ms);
        }
    }, Qt::QueuedConnection);

    panel_ = new TelemetryPanel(tab_quad_view_);

    // TROCADO: usa os _widget_ (QQuickWidget) no grid, em vez dos antigos CameraGLWidget
    grid->addWidget(cam_left_widget_,   0, 0, 2, 1);
    grid->addWidget(cam_front_widget_,  0, 1, 1, 1);
    grid->addWidget(cam_back_widget_,   1, 1, 1, 1);
    grid->addWidget(cam_right_widget_,  0, 2, 2, 1);

    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 4);
    grid->setColumnStretch(2, 1);
    grid->setRowStretch(0, 2);
    grid->setRowStretch(1, 1);

    panel_->adjustSize();
    panel_->show();
    panel_->raise();

    tab_widget_->addTab(tab_quad_view_, "Visão das Câmaras");

    // ==========================================
    // ABA 2: O POINT CLOUD 3D (sem alterações)
    // ==========================================
    auto* tab_nova = new QWidget();
    auto* layout_nova = new QVBoxLayout(tab_nova);
    layout_nova->setContentsMargins(0, 0, 0, 0);

    pc_widget_ = new PointCloudGLWidget(tab_nova);
    layout_nova->addWidget(pc_widget_);

    tab_widget_->addTab(tab_nova, "LiDAR 3D");

    connect(bridge_, &RosBridge::pointCloudReceived, pc_widget_, &PointCloudGLWidget::onPointCloudReceived, Qt::QueuedConnection);

    // ==========================================
    // LIGAÇÕES E REDIMENSIONAMENTO
    // ==========================================
    connect(bridge_, &RosBridge::telemetryReceived, this, [this](TelemetryState msg, int64_t rx_time_ns) {
        panel_->onTelemetryReceived(msg);
        int64_t display_time_ns = rclcpp::Clock().now().nanoseconds();
        bridge_->publishTelemetryGuiMetrics(msg.id, msg.origin_stamp, msg.e2e_command_ms, rx_time_ns, display_time_ns);
    }, Qt::QueuedConnection);

    resize(1440, 900);

    // TROCADO: cam_front_ -> cam_front_widget_ (mas x()/y() funcionam igual, é QWidget)
    QTimer::singleShot(0, this, [this]() {
        if (panel_ && cam_front_widget_) {
            int padding = 20;
            panel_->move(cam_front_widget_->x() + padding, cam_front_widget_->y() + padding);
        }
    });
}

void MainWindow::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);

    if (panel_ && cam_front_widget_) {
        int padding = 20;
        int x = cam_front_widget_->x() + padding;
        int y = cam_front_widget_->y() + padding;
        panel_->move(x, y);
    }
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_1 && !is_single_camera_) {
        setSingleCameraMode(true);
    }
    else if (event->key() == Qt::Key_4 && is_single_camera_) {
        setSingleCameraMode(false);
    }
    QMainWindow::keyPressEvent(event);
}

void MainWindow::setSingleCameraMode(bool single)
{
    is_single_camera_ = single;
    auto* grid = qobject_cast<QGridLayout*>(tab_quad_view_->layout());

    if (!grid) return;

    // TROCADO: hide()/show() nos _widget_ (QQuickWidget), funciona igual a antes
    if (single) {
        cam_left_widget_->hide();
        cam_back_widget_->hide();
        cam_right_widget_->hide();

        grid->setColumnStretch(0, 0);
        grid->setColumnStretch(1, 1);
        grid->setColumnStretch(2, 0);
        grid->setRowStretch(0, 1);
        grid->setRowStretch(1, 0);
    } else {
        cam_left_widget_->show();
        cam_back_widget_->show();
        cam_right_widget_->show();

        grid->setColumnStretch(0, 1);
        grid->setColumnStretch(1, 4);
        grid->setColumnStretch(2, 1);
        grid->setRowStretch(0, 2);
        grid->setRowStretch(1, 1);
    }

    QTimer::singleShot(50, this, [this]() {
        if (panel_ && cam_front_widget_) {
            int padding = 20;
            panel_->move(cam_front_widget_->x() + padding, cam_front_widget_->y() + padding);
        }
    });
}