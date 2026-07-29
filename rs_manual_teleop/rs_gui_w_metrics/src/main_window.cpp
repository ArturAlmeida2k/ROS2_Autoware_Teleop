#include "main_window.hpp"
#include <QGridLayout> 
#include <QVBoxLayout>
#include <QLabel>
#include <QWidget>
#include <QResizeEvent>
#include <QTimer>


MainWindow::MainWindow(RosBridge* bridge, QWidget* parent)
: QMainWindow(parent), bridge_(bridge)
{
    setWindowTitle("Teleoperation HUD - Multi-Tab");
    setStyleSheet("background-color: #11111b;");

    // 1. CRIAR O SISTEMA DE ABAS E DEFINIR COMO CENTRAL
    tab_widget_ = new QTabWidget(this);
    setCentralWidget(tab_widget_);

    // (Opcional) Estilizar as abas para manter o design escuro
    tab_widget_->setStyleSheet(
        "QTabBar::tab { background: #1e1e2e; color: #cdd6f4; padding: 8px 16px; border-radius: 4px; margin: 2px; }"
        "QTabBar::tab:selected { background: #313244; font-weight: bold; }"
        "QTabWidget::pane { border: none; }" // Remove a borda padrão do Qt
    );

    // ==========================================
    // ABA 1: O QUAD VIEW (As 4 Câmaras)
    // ==========================================
    tab_quad_view_ = new QWidget();
    auto* grid = new QGridLayout(tab_quad_view_);
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setSpacing(2); 

    // O pai agora é o tab_quad_view_
    cam_left_  = new CameraGLWidget(5008, tab_quad_view_);
    cam_front_ = new CameraGLWidget(5007, tab_quad_view_);
    cam_back_  = new CameraGLWidget(5009, tab_quad_view_);
    cam_right_ = new CameraGLWidget(5010, tab_quad_view_);   

    // ==========================================
    // NOVA ALTERAÇÃO: Ligar os sinais de latência
    // ==========================================
    
    connect(cam_front_, &CameraGLWidget::latencyUpdated, this, [this](uint64_t frame_id, double latency_ms) {
        // 1. Envia para o ROS 2
        bridge_->publishFrontCameraMetrics(static_cast<uint32_t>(frame_id), latency_ms);
        
        // 2. Atualiza no HUD
        if (panel_) {
            panel_->setVideoLatency(latency_ms);
        }
    }, Qt::QueuedConnection);

    panel_ = new TelemetryPanel(tab_quad_view_); 

    grid->addWidget(cam_left_,   0, 0, 2, 1);
    grid->addWidget(cam_front_,  0, 1, 1, 1);
    grid->addWidget(cam_back_,   1, 1, 1, 1);
    grid->addWidget(cam_right_,  0, 2, 2, 1);

    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 4);
    grid->setColumnStretch(2, 1);
    grid->setRowStretch(0, 2);
    grid->setRowStretch(1, 1);

    panel_->adjustSize(); 
    panel_->show();       
    panel_->raise();      

    // Adicionar a aba ao sistema com um título
    tab_widget_->addTab(tab_quad_view_, "Visão das Câmaras");

    // ==========================================
    // ABA 2: O POINT CLOUD 3D
    // ==========================================
    auto* tab_nova = new QWidget();
    auto* layout_nova = new QVBoxLayout(tab_nova);
    layout_nova->setContentsMargins(0, 0, 0, 0); // Sem margens
    
    // Instanciar o widget 3D
    pc_widget_ = new PointCloudGLWidget(tab_nova);
    layout_nova->addWidget(pc_widget_);

    tab_widget_->addTab(tab_nova, "LiDAR 3D");

    // Ligar o sinal do ROS ao widget 3D
    connect(bridge_, &RosBridge::pointCloudReceived, pc_widget_, &PointCloudGLWidget::onPointCloudReceived, Qt::QueuedConnection);

    // ==========================================
    // LIGAÇÕES E REDIMENSIONAMENTO
    // ==========================================    

    connect(bridge_, &RosBridge::telemetryReceived, this, [this](TelemetryState msg, int64_t rx_time_ns) {
        panel_->onTelemetryReceived(msg);
        int64_t display_time_ns = rclcpp::Clock().now().nanoseconds();
        
        // ATENÇÃO: Assumi que o nome da variável na tua mensagem é 'e2e_command'. 
        // Se no teu ficheiro .msg for 'e2e_command_ms' ou algo parecido, altera aqui:
        bridge_->publishTelemetryGuiMetrics(msg.id, msg.origin_stamp, msg.e2e_command_ms, rx_time_ns, display_time_ns);
    }, Qt::QueuedConnection);

    resize(1440, 900);
    
    QTimer::singleShot(0, this, [this]() {
        if (panel_ && cam_front_) {
            int padding = 20;
            panel_->move(cam_front_->x() + padding, cam_front_->y() + padding);
        }
    });
}

void MainWindow::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
    
    if (panel_ && cam_front_) {
        int padding = 20;
        int x = cam_front_->x() + padding;
        int y = cam_front_->y() + padding;
        
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
    
    // Passa o evento para a classe base para não bloquear outros atalhos do Qt
    QMainWindow::keyPressEvent(event); 
}

void MainWindow::setSingleCameraMode(bool single)
{
    is_single_camera_ = single;
    auto* grid = qobject_cast<QGridLayout*>(tab_quad_view_->layout());
    
    if (!grid) return;

    if (single) {
        // Ocultar as câmaras secundárias
        cam_left_->hide();
        cam_back_->hide();
        cam_right_->hide();

        // Alterar os "pesos" da grelha para a coluna 1 e linha 0 ocuparem 100% do espaço
        grid->setColumnStretch(0, 0);
        grid->setColumnStretch(1, 1);
        grid->setColumnStretch(2, 0);
        grid->setRowStretch(0, 1);
        grid->setRowStretch(1, 0);
    } else {
        // Mostrar as câmaras secundárias
        cam_left_->show();
        cam_back_->show();
        cam_right_->show();

        // Restaurar os pesos originais do Quad View
        grid->setColumnStretch(0, 1);
        grid->setColumnStretch(1, 4);
        grid->setColumnStretch(2, 1);
        grid->setRowStretch(0, 2);
        grid->setRowStretch(1, 1);
    }

    // Como a posição da cam_front_ acabou de mudar, precisamos de reposicionar o HUD
    QTimer::singleShot(50, this, [this]() {
        if (panel_ && cam_front_) {
            int padding = 20;
            panel_->move(cam_front_->x() + padding, cam_front_->y() + padding);
        }
    });
}