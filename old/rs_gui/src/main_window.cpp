#include "main_window.hpp"
#include <QGridLayout> 
#include <QVBoxLayout>
#include <QLabel>
#include <QWidget>
#include <QResizeEvent>


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
    cam_left_  = new CameraGLWidget(tab_quad_view_);
    cam_front_ = new CameraGLWidget(tab_quad_view_);
    cam_back_  = new CameraGLWidget(tab_quad_view_);
    cam_right_ = new CameraGLWidget(tab_quad_view_);
    
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
    connect(bridge_, &RosBridge::telemetryReceived, panel_, &TelemetryPanel::onTelemetryReceived, Qt::QueuedConnection);
    
    connect(bridge_, &RosBridge::imageLeftReceived,  cam_left_,  &CameraGLWidget::onImageReceived, Qt::QueuedConnection);
    connect(bridge_, &RosBridge::imageFrontReceived, cam_front_, &CameraGLWidget::onImageReceived, Qt::QueuedConnection);
    connect(bridge_, &RosBridge::imageBackReceived,  cam_back_,  &CameraGLWidget::onImageReceived, Qt::QueuedConnection);
    connect(bridge_, &RosBridge::imageRightReceived, cam_right_, &CameraGLWidget::onImageReceived, Qt::QueuedConnection);

    resize(1440, 900);
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