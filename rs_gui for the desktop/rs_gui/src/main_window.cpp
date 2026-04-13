#include "main_window.hpp"
#include <QGridLayout>
#include <QWidget>
#include <QResizeEvent> // Necessário para o evento de redimensionamento

MainWindow::MainWindow(RosBridge* bridge, QWidget* parent)
: QMainWindow(parent), bridge_(bridge)
{
    setWindowTitle("Teleoperation HUD - Quad View");
    setStyleSheet("background-color: #11111b;");
    
    auto* central = new QWidget(this);
    auto* grid    = new QGridLayout(central);
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setSpacing(2); 
    setCentralWidget(central);

    // 1. CRIAR TODOS OS WIDGETS PRIMEIRO
    cam_left_  = new CameraGLWidget(this);
    cam_front_ = new CameraGLWidget(this); // Frente
    cam_back_  = new CameraGLWidget(this); // Trás
    cam_right_ = new CameraGLWidget(this);

    panel_ = new TelemetryPanel(central); 

    // 2. CONFIGURAR O GRID DAS CÂMARAS
    grid->addWidget(cam_left_,  0, 0, 2, 1);
    grid->addWidget(cam_front_, 0, 1, 1, 1); // Frente na parte superior central
    grid->addWidget(cam_back_,  1, 1, 1, 1); // Trás na parte inferior central
    grid->addWidget(cam_right_, 0, 2, 2, 1);

    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 4);
    grid->setColumnStretch(2, 1);
    grid->setRowStretch(0, 2);
    grid->setRowStretch(1, 1);

    // 3. GARANTIR QUE O PAINEL FLUTUANTE FICA VISÍVEL
    panel_->adjustSize(); 
    panel_->show();       
    panel_->raise();      

    // 4. FAZER AS CONEXÕES APENAS DEPOIS DE TUDO EXISTIR
    connect(bridge_, &RosBridge::telemetryReceived, panel_, &TelemetryPanel::onTelemetryReceived, Qt::QueuedConnection);
    
    connect(bridge_, &RosBridge::imageReceived, cam_left_,  &CameraGLWidget::onImageReceived, Qt::QueuedConnection);
    connect(bridge_, &RosBridge::imageReceived, cam_front_, &CameraGLWidget::onImageReceived, Qt::QueuedConnection);
    connect(bridge_, &RosBridge::imageReceived, cam_back_,  &CameraGLWidget::onImageReceived, Qt::QueuedConnection);
    connect(bridge_, &RosBridge::imageReceived, cam_right_, &CameraGLWidget::onImageReceived, Qt::QueuedConnection);

    // 5. REDIMENSIONAR NO FIM
    resize(1440, 900);
}

void MainWindow::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
    
    // Atualizado para usar cam_front_
    if (panel_ && cam_front_) {
        int padding = 20;
        
        int x = cam_front_->x() + padding;
        int y = cam_front_->y() + padding;
        
        panel_->move(x, y);
    }
}
