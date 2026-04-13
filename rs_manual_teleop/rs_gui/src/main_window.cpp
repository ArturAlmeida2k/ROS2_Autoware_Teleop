#include "main_window.hpp"
#include <QGridLayout> 
#include <QWidget>

MainWindow::MainWindow(RosBridge* bridge, QWidget* parent)
: QMainWindow(parent), bridge_(bridge)
{
    setWindowTitle("Teleoperation HUD - Quad View");
    setStyleSheet("background-color: #11111b;");
    resize(1440, 900);

    auto* central = new QWidget(this);
    auto* grid    = new QGridLayout(central);
    
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setSpacing(2); 
    setCentralWidget(central);

    // Instanciar as 4 câmaras
    cam_left_   = new CameraGLWidget(this);
    cam_front_   = new CameraGLWidget(this);
    cam_back_ = new CameraGLWidget(this);
    cam_right_  = new CameraGLWidget(this);

    // Adicionar ao Grid: addWidget(widget, linha, coluna, span_linhas, span_colunas)
    
    // 1/6 Mais à Esquerda (linha 0, coluna 0, ocupa 2 linhas, ocupa 1 coluna)
    grid->addWidget(cam_left_,   0, 0, 2, 1);
    
    // 4/6 do Meio (Cima) (linha 0, coluna 1, ocupa 1 linha, ocupa 1 coluna)
    grid->addWidget(cam_front_,   0, 1, 1, 1);
    
    // 4/6 do Meio (Baixo) (linha 1, coluna 1, ocupa 1 linha, ocupa 1 coluna)
    grid->addWidget(cam_back_, 1, 1, 1, 1);
    
    // 1/6 Mais à Direita (linha 0, coluna 2, ocupa 2 linhas, ocupa 1 coluna)
    grid->addWidget(cam_right_,  0, 2, 2, 1);

    // --- DEFINIR AS PROPORÇÕES ---

    // Colunas (Proporção 1:4:1)
    grid->setColumnStretch(0, 1); // Esquerda: 1/6
    grid->setColumnStretch(1, 4); // Meio: 4/6
    grid->setColumnStretch(2, 1); // Direita: 1/6

    // Linhas (Proporção 2:1 para a coluna central)
    // As laterais não são afetadas porque têm span de 2 (ocupam as duas linhas)
    grid->setRowStretch(0, 2);    // Cima: 2/3
    grid->setRowStretch(1, 1);    // Baixo: 1/3

    // Ligar o painel (se for mostrado noutra janela)
    connect(bridge_, &RosBridge::telemetryReceived,
            panel_,  &TelemetryPanel::onTelemetryReceived,
            Qt::QueuedConnection);

    // Exemplo de ligação de imagem: Ligar todas à mesma imagem para testares o layout
    // No futuro, terás de criar sinais separados no ros_bridge (ex: imageFrontReceived, imageRearReceived, etc.)
    connect(bridge_, &RosBridge::imageReceived, cam_left_,   &CameraGLWidget::onImageReceived, Qt::QueuedConnection);
    connect(bridge_, &RosBridge::imageReceived, cam_front_,   &CameraGLWidget::onImageReceived, Qt::QueuedConnection);
    connect(bridge_, &RosBridge::imageReceived, cam_back_, &CameraGLWidget::onImageReceived, Qt::QueuedConnection);
    connect(bridge_, &RosBridge::imageReceived, cam_right_,  &CameraGLWidget::onImageReceived, Qt::QueuedConnection);


    // 1. Passamos o widget 'central' como parent
    panel_ = new TelemetryPanel(central); 
    
    // 2. Trazemos o painel para a camada da frente (acima do vídeo)
    panel_->raise(); 

    // As conexões continuam iguais:
    connect(bridge_, &RosBridge::telemetryReceived,
            panel_,  &TelemetryPanel::onTelemetryReceived,
            Qt::QueuedConnection);
}

void MainWindow::resizeEvent(QResizeEvent* event)
{
    // Chama a função base do Qt primeiro para ele fazer os cálculos normais
    QMainWindow::resizeEvent(event);
    
    // Se o painel e a câmara já existirem
    if (panel_ && cam_main_) {
        // Define uma margem de 20 píxeis
        int padding = 20;
        
        // Calcula a posição X e Y (relativa à janela principal) 
        // para o canto superior esquerdo da câmara central
        int x = cam_main_->x() + padding;
        int y = cam_main_->y() + padding;
        
        // Move o painel flutuante para a nova coordenada
        panel_->move(x, y);
    }
}