#include "pointcloud_gl_widget.hpp"
#include <cstring>

static const char* PC_VERT_SRC = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 u_MVP;
void main() {
    gl_Position = u_MVP * vec4(aPos, 1.0);
    gl_PointSize = 3.0; // Tamanho de cada ponto
}
)";

static const char* PC_FRAG_SRC = R"(
#version 330 core
out vec4 FragColor;
void main() {
    FragColor = vec4(0.0, 1.0, 0.8, 1.0); // Cor dos pontos (Ciano)
}
)";

static const char* CAR_VERT_SRC = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 u_MVP;
void main() { gl_Position = u_MVP * vec4(aPos, 1.0); }
)";

static const char* CAR_FRAG_SRC = R"(
#version 330 core
out vec4 FragColor;
void main() { FragColor = vec4(1.0, 0.2, 0.2, 1.0); } // Cor: Vermelho vivo
)";

static const float CAR_VERTICES[] = {
    // Base (Chão)
    -1.0f, -1.0f, 0.0f,   3.0f, -1.0f, 0.0f,
     3.0f, -1.0f, 0.0f,   3.0f,  1.0f, 0.0f,
     3.0f,  1.0f, 0.0f,  -1.0f,  1.0f, 0.0f,
    -1.0f,  1.0f, 0.0f,  -1.0f, -1.0f, 0.0f,
    // Topo (Tejadilho)
    -1.0f, -1.0f, 1.5f,   3.0f, -1.0f, 1.5f,
     3.0f, -1.0f, 1.5f,   3.0f,  1.0f, 1.5f,
     3.0f,  1.0f, 1.5f,  -1.0f,  1.0f, 1.5f,
    -1.0f,  1.0f, 1.5f,  -1.0f, -1.0f, 1.5f,
    // Pilares (Ligar Base ao Topo)
    -1.0f, -1.0f, 0.0f,  -1.0f, -1.0f, 1.5f,
     3.0f, -1.0f, 0.0f,   3.0f, -1.0f, 1.5f,
     3.0f,  1.0f, 0.0f,   3.0f,  1.0f, 1.5f,
    -1.0f,  1.0f, 0.0f,  -1.0f,  1.0f, 1.5f
};


PointCloudGLWidget::PointCloudGLWidget(QWidget* parent) : QOpenGLWidget(parent) {
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setDepthBufferSize(24); // Necessário para 3D
    setFormat(fmt);
}

PointCloudGLWidget::~PointCloudGLWidget() {
    makeCurrent();
    delete shader_;
    delete car_shader_; 
    if (gl33_) {
        gl33_->glDeleteVertexArrays(1, &vao_);
        gl33_->glDeleteBuffers(1, &vbo_);
        gl33_->glDeleteVertexArrays(1, &car_vao_); 
        gl33_->glDeleteBuffers(1, &car_vbo_);      
    }
    doneCurrent();
}

void PointCloudGLWidget::initializeGL() {
    gl33_ = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_3_3_Core>();
    gl33_->initializeOpenGLFunctions();
    
    gl33_->glClearColor(0.05f, 0.05f, 0.08f, 1.f);
    gl33_->glEnable(GL_DEPTH_TEST);
    gl33_->glEnable(GL_PROGRAM_POINT_SIZE);

    shader_ = new QOpenGLShaderProgram(this);
    shader_->addShaderFromSourceCode(QOpenGLShader::Vertex, PC_VERT_SRC);
    shader_->addShaderFromSourceCode(QOpenGLShader::Fragment, PC_FRAG_SRC);
    shader_->link();

    gl33_->glGenVertexArrays(1, &vao_);
    gl33_->glGenBuffers(1, &vbo_);

    car_shader_ = new QOpenGLShaderProgram(this);
    car_shader_->addShaderFromSourceCode(QOpenGLShader::Vertex, CAR_VERT_SRC);
    car_shader_->addShaderFromSourceCode(QOpenGLShader::Fragment, CAR_FRAG_SRC);
    car_shader_->link();
    if (!car_shader_->link()) {
        qWarning() << "Shader link failed:" << car_shader_->log();
    }

    gl33_->glGenVertexArrays(1, &car_vao_);
    gl33_->glGenBuffers(1, &car_vbo_);
    gl33_->glBindVertexArray(car_vao_);
    gl33_->glBindBuffer(GL_ARRAY_BUFFER, car_vbo_);
    gl33_->glBufferData(GL_ARRAY_BUFFER, sizeof(CAR_VERTICES), CAR_VERTICES, GL_STATIC_DRAW);
    gl33_->glEnableVertexAttribArray(0);
    gl33_->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    gl33_->glBindVertexArray(0);
}

void PointCloudGLWidget::resizeGL(int w, int h) {
    gl33_->glViewport(0, 0, w, h);
}

void PointCloudGLWidget::paintGL() {
    gl33_->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Calcular Matrizes da Câmara
    QMatrix4x4 proj, view, model;
    proj.perspective(45.0f, float(width()) / float(height() ? height() : 1), 0.1f, 1000.0f);
    
    view.translate(0.0f, 0.0f, -distance_);
    view.rotate(pitch_, 1.0f, 0.0f, 0.0f);
    view.rotate(yaw_, 0.0f, 1.0f, 0.0f);
    
    view.rotate(90.0f, 0.0f, 0.0f, 1.0f);

    QMatrix4x4 mvp = proj * view * model;

    shader_->bind();
    shader_->setUniformValue("u_MVP", mvp);

    gl33_->glBindVertexArray(vao_);

    size_t num_points = 0;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (dirty_) {
            gl33_->glBindBuffer(GL_ARRAY_BUFFER, vbo_);
            gl33_->glBufferData(GL_ARRAY_BUFFER, points_.size() * sizeof(float), points_.data(), GL_DYNAMIC_DRAW);
            dirty_ = false;
        }
        num_points = points_.size() / 3;
    }

    if (num_points > 0) {
        gl33_->glEnableVertexAttribArray(0);
        gl33_->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        gl33_->glDrawArrays(GL_POINTS, 0, num_points);
    }

    gl33_->glBindVertexArray(0);
    shader_->release();

    // === NOVO: DESENHAR O CARRO ===
    car_shader_->bind();
    car_shader_->setUniformValue("u_MVP", mvp); // Usamos a mesma matriz da câmara
    gl33_->glBindVertexArray(car_vao_);
    
    // Configurar a grossura da linha (opcional)
    gl33_->glLineWidth(2.0f); 
    
    // Desenhar os 24 vértices como Linhas (12 segmentos)
    gl33_->glDrawArrays(GL_LINES, 0, 24); 
    
    gl33_->glBindVertexArray(0);
    car_shader_->release();
    
}

void PointCloudGLWidget::onPointCloudReceived(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    const uint8_t* ptr = msg->data.data();
    size_t point_step = msg->point_step;
    size_t total_points = msg->width * msg->height;

    std::vector<float> new_points;
    new_points.reserve(total_points * 3);

    for (size_t i = 0; i < total_points; ++i) {
        float x, y, z;
        std::memcpy(&x, ptr + (i * point_step) + 0, 4);
        std::memcpy(&y, ptr + (i * point_step) + 4, 4);
        std::memcpy(&z, ptr + (i * point_step) + 8, 4);
        
        new_points.push_back(x);
        new_points.push_back(y);
        new_points.push_back(z);
    }

    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        points_ = std::move(new_points);
        dirty_ = true;
    }
    QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);
}

void PointCloudGLWidget::mousePressEvent(QMouseEvent *event) {
    last_mouse_pos_ = event->pos();
}

void PointCloudGLWidget::mouseMoveEvent(QMouseEvent *event) {
    if (event->buttons() & Qt::LeftButton) {
        int dx = event->x() - last_mouse_pos_.x();
        int dy = event->y() - last_mouse_pos_.y();
        yaw_ += dx * 0.5f;
        pitch_ += dy * 0.5f;
        last_mouse_pos_ = event->pos();
        update();
    }
}

void PointCloudGLWidget::wheelEvent(QWheelEvent *event) {
    distance_ -= event->angleDelta().y() * 0.01f;
    if (distance_ < 0.1f) distance_ = 0.1f;
    update();
}