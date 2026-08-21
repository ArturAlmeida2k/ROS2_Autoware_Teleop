#include "telemetry_panel.hpp"
#include <QVBoxLayout>
#include <QFrame>
#include <QFont>
#include <unordered_map>

static const char* PANEL_STYLE = R"(
    QWidget#panel {
        background-color: rgba(13, 13, 26, 180);
        border-radius: 8px;
    }
    QFrame#card {
        background-color: rgba(30, 30, 46, 100);
        border: 1px solid rgba(49, 50, 68, 150);
        border-radius: 6px;
    }
)";

static QLabel* make_title(const QString& text) {
    auto* l = new QLabel(text);
    l->setStyleSheet("color: #585b70; font-size: 9px; background: transparent;");
    l->setAlignment(Qt::AlignCenter);
    return l;
}

static QLabel* make_value(int size = 14) {
    auto* l = new QLabel("–");
    QFont f("Monospace", size);
    f.setBold(true);
    l->setFont(f);
    l->setStyleSheet("color: #cdd6f4; background: transparent;");
    l->setAlignment(Qt::AlignCenter);
    return l;
}

TelemetryPanel::TelemetryPanel(QWidget* parent)
: QWidget(parent)
{
    setObjectName("panel");
    setStyleSheet(PANEL_STYLE);
    setFixedWidth(220);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(12, 16, 12, 16);
    layout->setSpacing(8);

    layout->addWidget(make_card("GEAR",                lbl_gear_    = make_value(18)));
    layout->addWidget(make_card("MODO",                lbl_mode_    = make_value(18)));
    layout->addWidget(make_card("ENGAGE",              lbl_engage_  = make_value(13)));
    layout->addWidget(make_card("SINAL",               lbl_signal_  = make_value(13)));
    layout->addWidget(make_card("LIGAÇÃO",             lbl_network_ = make_value(13)));
    layout->addWidget(make_card("LATÊNCIA VÍDEO (ms)", lbl_latency_ = make_value(18)));
    layout->addWidget(make_card("CICLO COMPLETO (ms)", lbl_loop_    = make_value(18)));
    layout->addStretch();
}

QWidget* TelemetryPanel::make_card(const QString& title, QLabel*& value_label)
{
    auto* frame = new QFrame();
    frame->setObjectName("card");
    auto* vl = new QVBoxLayout(frame);
    vl->setContentsMargins(8, 6, 8, 6);
    vl->setSpacing(2);
    vl->addWidget(make_title(title));
    vl->addWidget(value_label);
    return frame;
}

void TelemetryPanel::onTelemetryReceived(TelemetryState msg)
{
    // Gear
    static const std::unordered_map<int, QString> gear_map = {
        {0, "PARK"}, {1, "DRIVE"}, {2, "REVERSE"}
    };
    lbl_gear_->setText(gear_map.count(msg.gear) ? gear_map.at(msg.gear) : "?");

    // Modo de operação do Autoware
    static const std::unordered_map<int, std::pair<QString, QString>> mode_map = {
        {0, {"UNKNOWN", "#888888"}},
        {1, {"STOP",    "#e74c3c"}},
        {2, {"AUTO",    "#2ecc71"}},
        {3, {"LOCAL",   "#248ed5"}},
        {4, {"REMOTE",  "#9b59b6"}},
    };
    if (mode_map.count(msg.mode)) {
        auto [name, color] = mode_map.at(msg.mode);
        lbl_mode_->setText(name);
        lbl_mode_->setStyleSheet(
            QString("color: %1; background: transparent; font-weight: bold;").arg(color));
    }

    // Engage
    const bool eng = msg.engaged;
    lbl_engage_->setText(eng ? "● ENGAGED" : "○ DISENGAGED");
    lbl_engage_->setStyleSheet(
        QString("color: %1; background: transparent;").arg(eng ? "#2ecc71" : "#e74c3c"));

    // Piscas e pisca-alerta
    if (msg.turn_signal == 2) {
        lbl_signal_->setText("◄ LEFT");
        lbl_signal_->setStyleSheet("color:#f39c12; background:transparent;");
    } else if (msg.turn_signal == 3) {
        lbl_signal_->setText("RIGHT ►");
        lbl_signal_->setStyleSheet("color:#f39c12; background:transparent;");
    } else if (msg.hazard == 2) {
        lbl_signal_->setText("HAZARD ⚠");
        lbl_signal_->setStyleSheet("color:#e67e22; background:transparent;");
    } else {
        lbl_signal_->setText("–");
        lbl_signal_->setStyleSheet("color:#444; background:transparent;");
    }

    // Estado da ligação, avaliado no veículo pelo Topic Monitor.
    switch (msg.network_state) {
        case 0:
            lbl_network_->setText("● OK");
            lbl_network_->setStyleSheet("color:#2ecc71; background:transparent;");
            break;
        case 1:
            lbl_network_->setText("▲ DEGRADED");
            lbl_network_->setStyleSheet(
                "color:#f39c12; background:transparent; font-weight:bold;");
            break;
        case 2:
            lbl_network_->setText("✕ LOST");
            lbl_network_->setStyleSheet(
                "color:#e74c3c; background:transparent; font-weight:bold;");
            break;
        default:
            lbl_network_->setText("–");
            lbl_network_->setStyleSheet("color:#444; background:transparent;");
            break;
    }
}

void TelemetryPanel::setVideoLatency(double latency_ms)
{
    lbl_latency_->setText(QString::number(latency_ms, 'f', 1));

    if (latency_ms < 100.0) {
        lbl_latency_->setStyleSheet("color: #2ecc71; background: transparent;");
    } else if (latency_ms < 150.0) {
        lbl_latency_->setStyleSheet("color: #f39c12; background: transparent;");
    } else {
        lbl_latency_->setStyleSheet(
            "color: #e74c3c; background: transparent; font-weight: bold;");
    }
}

void TelemetryPanel::setLoopLatency(double latency_ms)
{
    lbl_loop_->setText(QString::number(latency_ms, 'f', 1));

    if (latency_ms < 50.0) {
        lbl_loop_->setStyleSheet("color: #2ecc71; background: transparent;");
    } else if (latency_ms < 100.0) {
        lbl_loop_->setStyleSheet("color: #f39c12; background: transparent;");
    } else {
        lbl_loop_->setStyleSheet(
            "color: #e74c3c; background: transparent; font-weight: bold;");
    }
}