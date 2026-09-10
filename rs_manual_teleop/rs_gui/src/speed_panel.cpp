#include "speed_panel.hpp"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFont>
#include <unordered_map>

SpeedPanel::SpeedPanel(QWidget* parent): QWidget(parent)
{
    setObjectName("speed");
    setStyleSheet(
        "QWidget#speed { background-color: rgba(13, 13, 26, 180); border-radius: 8px; }");
    setAttribute(Qt::WA_StyledBackground, true);

    // Largura fixa para o texto de gear/sinal (comprimento variável, ex.
    // "REVERSE" vs "PARK") não deslocar o velocímetro do centro.
    setFixedWidth(400);

    auto* row = new QHBoxLayout(this);
    row->setContentsMargins(16, 8, 16, 8);
    row->setSpacing(12);

    // Cor da legenda — mais clara que o cinza antigo (#585b70), que quase
    // não se via sobre o fundo escuro semi-transparente do HUD.
    static const char* CAPTION_STYLE = "color: #a6adc8; font-size: 11px; background: transparent;";

    // --- Gear, à esquerda ---
    auto* gear_col = new QVBoxLayout();
    gear_col->setSpacing(0);

    lbl_gear_ = new QLabel("–");
    QFont fg("Monospace", 16);
    fg.setBold(true);
    lbl_gear_->setFont(fg);
    lbl_gear_->setStyleSheet("color: #cdd6f4; background: transparent;");
    lbl_gear_->setAlignment(Qt::AlignCenter);
    lbl_gear_->setFixedWidth(100);

    auto* lbl_gear_caption = new QLabel("GEAR");
    lbl_gear_caption->setStyleSheet(CAPTION_STYLE);
    lbl_gear_caption->setAlignment(Qt::AlignCenter);
    lbl_gear_caption->setFixedWidth(100);

    gear_col->addWidget(lbl_gear_);
    gear_col->addWidget(lbl_gear_caption);

    // --- Velocidade, ao centro ---
    auto* speed_col = new QVBoxLayout();
    speed_col->setSpacing(0);

    lbl_value_ = new QLabel("–");
    QFont f("Monospace", 44);
    f.setBold(true);
    lbl_value_->setFont(f);
    lbl_value_->setStyleSheet("color: #cdd6f4; background: transparent;");
    lbl_value_->setAlignment(Qt::AlignCenter);

    lbl_unit_ = new QLabel("km/h");
    lbl_unit_->setStyleSheet(CAPTION_STYLE);
    lbl_unit_->setAlignment(Qt::AlignCenter);

    speed_col->addWidget(lbl_value_);
    speed_col->addWidget(lbl_unit_);

    // --- Pisca / hazard, à direita ---
    auto* turn_col = new QVBoxLayout();
    turn_col->setSpacing(0);

    lbl_turn_ = new QLabel("–");
    QFont ft("Monospace", 13);
    ft.setBold(true);
    lbl_turn_->setFont(ft);
    lbl_turn_->setStyleSheet("color: #444; background: transparent;");
    lbl_turn_->setAlignment(Qt::AlignCenter);
    lbl_turn_->setFixedWidth(100);

    auto* lbl_turn_caption = new QLabel("SINAL");
    lbl_turn_caption->setStyleSheet(CAPTION_STYLE);
    lbl_turn_caption->setAlignment(Qt::AlignCenter);
    lbl_turn_caption->setFixedWidth(100);

    turn_col->addWidget(lbl_turn_);
    turn_col->addWidget(lbl_turn_caption);

    // As 3 colunas têm alturas diferentes (o número da velocidade é maior
    // que o gear/sinal) — alinhar todas ao fundo da linha para as legendas
    // ficarem à mesma altura do "km/h".
    row->addLayout(gear_col);
    row->setAlignment(gear_col, Qt::AlignBottom);

    row->addLayout(speed_col, 1);
    row->setAlignment(speed_col, Qt::AlignBottom);

    row->addLayout(turn_col);
    row->setAlignment(turn_col, Qt::AlignBottom);
}

void SpeedPanel::setVelocity(float kmh)
{
    lbl_value_->setText(QString::number(kmh, 'f', 1));
}

void SpeedPanel::setGear(uint8_t gear)
{
    static const std::unordered_map<int, QString> gear_map = {
        {CmdEnums::GEAR_PARK, "PARK"}, {CmdEnums::GEAR_DRIVE, "DRIVE"}, {CmdEnums::GEAR_REVERSE, "REVERSE"}
    };
    lbl_gear_->setText(gear_map.count(gear) ? gear_map.at(gear) : "?");
}

void SpeedPanel::setTurnSignal(uint8_t turn_signal, uint8_t hazard)
{
    if (turn_signal == 2) {
        lbl_turn_->setText("◄ LEFT");
        lbl_turn_->setStyleSheet("color:#f39c12; background:transparent;");
    } else if (turn_signal == 3) {
        lbl_turn_->setText("RIGHT ►");
        lbl_turn_->setStyleSheet("color:#f39c12; background:transparent;");
    } else if (hazard == 2) {
        lbl_turn_->setText("HAZARD ⚠");
        lbl_turn_->setStyleSheet("color:#e67e22; background:transparent;");
    } else {
        lbl_turn_->setText("–");
        lbl_turn_->setStyleSheet("color:#444; background:transparent;");
    }
}