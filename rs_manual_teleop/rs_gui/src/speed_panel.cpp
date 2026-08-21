#include "speed_panel.hpp"
#include <QVBoxLayout>
#include <QFont>

SpeedPanel::SpeedPanel(QWidget* parent): QWidget(parent)
{
    setObjectName("speed");
    setStyleSheet(
        "QWidget#speed { background-color: rgba(13, 13, 26, 180); border-radius: 8px; }");
    setFixedWidth(180);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(12, 8, 12, 8);
    layout->setSpacing(0);

    setAttribute(Qt::WA_StyledBackground, true);
    
    lbl_value_ = new QLabel("–");
    QFont f("Monospace", 44);
    f.setBold(true);
    lbl_value_->setFont(f);
    lbl_value_->setStyleSheet("color: #cdd6f4; background: transparent;");
    lbl_value_->setAlignment(Qt::AlignCenter);

    lbl_unit_ = new QLabel("km/h");
    lbl_unit_->setStyleSheet("color: #585b70; font-size: 11px; background: transparent;");
    lbl_unit_->setAlignment(Qt::AlignCenter);

    layout->addWidget(lbl_value_);
    layout->addWidget(lbl_unit_);
}

void SpeedPanel::setVelocity(float kmh)
{
    lbl_value_->setText(QString::number(kmh, 'f', 1));
}