#pragma once
#include <QWidget>
#include <QLabel>

#include <msg_manual_teleop/msg/telemetry_state.hpp>
#include "msg_manual_teleop/msg/command_enums.hpp"

using TelemetryState = msg_manual_teleop::msg::TelemetryState;
using CmdEnums = msg_manual_teleop::msg::CommandEnums;

class SpeedPanel : public QWidget {
    Q_OBJECT
public:
    explicit SpeedPanel(QWidget* parent = nullptr);

public slots:
    void setVelocity(float kmh);
    void setGear(uint8_t gear);
    void setTurnSignal(uint8_t turn_signal, uint8_t hazard);

private:
    QLabel* lbl_gear_  = nullptr;
    QLabel* lbl_value_ = nullptr;
    QLabel* lbl_unit_  = nullptr;
    QLabel* lbl_turn_  = nullptr;
};