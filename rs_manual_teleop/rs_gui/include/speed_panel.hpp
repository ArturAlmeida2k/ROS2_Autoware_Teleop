#pragma once
#include <QWidget>
#include <QLabel>

#include <teleop_msgs/msg/telemetry_state.hpp>
#include "teleop_msgs/msg/command_enums.hpp"

using TelemetryState = teleop_msgs::msg::TelemetryState;
using CmdEnums = teleop_msgs::msg::CommandEnums;

class SpeedPanel : public QWidget {
    Q_OBJECT
public:
    explicit SpeedPanel(QWidget* parent = nullptr);

public slots:
    void setVelocity(float kmh);
    void setGear(uint8_t gear);
    void setTurnSignal(uint8_t turn_signal);

private:
    QLabel* lbl_gear_  = nullptr;
    QLabel* lbl_value_ = nullptr;
    QLabel* lbl_unit_  = nullptr;
    QLabel* lbl_turn_  = nullptr;
};