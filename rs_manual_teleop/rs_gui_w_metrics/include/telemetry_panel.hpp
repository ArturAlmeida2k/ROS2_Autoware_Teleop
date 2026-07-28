#pragma once
#include <QWidget>
#include <QLabel>
#include <msg_manual_teleop/msg/telemetry_state.hpp>

using TelemetryState = msg_manual_teleop::msg::TelemetryState;

class TelemetryPanel : public QWidget {
    Q_OBJECT
public:
    explicit TelemetryPanel(QWidget* parent = nullptr);

public slots:
    void onTelemetryReceived(TelemetryState msg);
    void setVideoLatency(double latency_ms);

private:
    QLabel* lbl_velocity_;
    QLabel* lbl_gear_;
    QLabel* lbl_mode_;
    QLabel* lbl_engage_;
    QLabel* lbl_signal_;
    QLabel* lbl_latency_;

    QWidget* make_card(const QString& title, QLabel*& value_label);
    void     apply_mode_color(int mode);
};
