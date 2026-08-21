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
    void setLoopLatency(double latency_ms);
 
private:
    QLabel* lbl_gear_    = nullptr;
    QLabel* lbl_mode_    = nullptr;
    QLabel* lbl_engage_  = nullptr;
    QLabel* lbl_signal_  = nullptr;
    QLabel* lbl_network_ = nullptr;
    QLabel* lbl_latency_ = nullptr;
    QLabel* lbl_loop_    = nullptr;
 
    QWidget* make_card(const QString& title, QLabel*& value_label);
};
 
