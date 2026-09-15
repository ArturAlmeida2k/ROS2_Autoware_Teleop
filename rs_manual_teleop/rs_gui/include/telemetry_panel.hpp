#pragma once
#include <QWidget>
#include <QLabel>

#include <teleop_msgs/msg/telemetry_state.hpp>

 
using TelemetryState = teleop_msgs::msg::TelemetryState;

 
class TelemetryPanel : public QWidget {
    Q_OBJECT
public:
    explicit TelemetryPanel(QWidget* parent = nullptr);
 
public slots:
    void onTelemetryReceived(TelemetryState msg);
    void setVideoLatency(double latency_ms);
    void setLoopLatency(double latency_ms);
 
private:
    QLabel* lbl_mode_    = nullptr;
    QLabel* lbl_engage_  = nullptr;
    QLabel* lbl_network_ = nullptr;
    QLabel* lbl_latency_ = nullptr;
    QLabel* lbl_loop_    = nullptr;
 
    QWidget* make_card(const QString& title, QLabel*& value_label);
};