#pragma once
#include <QWidget>
#include <QLabel>
 
class SpeedPanel : public QWidget {
    Q_OBJECT
public:
    explicit SpeedPanel(QWidget* parent = nullptr);
 
public slots:
    void setVelocity(float kmh);
 
private:
    QLabel* lbl_value_ = nullptr;
    QLabel* lbl_unit_  = nullptr;
};
 
