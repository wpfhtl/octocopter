#ifndef STATUSWIDGET_H
#define STATUSWIDGET_H

#include <QDockWidget>
#include "ui_statuswidget.h"
#include "battery.h"

class StatusWidget : public QDockWidget, public Ui::DockWidget
{
Q_OBJECT
private:
    Battery* mBattery;
public:
    StatusWidget(QWidget *parent = 0, Battery* battery = 0);

signals:

private slots:
    void slotUpdateBattery(const int chargeStateInPercent);
    void slotUpdateVisualization(QSize windowSize, int triangles, float fps);

};

#endif // STATUSWIDGET_H
