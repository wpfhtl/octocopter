#ifndef STATUSWIDGET_H
#define STATUSWIDGET_H

#include <QtGui>
 #include <QPlastiqueStyle>
#include "coordinateconverter.h"
#include "ui_statuswidget.h"
#include "battery.h"

#define QT_USE_FAST_CONCATENATION
#define QT_USE_FAST_OPERATOR_PLUS

class Battery;

class StatusWidget : public QDockWidget, public Ui::DockWidget
{
Q_OBJECT
private:
    Battery* mBattery;
    CoordinateConverter *mCoordinateConverter;

public:
    StatusWidget(QWidget *parent, Battery* battery, CoordinateConverter* coordinateConverter);

signals:

private slots:
    void slotUpdateBattery(const int chargeStateInPercent);
    void slotUpdateVisualization(QSize windowSize, int triangles, float fps);
    void slotUpdatePose(const Ogre::Vector3 &position, const Ogre::Quaternion &rotation);

};

#endif // STATUSWIDGET_H
