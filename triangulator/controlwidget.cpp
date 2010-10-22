#include "controlwidget.h"

ControlWidget::ControlWidget(Triangulator* triangulator) : QDockWidget((QWidget*)triangulator)
{
    setupUi(this);

    // Wire up the time-box
//    connect(mSpinBoxTimeFactor, SIGNAL(valueChanged(double)), SIGNAL(timeFactorChanged(double)));
//    connect(mBtnStart, SIGNAL(clicked()), SLOT(slotSimulationStarted()));
//    connect(mBtnPause, SIGNAL(clicked()), SLOT(slotSimulationPaused()));

    connect(mBtnWptAdd, SIGNAL(clicked()), SLOT(slotAddWaypoint()));
    connect(mWayPoints, SIGNAL(itemChanged(QTableWidgetItem*)), SLOT(slotWayPointChanged(QTableWidgetItem*)));

    mTriangulator = triangulator;
//    mCoordinateConverter = mSimulator->mCoordinateConverter;
//    mBattery = mSimulator->mBattery;

//    connect(mBattery, SIGNAL(chargeStatusChanged(int)), SLOT(slotUpdateBattery(int)));

//    mLabelBatteryVoltage->setText(QString::number(mBattery->voltage(), 'g', 2) + " V");
//    mLabelBatteryEnergy->setText(QString::number(mBattery->capacity(), 'g', 2) + " Ah");

    mCompass->setStyle(new QPlastiqueStyle);

//    mDialogConfiguration = new DialogConfiguration(mSimulator);
}

ControlWidget::~ControlWidget()
{
//    delete mDialogConfiguration;
}

void ControlWidget::slotSimulationStarted()
{
//    mBtnPause->setEnabled(true);
//    mBtnStart->setEnabled(false);

    emit simulationStart();
}

void ControlWidget::slotSimulationPaused()
{
//    mBtnPause->setEnabled(false);
//    mBtnStart->setEnabled(true);

    emit simulationPause();
}


void ControlWidget::slotAddWaypoint()
{
    QVector3D wpt(mSpinBoxWptX->value(), mSpinBoxWptY->value(), mSpinBoxWptZ->value());
    emit addWayPoint(wpt);
}

void ControlWidget::slotUpdateBattery(const int chargeStateInPercent)
{
//    mLabelBatteryVoltageCurrent->setText(QString::number(mBattery->voltage(), 'g', 2) + " V");
//    mLabelBatteryEnergyCurrent->setText(QString::number(mBattery->energy(), 'g', 2) + " Ah");
}

void ControlWidget::slotUpdatePose(const QVector3D &position, const QQuaternion &rot)
{
    mLabelPoseOgreX->setText(QString("%1m").arg(position.x(), 3, 'f', 1, '0'));
    mLabelPoseOgreY->setText(QString("%1m").arg(position.y(), 3, 'f', 1, '0'));
    mLabelPoseOgreZ->setText(QString("%1m").arg(position.z(), 3, 'f', 1, '0'));

//    CoordinateGps wgs84 = mCoordinateConverter->convert(position);

//    mLabelPoseWgs84Longitude->setText(wgs84.formatGpsDegree(wgs84.longitude()));
//    mLabelPoseWgs84Latitude->setText(wgs84.formatGpsDegree(wgs84.latitude()));
//    mLabelPoseWgs84Elevation->setText(QString("%1m").arg(wgs84.elevation(), 3, 'f', 1, QLatin1Char('0')));

    Ogre::Quaternion rotation(rot.scalar(), rot.x(), rot.y(), rot.z());

    const int pitch = rotation.getPitch(false).valueDegrees();
    const int roll = rotation.getRoll(false).valueDegrees();
    const int yaw = rotation.getYaw(false).valueDegrees();

    QString deg;
    deg.sprintf("%c", 176);
    deg.prepend("%1");

    mLabelPitch->setText(deg.arg(pitch, 3, 10, QLatin1Char('0')));
    mLabelRoll->setText(deg.arg(roll, 3, 10, QLatin1Char('0')));
    mLabelYaw->setText(deg.arg(yaw, 3, 10, QLatin1Char('0')));

    mCompass->setValue(yaw+180);
}

void ControlWidget::slotUpdateDynamics(QVector3D linearVelocity)
{
    // flight dynamics
    mLabelSpeed->setText(QString::number(linearVelocity.length(), 'f', 4) + " m/s");
    mLabelSpeedVertical->setText(QString::number(linearVelocity.y(), 'f', 4) + " m/s");
    linearVelocity.setY(0.0);
    mLabelSpeedHorizontal->setText(QString::number(linearVelocity.length(), 'f', 4) + " m/s");
}

void ControlWidget::slotUpdateWayPoints(QList<QVector3D> waypoints)
{
    mWayPoints->clear();
    mWayPoints->setRowCount(waypoints.size());

    for(int i=0;i<waypoints.size();i++)
    {
        mWayPoints->setItem(0, 0, new QTableWidgetItem(QString::number(waypoints.at(i).x())));
        mWayPoints->setItem(0, 0, new QTableWidgetItem(QString::number(waypoints.at(i).y())));
        mWayPoints->setItem(0, 0, new QTableWidgetItem(QString::number(waypoints.at(i).z())));
    }
}

//void ControlWidget::slotWayPointChanged(QTableWidgetItem* item)
//{
//    if(item->column() == 0) mWayPoi;
//}
