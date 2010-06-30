#include "statuswidget.h"

StatusWidget::StatusWidget(Simulator *simulator) : QDockWidget((QWidget*)simulator)
{
    setupUi(this);

    // Wire up the time-box
    connect(mSpinBoxTimeFactor, SIGNAL(valueChanged(double)), SIGNAL(timeFactorChanged(double)));
    connect(mBtnStart, SIGNAL(clicked()), SLOT(slotSimulationStarted()));
    connect(mBtnPause, SIGNAL(clicked()), SLOT(slotSimulationPaused()));

    connect(mBtnConfiguration, SIGNAL(clicked()), SLOT(slotShowConfiguration()));

    mSimulator = simulator;
    mCoordinateConverter = mSimulator->mCoordinateConverter;
    mBattery = mSimulator->mBattery;

    connect(mBattery, SIGNAL(chargeStatusChanged(int)), SLOT(slotUpdateBattery(int)));

    mLabelBatteryVoltage->setText(QString::number(mBattery->voltage(), 'g', 2) + " V");
    mLabelBatteryEnergy->setText(QString::number(mBattery->capacity(), 'g', 2) + " Ah");

    mCompass->setStyle(new QPlastiqueStyle);

    mDialogConfiguration = new DialogConfiguration(mSimulator);
}

void StatusWidget::slotSimulationStarted()
{
    mBtnPause->setEnabled(true);
    mBtnStart->setEnabled(false);

    emit simulationStart();
}

void StatusWidget::slotSimulationPaused()
{
    mBtnPause->setEnabled(false);
    mBtnStart->setEnabled(true);

    emit simulationPause();
}

double StatusWidget::getTimeFactor() const
{
    return mSpinBoxTimeFactor->value();
}

void StatusWidget::slotShowConfiguration()
{
    mDialogConfiguration->show();
}

void StatusWidget::slotUpdateBattery(const int chargeStateInPercent)
{
    mLabelBatteryVoltageCurrent->setText(QString::number(mBattery->voltage(), 'g', 2) + " V");
    mLabelBatteryEnergyCurrent->setText(QString::number(mBattery->energy(), 'g', 2) + " Ah");
}

void StatusWidget::slotUpdateVisualization(QSize windowSize, int triangles, float fps)
{
    mLabelVisualizationSize->setText(QString::number(windowSize.width()) + " x " + QString::number(windowSize.height()));
    mLabelVisualizationTriangles->setText(QString::number(triangles));
    mLabelVisualizationFramerate->setText(QString::number(fps, 'g', 2));
}

void StatusWidget::slotUpdatePose(const Ogre::Vector3 &position, const Ogre::Quaternion &rotation)
{
    mLabelPoseOgreX->setText(QString("%1m").arg(position.x, 3, 'f', 1, '0'));
    mLabelPoseOgreY->setText(QString("%1m").arg(position.y, 3, 'f', 1, '0'));
    mLabelPoseOgreZ->setText(QString("%1m").arg(position.z, 3, 'f', 1, '0'));

    CoordinateGps wgs84 = mCoordinateConverter->convert(position);

    mLabelPoseWgs84Longitude->setText(wgs84.formatGpsDegree(wgs84.longitude()));
    mLabelPoseWgs84Latitude->setText(wgs84.formatGpsDegree(wgs84.latitude()));
    mLabelPoseWgs84Elevation->setText(QString("%1m").arg(wgs84.elevation(), 3, 'f', 1, QLatin1Char('0')));

    const int pitch = rotation.getPitch(true).valueDegrees();
    const int roll = rotation.getRoll(true).valueDegrees();
    const int yaw = rotation.getYaw(true).valueDegrees();

    QString deg;
    deg.sprintf("%c", 176);
    deg.prepend("%1");

    mLabelPitch->setText(deg.arg(pitch, 3, 10, QLatin1Char('0')));
    mLabelRoll->setText(deg.arg(roll, 3, 10, QLatin1Char('0')));
    mLabelYaw->setText(deg.arg(yaw, 3, 10, QLatin1Char('0')));

    mCompass->setValue(yaw+180);
}
