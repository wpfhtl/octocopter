#include "controlwidget.h"

ControlWidget::ControlWidget(BaseStation* baseStation) : QDockWidget((QWidget*)baseStation)
{
    setupUi(this);

    // Wire up the time-box
//    connect(mSpinBoxTimeFactor, SIGNAL(valueChanged(double)), SIGNAL(timeFactorChanged(double)));
//    connect(mBtnStart, SIGNAL(clicked()), SLOT(slotSimulationStarted()));
//    connect(mBtnPause, SIGNAL(clicked()), SLOT(slotSimulationPaused()));

    initWayPointTable();

    connect(mBtnWptPrepend, SIGNAL(clicked()), SLOT(slotWayPointPrepend()));
    connect(mBtnWptAppend, SIGNAL(clicked()), SLOT(slotWayPointAppend()));
    connect(mBtnWptDelete, SIGNAL(clicked()), SLOT(slotWayPointDelete()));

    connect(mBtnWptUp, SIGNAL(clicked()), SLOT(slotWayPointUp()));
    connect(mBtnWptDown, SIGNAL(clicked()), SLOT(slotWayPointDown()));

    connect(mWayPointTable, SIGNAL(cellChanged(int,int)), SLOT(slotWayPointChange(int,int)));

    connect(mBtnGenerateWaypoints, SIGNAL(clicked()), SIGNAL(generateWaypoints()));

    connect(mBtnSetScanVolume, SIGNAL(clicked()), SLOT(slotSetScanVolume()));

    mBaseStation = baseStation;
//    mCoordinateConverter = mSimulator->mCoordinateConverter;
//    mBattery = mSimulator->mBattery;

//    connect(mBattery, SIGNAL(chargeStatusChanged(int)), SLOT(slotUpdateBattery(int)));

//    mLabelBatteryVoltage->setText(QString::number(mBattery->voltage(), 'g', 2) + " V");
//    mLabelBatteryEnergy->setText(QString::number(mBattery->capacity(), 'g', 2) + " Ah");

    mCompass->setStyle(new QPlastiqueStyle);

//    mDialogConfiguration = new DialogConfiguration(mSimulator);

    resize(minimumSizeHint());
}

ControlWidget::~ControlWidget()
{
//    delete mDialogConfiguration;
}

void ControlWidget::initWayPointTable()
{
    mWayPointTable->resizeColumnsToContents();
    QStringList headers;
    headers << "X" << "Y" << "Z";
    mWayPointTable->setHorizontalHeaderLabels(headers);

    mWayPointTable->setColumnWidth(0, 53);
    mWayPointTable->setColumnWidth(1, 53);
    mWayPointTable->setColumnWidth(2, 53);
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


void ControlWidget::slotWayPointPrepend()
{
    QVector3D wpt(mSpinBoxWptX->value(), mSpinBoxWptY->value(), mSpinBoxWptZ->value());
    QList<WayPoint> list;
    list << WayPoint(wpt);
    emit wayPointInsert(hash(mWayPoints), 0, list);
}

void ControlWidget::slotWayPointAppend()
{
    QVector3D wpt(mSpinBoxWptX->value(), mSpinBoxWptY->value(), mSpinBoxWptZ->value());
    QList<WayPoint> list;
    list << WayPoint(wpt);
    emit wayPointInsert(hash(mWayPoints), mWayPoints.size(), list);
}

void ControlWidget::slotWayPointDelete()
{
    QList<QTableWidgetItem *> items = mWayPointTable->selectedItems();
    if(items.size() != 1) return;
    emit wayPointDelete(hash(mWayPoints), items.at(0)->row());
}

void ControlWidget::slotRoverReachedNextWayPoint()
{
    mWayPointTable->removeRow(0);
    mGroupBoxWayPoints->setTitle(QString("%1 Waypoints").arg(mWayPointTable->rowCount()));
}

void ControlWidget::slotWayPointChange(int row, int /*column*/)
{
    emit wayPointDelete(hash(mWayPoints), row);

    // update local list to keep the hash correct
    mWayPoints.removeAt(row);

    QVector3D wpt(
                mWayPointTable->item(row, 0)->text().toFloat(),
                mWayPointTable->item(row, 1)->text().toFloat(),
                mWayPointTable->item(row, 2)->text().toFloat());

    QList<WayPoint> list;
    list << WayPoint(wpt);

    emit wayPointInsert(hash(mWayPoints), row, list);
}

void ControlWidget::slotWayPointUp()
{
    QList<QTableWidgetItem *> items = mWayPointTable->selectedItems();

    if(items.size() != 1 || items.at(0)->row() < 1) return;

    const int rowUpper = items.at(0)->row()-1;
    const int rowLower = items.at(0)->row();

    QVector3D wptUpper(
                mWayPointTable->item(rowUpper, 0)->text().toFloat(),
                mWayPointTable->item(rowUpper, 1)->text().toFloat(),
                mWayPointTable->item(rowUpper, 2)->text().toFloat());

    QVector3D wptLower(
                mWayPointTable->item(rowLower, 0)->text().toFloat(),
                mWayPointTable->item(rowLower, 1)->text().toFloat(),
                mWayPointTable->item(rowLower, 2)->text().toFloat());

    // Now switch upper and lower
    emit wayPointDelete(hash(mWayPoints), rowLower);
    mWayPoints.removeAt(rowLower);

    emit wayPointDelete(hash(mWayPoints), rowUpper);
    mWayPoints.removeAt(rowUpper);


    QList<WayPoint> listLower;
    listLower << WayPoint(wptLower);
    emit wayPointInsert(hash(mWayPoints), rowUpper, listLower);
    mWayPoints.insert(rowUpper, wptLower);

    QList<WayPoint> listUpper;
    listUpper << WayPoint(wptUpper);
    emit wayPointInsert(hash(mWayPoints), rowLower, listUpper);
    mWayPoints.insert(rowLower, wptUpper);
}

void ControlWidget::slotWayPointDown()
{
    QList<QTableWidgetItem *> items = mWayPointTable->selectedItems();

    if(items.size() != 1 || items.at(0)->row() >= mWayPointTable->rowCount()-1) return;

    const int rowUpper = items.at(0)->row();
    const int rowLower = items.at(0)->row()+1;

    WayPoint wptUpper(
                QVector3D(
                    mWayPointTable->item(rowUpper, 0)->text().toFloat(),
                    mWayPointTable->item(rowUpper, 1)->text().toFloat(),
                    mWayPointTable->item(rowUpper, 2)->text().toFloat())
                );

    WayPoint wptLower(
                QVector3D(
                    mWayPointTable->item(rowLower, 0)->text().toFloat(),
                    mWayPointTable->item(rowLower, 1)->text().toFloat(),
                    mWayPointTable->item(rowLower, 2)->text().toFloat())
                );

    // Now switch upper and lower
    emit wayPointDelete(hash(mWayPoints), rowLower);
    mWayPoints.removeAt(rowLower);

    emit wayPointDelete(hash(mWayPoints), rowUpper);
    mWayPoints.removeAt(rowUpper);

    QList<WayPoint> listLower;
    listLower << WayPoint(wptLower);
    emit wayPointInsert(hash(mWayPoints), rowUpper, listLower);
    mWayPoints.insert(rowUpper, wptLower);

    QList<WayPoint> listUpper;
    listUpper << WayPoint(wptUpper);
    emit wayPointInsert(hash(mWayPoints), rowLower, listUpper);
    mWayPoints.insert(rowLower, wptUpper);

    mWayPointTable->setCurrentCell(rowLower, 1);
}

void ControlWidget::slotNewWayPoints(const QList<WayPoint>& waypoints)
{
    emit wayPointInsert(hash(mWayPoints), mWayPoints.size(), waypoints);
    mWayPoints.append(waypoints);

    mWayPointTable->blockSignals(true);
    mWayPointTable->setRowCount(mWayPointTable->rowCount() + waypoints.size());

    for(int i=0;i<waypoints.size();i++)
    {
        mWayPointTable->setItem(mWayPointTable->rowCount()-waypoints.size()+i, 0, new QTableWidgetItem(QString::number(waypoints.at(i).x())));
        mWayPointTable->setItem(mWayPointTable->rowCount()-waypoints.size()+i, 1, new QTableWidgetItem(QString::number(waypoints.at(i).y())));
        mWayPointTable->setItem(mWayPointTable->rowCount()-waypoints.size()+i, 2, new QTableWidgetItem(QString::number(waypoints.at(i).z())));
    }
    mWayPointTable->blockSignals(false);

    mGroupBoxWayPoints->setTitle(QString("%1 Waypoints").arg(mWayPointTable->rowCount()));
}

void ControlWidget::slotUpdateBattery(const double& voltageCurrent, const double& voltageMax)
{
    mLabelBatteryVoltageCurrent->setText(QString::number(voltageCurrent, 'f', 2) + " V");
    mLabelBatteryVoltage->setText(QString::number(voltageMax, 'f', 2) + " V");
}

void ControlWidget::slotUpdatePose(const Pose &pose)
{
    mLabelPoseOgreX->setText(QString("%1m").arg(pose.position.x(), 3, 'f', 1, '0'));
    mLabelPoseOgreY->setText(QString("%1m").arg(pose.position.y(), 3, 'f', 1, '0'));
    mLabelPoseOgreZ->setText(QString("%1m").arg(pose.position.z(), 3, 'f', 1, '0'));

//    CoordinateGps wgs84 = mCoordinateConverter->convert(position);

//    mLabelPoseWgs84Longitude->setText(wgs84.formatGpsDegree(wgs84.longitude()));
//    mLabelPoseWgs84Latitude->setText(wgs84.formatGpsDegree(wgs84.latitude()));
//    mLabelPoseWgs84Elevation->setText(QString("%1m").arg(wgs84.elevation(), 3, 'f', 1, QLatin1Char('0')));

    QString deg;
    deg.sprintf("%c", 176);
    deg.prepend("%1");

    mLabelPitch->setText(deg.arg((int)pose.getPitchDegrees(), 3, 10, QLatin1Char('0')));
    mLabelRoll->setText(deg.arg((int)pose.getRollDegrees(), 3, 10, QLatin1Char('0')));
    mLabelYaw->setText(deg.arg((int)pose.getYawDegrees(), 3, 10, QLatin1Char('0')));

    mCompass->setValue(pose.getYawDegrees()+180);
}

void ControlWidget::slotUpdateDynamics(QVector3D linearVelocity)
{
    // flight dynamics
    mLabelSpeed->setText(QString::number(linearVelocity.length(), 'f', 4) + " m/s");
    mLabelSpeedVertical->setText(QString::number(linearVelocity.y(), 'f', 4) + " m/s");
    linearVelocity.setY(0.0);
    mLabelSpeedHorizontal->setText(QString::number(linearVelocity.length(), 'f', 4) + " m/s");
}

void ControlWidget::slotUpdateWayPoints(const QList<WayPoint>& waypoints)
{
    mWayPoints = waypoints;

    mWayPointTable->blockSignals(true);
    mWayPointTable->clear();
    mWayPointTable->setRowCount(waypoints.size());

    for(int i=0;i<waypoints.size();i++)
    {
        QVector3D wpt = waypoints.at(i);
        qDebug() << "now adding waypoint" << wpt << "to table";
        mWayPointTable->setItem(i, 0, new QTableWidgetItem(QString::number(waypoints.at(i).x())));
        mWayPointTable->setItem(i, 1, new QTableWidgetItem(QString::number(waypoints.at(i).y())));
        mWayPointTable->setItem(i, 2, new QTableWidgetItem(QString::number(waypoints.at(i).z())));
    }

    initWayPointTable();
    mWayPointTable->blockSignals(false);

    mGroupBoxWayPoints->setTitle(QString("%1 Waypoints").arg(mWayPointTable->rowCount()));
}

void ControlWidget::slotUpdateSimulationTime(const quint32& time)
{

    const int secsPassed = time / 1000;
    const int secs = secsPassed % 60;
    const int mins = (secsPassed+1) / 60;
    const int hours = (mins+1) / 60;

    mLabelDuration->setText(
            QString("%1:%2:%3").arg(QString::number(hours), 2, '0').arg(QString::number(mins), 2, '0').arg(QString::number(secs), 2, '0')
            );
}

void ControlWidget::slotSetScanVolume()
{
    emit setScanVolume(
                QVector3D(mSpinBoxScanVolumeMinX->value(), mSpinBoxScanVolumeMinY->value(), mSpinBoxScanVolumeMinZ->value()),
                QVector3D(mSpinBoxScanVolumeMaxX->value(), mSpinBoxScanVolumeMaxY->value(), mSpinBoxScanVolumeMaxZ->value())
                );
}

const WayPoint ControlWidget::getNextWayPoint() const
{
    if(mWayPoints.empty())
        return WayPoint();
    else
        return mWayPoints.first();
}
