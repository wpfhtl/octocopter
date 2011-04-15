#include "controlwidget.h"

ControlWidget::ControlWidget(Triangulator* triangulator) : QDockWidget((QWidget*)triangulator)
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

    mTriangulator = triangulator;
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
    emit wayPointInsert(hash(mWayPoints), 0, wpt);
}

void ControlWidget::slotWayPointAppend()
{
    QVector3D wpt(mSpinBoxWptX->value(), mSpinBoxWptY->value(), mSpinBoxWptZ->value());
    emit wayPointInsert(hash(mWayPoints), mWayPoints.size(), wpt);
}

void ControlWidget::slotWayPointDelete()
{
    QList<QTableWidgetItem *> items = mWayPointTable->selectedItems();
    if(items.size() != 1) return;
    emit wayPointDelete(hash(mWayPoints), items.at(0)->row());
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

    emit wayPointInsert(hash(mWayPoints), row, wpt);
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

    emit wayPointInsert(hash(mWayPoints), rowUpper, wptLower);
    mWayPoints.insert(rowUpper, wptLower);

    emit wayPointInsert(hash(mWayPoints), rowLower, wptUpper);
    mWayPoints.insert(rowLower, wptUpper);
}

void ControlWidget::slotWayPointDown()
{
    QList<QTableWidgetItem *> items = mWayPointTable->selectedItems();

    if(items.size() != 1 || items.at(0)->row() >= mWayPointTable->rowCount()-1) return;

    const int rowUpper = items.at(0)->row();
    const int rowLower = items.at(0)->row()+1;

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

    emit wayPointInsert(hash(mWayPoints), rowUpper, wptLower);
    mWayPoints.insert(rowUpper, wptLower);

    emit wayPointInsert(hash(mWayPoints), rowLower, wptUpper);
    mWayPoints.insert(rowLower, wptUpper);

    mWayPointTable->setCurrentCell(rowLower, 1);
}

void ControlWidget::slotNewWayPoint(const QVector3D point)
{
    emit wayPointInsert(hash(mWayPoints), mWayPoints.size(), point);
    mWayPoints.append(point);

    mWayPointTable->blockSignals(true);
    mWayPointTable->setRowCount(mWayPointTable->rowCount()+1);
    mWayPointTable->setItem(mWayPointTable->rowCount()-1, 0, new QTableWidgetItem(QString::number(point.x())));
    mWayPointTable->setItem(mWayPointTable->rowCount()-1, 1, new QTableWidgetItem(QString::number(point.y())));
    mWayPointTable->setItem(mWayPointTable->rowCount()-1, 2, new QTableWidgetItem(QString::number(point.z())));
    mWayPointTable->blockSignals(false);

    mGroupBoxWayPoints->setTitle(QString("%1 Waypoints").arg(mWayPointTable->rowCount()));
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

void ControlWidget::slotSetScanVolume()
{
    emit setScanVolume(
                QVector3D(mSpinBoxScanVolumeMinX->value(), mSpinBoxScanVolumeMinY->value(), mSpinBoxScanVolumeMinZ->value()),
                QVector3D(mSpinBoxScanVolumeMaxX->value(), mSpinBoxScanVolumeMaxY->value(), mSpinBoxScanVolumeMaxZ->value())
                );
}

const QVector3D ControlWidget::getNextWayPoint() const
{
    if(mWayPoints.empty())
        return QVector3D();
    else
        return mWayPoints.first();
}
