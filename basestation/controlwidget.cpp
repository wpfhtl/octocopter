#include "controlwidget.h"

ControlWidget::ControlWidget(QWidget* widget) : QDockWidget(widget)
{
    setupUi(this);

    // this removes the title bar. No more dragging, but saves space
    //setTitleBarWidget(new QWidget());

    initWayPointTable();

    // To enable colored text
    mLabelGnssInfo->setTextFormat(Qt::RichText);

    connect(mBtnWptPrepend, SIGNAL(clicked()), SLOT(slotWayPointPrepend()));
    connect(mBtnWptAppend, SIGNAL(clicked()), SLOT(slotWayPointAppend()));
    connect(mBtnWptDelete, SIGNAL(clicked()), SLOT(slotWayPointDelete()));
    connect(mBtnWptLoad, SIGNAL(clicked()), SLOT(slotWayPointLoad()));
    connect(mBtnWptSave, SIGNAL(clicked()), SLOT(slotWayPointSave()));

    connect(mBtnWptUp, SIGNAL(clicked()), SLOT(slotWayPointUp()));
    connect(mBtnWptDown, SIGNAL(clicked()), SLOT(slotWayPointDown()));

    connect(mWayPointTable, SIGNAL(cellChanged(int,int)), SLOT(slotWayPointChange(int,int)));

    connect(mBtnGenerateWaypoints, SIGNAL(clicked()), SIGNAL(generateWaypoints()));

    connect(mBtnSetScanVolume, SIGNAL(clicked()), SLOT(slotSetScanVolume()));

    mCompass->setStyle(new QPlastiqueStyle);

    mBarWirelessRssi->setRange(0, 100);

    resize(minimumSizeHint());
}

ControlWidget::~ControlWidget()
{
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

void ControlWidget::slotUpdateConnectionRover(bool connected)
{
    if(connected)
    {
        if(mLabelRoverIndicator->styleSheet() == getBackgroundCss(false, false))
            mLabelRoverIndicator->setStyleSheet(getBackgroundCss(false, true));
        else
            mLabelRoverIndicator->setStyleSheet(getBackgroundCss(false, false));

        mLabelRoverIndicator->setText("OK");
    }
    else
    {
        if(mLabelRoverIndicator->styleSheet() == getBackgroundCss(true, false))
            mLabelRoverIndicator->setStyleSheet(getBackgroundCss(true, true));
        else
            mLabelRoverIndicator->setStyleSheet(getBackgroundCss(true, false));

        mLabelRoverIndicator->setText("ERR");
    }
}

void ControlWidget::slotUpdateConnectionRtk(bool working)
{
    if(working)
    {
        mLabelRtkIndicator->setText("OK");

        if(mLabelRtkIndicator->styleSheet() == getBackgroundCss(false, false))
            mLabelRtkIndicator->setStyleSheet(getBackgroundCss(false, true));
        else
            mLabelRtkIndicator->setStyleSheet(getBackgroundCss(false, false));
    }
    else
    {
        mLabelRtkIndicator->setText("ERR");

        if(mLabelRtkIndicator->styleSheet() == getBackgroundCss(true, false))
            mLabelRtkIndicator->setStyleSheet(getBackgroundCss(true, true));
        else
            mLabelRtkIndicator->setStyleSheet(getBackgroundCss(true, false));
    }
}

void ControlWidget::slotUpdateBattery(const float& voltageCurrent)
{
    mLabelBatteryVoltage->setText(QString::number(voltageCurrent, 'f', 2) + " V");
    if(voltageCurrent > 13.5) mLabelBatteryVoltage->setStyleSheet(""); else mLabelBatteryVoltage->setStyleSheet(getBackgroundCss(true, false));
}

void ControlWidget::slotFlightStateChanged(FlightState fs)
{
    mLabelFlightState->setText(getFlightStateString(fs));
    if(fs != Hover) mLabelFlightState->setStyleSheet(""); else mLabelFlightState->setStyleSheet(getBackgroundCss(true, false));
}

void ControlWidget::slotUpdatePose(const Pose &pose)
{
    const QVector3D position = pose.getPosition();
    mLabelPoseOgreX->setText(QString("%1").arg(position.x(), 4, 'f', 3, '0'));
    mLabelPoseOgreY->setText(QString("%1").arg(position.y(), 4, 'f', 3, '0'));
    mLabelPoseOgreZ->setText(QString("%1").arg(position.z(), 4, 'f', 3, '0'));

    QString deg;
    deg.sprintf("%c", 176);
    deg.prepend("%1");

    mLabelPitch->setText(deg.arg(pose.getPitchDegrees(), 3, 'f', 2, '0'));
    mLabelRoll->setText(deg.arg(pose.getRollDegrees(), 3, 'f', 2, '0'));
    mLabelYaw->setText(deg.arg(pose.getYawDegrees(), 3, 'f', 2, '0'));

    mCompass->setValue(pose.getYawDegrees()+180);
}

void ControlWidget::slotUpdateMissionRunTime(const quint32& time)
{
    //qDebug() << "time passed:"<< time;
    const int secsPassed = time / 1000;
    const int secs = secsPassed % 60;
    const int mins = (secsPassed+1) / 60;
    const int hours = (mins+1) / 60;

    mLabelMissionRunTime->setText(
            QString("%1:%2:%3").arg(QString::number(hours), 2, '0').arg(QString::number(mins), 2, '0').arg(QString::number(secs), 2, '0')
            );
}

void ControlWidget::slotUpdateWirelessRssi(const qint8& wirelessRssi)
{
    if(wirelessRssi < 0 && mBarWirelessRssi->maximum() == 100)
    {
        mBarWirelessRssi->setRange(0, 0);
    }
    else
    {
        mBarWirelessRssi->setRange(0, 100);
        mBarWirelessRssi->setValue(wirelessRssi);
    }
}

QString ControlWidget::getBackgroundCss(const bool& error, const bool& dark)
{
    QColor bgColor = error ? QColor(255,80,80) : QColor(80,255,80);

    if(dark) bgColor = bgColor.darker(150);

    return QString("background-color:%1;").arg(bgColor.name());
}

void ControlWidget::slotUpdateGnssStatus(const GnssStatusInformation::GnssStatus& gnssStatus)
{
    mLabelGnssMode->setText(GnssStatusInformation::getGnssMode(gnssStatus.gnssMode));
    if(gnssStatus.gnssMode == 4) mLabelGnssMode->setStyleSheet(""); else mLabelGnssMode->setStyleSheet(getBackgroundCss());

    mLabelGnssIntegrationMode->setText(GnssStatusInformation::getIntegrationMode(gnssStatus.integrationMode));
    if(gnssStatus.integrationMode == 2) mLabelGnssIntegrationMode->setStyleSheet(""); else mLabelGnssIntegrationMode->setStyleSheet(getBackgroundCss());

    mLabelGnssInfo->setText(GnssStatusInformation::getInfoRichText(gnssStatus.info));
    mLabelGnssInfo->setToolTip(mLabelGnssInfo->text());

    mLabelGnssError->setText(GnssStatusInformation::getError(gnssStatus.error));
    if(gnssStatus.error == 0) mLabelGnssError->setStyleSheet(""); else mLabelGnssError->setStyleSheet(getBackgroundCss());

    mLabelGnssNumSats->setText(QString::number(gnssStatus.numSatellitesUsed));
    if(gnssStatus.numSatellitesUsed > 5) mLabelGnssNumSats->setStyleSheet(""); else mLabelGnssNumSats->setStyleSheet(getBackgroundCss());

    mLabelGnssAge->setText(QString::number(gnssStatus.gnssAge));
    if(gnssStatus.gnssAge < 1) mLabelGnssAge->setStyleSheet(""); else mLabelGnssAge->setStyleSheet(getBackgroundCss());

    mLabelGnssCorrAge->setText(QString::number(((float)gnssStatus.meanCorrAge) / 10.0));
    if(((float)gnssStatus.meanCorrAge)/10.0 < 5) mLabelGnssCorrAge->setStyleSheet(""); else mLabelGnssCorrAge->setStyleSheet(getBackgroundCss());

    mLabelGnssCpuLoad->setText(QString::number(gnssStatus.cpuLoad));
    if(gnssStatus.cpuLoad < 80) mLabelGnssCpuLoad->setStyleSheet(""); else mLabelGnssCpuLoad->setStyleSheet(getBackgroundCss());

    mLabelGnssCovariances->setText(QString::number(gnssStatus.covariances, 'f', 2));
    if(gnssStatus.covariances < 1.0) mLabelGnssCovariances->setStyleSheet(""); else mLabelGnssCovariances->setStyleSheet(getBackgroundCss());
}

void ControlWidget::slotUpdateBarometricHeight(const qint16& barometricHeight)
{
    mLabelBarometricHeight->setText(QString::number(barometricHeight));
}

void ControlWidget::slotSetScanVolume()
{
    emit setScanVolume(
                QVector3D(mSpinBoxScanVolumeMinX->value(), mSpinBoxScanVolumeMinY->value(), mSpinBoxScanVolumeMinZ->value()),
                QVector3D(mSpinBoxScanVolumeMaxX->value(), mSpinBoxScanVolumeMaxY->value(), mSpinBoxScanVolumeMaxZ->value())
                );
}

/*
  ###########################################################################
  Slots that are called when a button is pressed, emits signals with
  modification requests for the waypoint list
  ###########################################################################
*/

void ControlWidget::slotWayPointPrepend()
{
    emit wayPointInsert(0, WayPoint(QVector3D(mSpinBoxWptX->value(), mSpinBoxWptY->value(), mSpinBoxWptZ->value())));
}

void ControlWidget::slotWayPointAppend()
{
    emit wayPointInsert(mWayPointTable->rowCount(), WayPoint(QVector3D(mSpinBoxWptX->value(), mSpinBoxWptY->value(), mSpinBoxWptZ->value())));
}

void ControlWidget::slotWayPointDelete()
{
    const QList<QTableWidgetItem *> items = mWayPointTable->selectedItems();
    if(items.size()) emit wayPointDelete(items.at(0)->row());
}

void ControlWidget::slotWayPointChange(int row, int /*column*/)
{
    const QVector3D wpt(
                mWayPointTable->item(row, 0)->text().toFloat(),
                mWayPointTable->item(row, 1)->text().toFloat(),
                mWayPointTable->item(row, 2)->text().toFloat());

    emit wayPointDelete(row);
    emit wayPointInsert(row, wpt);
}

void ControlWidget::slotWayPointUp()
{
    QList<QTableWidgetItem *> items = mWayPointTable->selectedItems();

    if(/*items.size() != 1 || */items.at(0)->row() < 1) return;

    const int rowUpper = items.at(0)->row()-1;
    const int rowLower = items.at(0)->row();

    emit wayPointSwap(rowUpper, rowLower);
    mWayPointTable->setCurrentCell(rowUpper, 1);
}

void ControlWidget::slotWayPointDown()
{
    QList<QTableWidgetItem *> items = mWayPointTable->selectedItems();

    if(/*items.size() != 1 || */items.at(0)->row() >= mWayPointTable->rowCount()-1)
    {
        qDebug() << "items size" << items.size() << "row" << items.at(0)->row() << "rowcount" << mWayPointTable->rowCount();
        return;
    }

    const int rowUpper = items.at(0)->row();
    const int rowLower = items.at(0)->row()+1;

    emit wayPointSwap(rowUpper, rowLower);
    mWayPointTable->setCurrentCell(rowLower, 1);
}


/*
  ###########################################################################
  Slots that are called when a the waypoint list is changed externally, will
  sync the gui with the list.
  ###########################################################################
*/

void ControlWidget::slotWayPointInserted(const quint16& index, const WayPoint& waypoint)
{
    qDebug() << "ControlWidget::slotWayPointInserted(): waypoint" << waypoint << "inserted into index" << index << "by rover or flightplanner";
    mWayPointTable->blockSignals(true);
    mWayPointTable->insertRow(index);

    mWayPointTable->setItem(index, 0, new QTableWidgetItem(QString::number(waypoint.x())));
    mWayPointTable->setItem(index, 1, new QTableWidgetItem(QString::number(waypoint.y())));
    mWayPointTable->setItem(index, 2, new QTableWidgetItem(QString::number(waypoint.z())));

    mWayPointTable->resizeRowsToContents();

    mWayPointTable->blockSignals(false);

    mGroupBoxWayPoints->setTitle(QString("%1 Waypoints").arg(mWayPointTable->rowCount()));
}

void ControlWidget::slotWayPointDeleted(const quint16& index)
{
    qDebug() << "ControlWidget::slotWayPointInserted(): waypoint deleted at index" << index << "by rover or flightplanner";
    mWayPointTable->removeRow(index);
    mGroupBoxWayPoints->setTitle(QString("%1 Waypoints").arg(mWayPointTable->rowCount()));
}

void ControlWidget::slotSetWayPoints(QList<WayPoint> wayPoints)
{
    qDebug() << "ControlWidget::slotSetWayPoints():" << wayPoints.size() << "waypoints set by rover or flightplanner";
    mWayPointTable->clear();
    mWayPointTable->setRowCount(0);

    mWayPointTable->blockSignals(true);

    for(int i=0;i<wayPoints.size();i++)
    {
        const WayPoint waypoint = wayPoints.at(i);

        mWayPointTable->insertRow(i);

        mWayPointTable->setItem(i, 0, new QTableWidgetItem(QString::number(waypoint.x())));
        mWayPointTable->setItem(i, 1, new QTableWidgetItem(QString::number(waypoint.y())));
        mWayPointTable->setItem(i, 2, new QTableWidgetItem(QString::number(waypoint.z())));
    }

    mWayPointTable->blockSignals(false);
    mWayPointTable->resizeRowsToContents();

    mGroupBoxWayPoints->setTitle(QString("%1 Waypoints").arg(mWayPointTable->rowCount()));
}

void ControlWidget::slotWayPointsCleared()
{
    mWayPointTable->clear();
    mWayPointTable->setRowCount(0);
}

void ControlWidget::slotWayPointLoad()
{
    const QString fileName = QFileDialog::getOpenFileName(this, "Load WayPoints");
    if(!fileName.isNull())
    {
        QFile file(fileName);
        if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QMessageBox::critical(this, "File Error", "Couldn't open file for reading!");
            return;
        }

        int i = 0;
        while (!file.atEnd())
        {
            const QString line(file.readLine());
            const QStringList values = line.split(";", QString::SkipEmptyParts);

            if(values.size() != 3)
            {
                QMessageBox::warning(this, "File Format Error", QString("Not three numbers in line %1!").arg(i));
                return;
            }

            const WayPoint wpt(
                        QVector3D(
                            values.at(0).toFloat(),
                            values.at(1).toFloat(),
                            values.at(2).toFloat()
                            )
                        );

            emit wayPointInsert(mWayPointTable->rowCount(), wpt);

            i++;
        }

        file.close();
    }
}

void ControlWidget::slotWayPointSave()
{
    const QString fileName = QFileDialog::getSaveFileName(this, "Save WayPoints");
    if(!fileName.isNull())
    {
        QFile file(fileName);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            QMessageBox::critical(this, "File Error", "Couldn't open file for writing!");
            return;
        }

        QTextStream out(&file);

        for(int row=0;row<mWayPointTable->rowCount();row++)
        {
            out << mWayPointTable->item(row, 0)->text().toFloat() << ";";
            out << mWayPointTable->item(row, 1)->text().toFloat() << ";";
            out << mWayPointTable->item(row, 2)->text().toFloat();
            out << "\n";
        }

        file.close();
    }
}

void ControlWidget::slotSetWayPointCoordinateFields(Qt::MouseButton btn, QVector3D pos)
{
    if(btn == Qt::MiddleButton)
    {
        mSpinBoxWptX->setValue(pos.x());
        mSpinBoxWptY->setValue(pos.y());
        mSpinBoxWptZ->setValue(pos.z());
    }
}
