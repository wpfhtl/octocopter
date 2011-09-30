#include "controlwidget.h"

ControlWidget::ControlWidget(BaseStation* baseStation) : QDockWidget((QWidget*)baseStation)
{
    setupUi(this);

    // this removes the title bar. No more dragging, but saves space
    setTitleBarWidget(new QWidget());

    // Wire up the time-box
//    connect(mSpinBoxTimeFactor, SIGNAL(valueChanged(double)), SIGNAL(timeFactorChanged(double)));
//    connect(mBtnStart, SIGNAL(clicked()), SLOT(slotSimulationStarted()));
//    connect(mBtnPause, SIGNAL(clicked()), SLOT(slotSimulationPaused()));

    initWayPointTable();
    mTimerRtkIndicator.setInterval(3100);
    mTimerRtkIndicator.start();

    // To enable colored text
    mLabelGpsInfo->setTextFormat(Qt::RichText);

    connect(&mTimerRtkIndicator, SIGNAL(timeout()), SLOT(slotUpdateRtkStatus()));

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

    mBaseStation = baseStation;
//    mCoordinateConverter = mSimulator->mCoordinateConverter;
//    mBattery = mSimulator->mBattery;

//    connect(mBattery, SIGNAL(chargeStatusChanged(int)), SLOT(slotUpdateBattery(int)));

//    mLabelBatteryVoltage->setText(QString::number(mBattery->voltage(), 'g', 2) + " V");
//    mLabelBatteryEnergy->setText(QString::number(mBattery->capacity(), 'g', 2) + " Ah");

    mCompass->setStyle(new QPlastiqueStyle);

    mBarWirelessRssi->setRange(0, 100);

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

void ControlWidget::slotUpdateRoverConnection(bool connected)
{
    if(connected)
    {
        if(mLabelRoverIndicator->styleSheet() == "background-color:#55ff55;")
            mLabelRoverIndicator->setStyleSheet("background-color:#22aa22;");
        else
            mLabelRoverIndicator->setStyleSheet("background-color:#55ff55;");

        mLabelRoverIndicator->setText("OK");
    }
    else
    {
        if(mLabelRoverIndicator->styleSheet() == "background-color:#ff5555;")
            mLabelRoverIndicator->setStyleSheet("background-color:#aa2222;");
        else
            mLabelRoverIndicator->setStyleSheet("background-color:#ff5555;");

        mLabelRoverIndicator->setText("ERR");
    }

}

void ControlWidget::slotUpdateRtkStatus(bool working)
{
    if(working)
    {
        if(mLabelRtkIndicator->styleSheet() == "background-color:#55ff55;")
            mLabelRtkIndicator->setStyleSheet("background-color:#22aa22;");
        else
            mLabelRtkIndicator->setStyleSheet("background-color:#55ff55;");

        mLabelRtkIndicator->setText("OK");

        mTimerRtkIndicator.stop();
        mTimerRtkIndicator.start();
    }
    else
    {
        if(mLabelRtkIndicator->styleSheet() == "background-color:#ff5555;")
            mLabelRtkIndicator->setStyleSheet("background-color:#aa2222;");
        else
            mLabelRtkIndicator->setStyleSheet("background-color:#ff5555;");

        mLabelRtkIndicator->setText("ERR");
    }
}

void ControlWidget::slotUpdateBattery(const float& voltageCurrent)
{
    mLabelBatteryVoltage->setText(QString::number(voltageCurrent, 'f', 2) + " V");
    if(voltageCurrent > 13.5) mLabelBatteryVoltage->setStyleSheet(""); else mLabelBatteryVoltage->setStyleSheet("background-color:#ff5555;");
}

void ControlWidget::slotUpdatePose(const Pose &pose)
{
    mLabelPoseOgreX->setText(QString("%1").arg(pose.position.x(), 4, 'f', 2, '0'));
    mLabelPoseOgreY->setText(QString("%1").arg(pose.position.y(), 4, 'f', 2, '0'));
    mLabelPoseOgreZ->setText(QString("%1").arg(pose.position.z(), 4, 'f', 2, '0'));

    QString deg;
    deg.sprintf("%c", 176);
    deg.prepend("%1");

    mLabelPitch->setText(deg.arg((int)pose.getPitchDegrees(), 3, 10, QLatin1Char('0')));
    mLabelRoll->setText(deg.arg((int)pose.getRollDegrees(), 3, 10, QLatin1Char('0')));
    mLabelYaw->setText(deg.arg((int)pose.getYawDegrees(), 3, 10, QLatin1Char('0')));

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

void ControlWidget::slotUpdateGpsStatus(const quint8& gnssMode,
                                        const quint8& integrationMode,
                                        const quint16& info,
                                        const quint8& error,
                                        const quint8& numSatellitesTracked,
                                        const quint8& lastPvtAge,
                                        const quint8& meanCorrAge,
                                        const QString& status)
{
    mLabelGpsGnssMode->setText(GpsStatusInformation::getGnssMode(gnssMode));
    if(gnssMode == 4) mLabelGpsGnssMode->setStyleSheet(""); else mLabelGpsGnssMode->setStyleSheet("background-color:#ff5555;");

    mLabelGpsIntegrationMode->setText(GpsStatusInformation::getIntegrationMode(integrationMode));
    if(integrationMode == 2) mLabelGpsIntegrationMode->setStyleSheet(""); else mLabelGpsIntegrationMode->setStyleSheet("background-color:#ff5555;");

    mLabelGpsInfo->setText(GpsStatusInformation::getInfoRichText(info));

    mLabelGpsError->setText(GpsStatusInformation::getError(error));
    if(error == 0) mLabelGpsError->setStyleSheet(""); else mLabelGpsError->setStyleSheet("background-color:#ff5555;");

    mLabelGpsNumSats->setText(QString::number(numSatellitesTracked));
    if(numSatellitesTracked > 5) mLabelGpsNumSats->setStyleSheet(""); else mLabelGpsNumSats->setStyleSheet("background-color:#ff5555;");

    mLabelGpsPvtAge->setText(QString::number(lastPvtAge));
    if(lastPvtAge < 1) mLabelGpsPvtAge->setStyleSheet(""); else mLabelGpsPvtAge->setStyleSheet("background-color:#ff5555;");

    mLabelGpsCorrAge->setText(QString::number(((float)meanCorrAge) / 10.0));
    if(((float)meanCorrAge)/10.0 < 5) mLabelGpsCorrAge->setStyleSheet(""); else mLabelGpsCorrAge->setStyleSheet("background-color:#ff5555;");

    if(!status.isEmpty())
    {
        mTextEditGpsStatus->appendPlainText(status);
        mTextEditGpsStatus->moveCursor(QTextCursor::End);
        mTextEditGpsStatus->moveCursor(QTextCursor::StartOfLine);
    }
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
    QList<QTableWidgetItem *> items = mWayPointTable->selectedItems();
    //if(items.size() != 1) return;
    emit wayPointDelete(items.at(0)->row());
}

void ControlWidget::slotWayPointChange(int row, int /*column*/)
{
    QVector3D wpt(
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
    mWayPointTable->removeRow(index);
    mGroupBoxWayPoints->setTitle(QString("%1 Waypoints").arg(mWayPointTable->rowCount()));
}

void ControlWidget::slotSetWayPoints(QList<WayPoint> wayPoints)
{
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
