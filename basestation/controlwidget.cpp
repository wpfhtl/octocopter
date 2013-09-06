#include "controlwidget.h"
#include "flightstaterestriction.h"
#include <QFileDialog>
#include <QMessageBox>

ControlWidget::ControlWidget(QWidget* widget) : QDockWidget(widget)
{
    setupUi(this);

    // this removes the title bar. No more dragging, but saves space
    //setTitleBarWidget(new QWidget());

    initWayPointTable();

    // To enable colored text
    mLabelInsInfo->setTextFormat(Qt::RichText);

    connect(mBtnWptPrepend, &QPushButton::clicked, this, &ControlWidget::slotWayPointPrepend);
    connect(mBtnWptAppend, &QPushButton::clicked, this, &ControlWidget::slotWayPointAppend);
    connect(mBtnWptDelete, &QPushButton::clicked, this, &ControlWidget::slotWayPointDelete);
    connect(mBtnWptClear, &QPushButton::clicked, this, &ControlWidget::slotWayPointClear);
    connect(mBtnWptLoad, &QPushButton::clicked, this, &ControlWidget::slotWayPointLoad);
    connect(mBtnWptSave, &QPushButton::clicked, this, &ControlWidget::slotWayPointSave);

    connect(mWayPointTable, &QTableWidget::cellChanged, this, &ControlWidget::slotWayPointChanged);

    connect(mBtnGenerateWaypoints, SIGNAL(clicked()), SIGNAL(showUserInterface()));
    connect(mWayPointTable, SIGNAL(cellActivated(int,int)), SIGNAL(wayPointSelected(int)));
    connect(mWayPointTable, SIGNAL(cellClicked(int,int)), SIGNAL(wayPointSelected(int)));
    connect(mWayPointTable, SIGNAL(cellEntered(int,int)), SIGNAL(wayPointSelected(int)));

    connect(mBtnEnableScanning, &QPushButton::toggled, [=](const bool checked){
        mBtnEnableScanning->setText(checked ? "Enabled" : "Disabled");
        emit setScannerState(checked);
    });

    connect(mBtnSetVolumeGlobal, &QPushButton::clicked, [=](){
        emit volumeGlobal(
                    Box3D(
                        QVector3D(mSpinBoxScanVolumeMinX->value(), mSpinBoxScanVolumeMinY->value(), mSpinBoxScanVolumeMinZ->value()),
                        QVector3D(mSpinBoxScanVolumeMaxX->value(), mSpinBoxScanVolumeMaxY->value(), mSpinBoxScanVolumeMaxZ->value()))
                    );
    });

    connect(mBtnSetVolumeLocal, &QPushButton::clicked, [=](){
        emit volumeLocal(
                    Box3D(
                        QVector3D(mSpinBoxScanVolumeMinX->value(), mSpinBoxScanVolumeMinY->value(), mSpinBoxScanVolumeMinZ->value()),
                        QVector3D(mSpinBoxScanVolumeMaxX->value(), mSpinBoxScanVolumeMaxY->value(), mSpinBoxScanVolumeMaxZ->value()))
                    );
    });

    QTimer::singleShot(0, this, SLOT(slotResizeToMinimum()));

    setMaximumWidth(minimumWidth());
}

ControlWidget::~ControlWidget()
{
//    delete mStyle;
}

void ControlWidget::slotResizeToMinimum()
{
    // super ugly....
    setMaximumWidth(80000);
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

void ControlWidget::slotUpdateConnectionRover(const bool connected)
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

void ControlWidget::slotUpdateConnectionDiffCorr(const bool working)
{
    if(working)
    {
        mLabelDiffCorrIndicator->setText("OK");

        if(mLabelDiffCorrIndicator->styleSheet() == getBackgroundCss(false, false))
            mLabelDiffCorrIndicator->setStyleSheet(getBackgroundCss(false, true));
        else
            mLabelDiffCorrIndicator->setStyleSheet(getBackgroundCss(false, false));
    }
    else
    {
        mLabelDiffCorrIndicator->setText("ERR");

        if(mLabelDiffCorrIndicator->styleSheet() == getBackgroundCss(true, false))
            mLabelDiffCorrIndicator->setStyleSheet(getBackgroundCss(true, true));
        else
            mLabelDiffCorrIndicator->setStyleSheet(getBackgroundCss(true, false));
    }
}

void ControlWidget::slotSetFlightState(const FlightState* const fs)
{
    qDebug() << "ControlWidget::slotSetFlightState(): flightstate changed to" << fs->toString();
    mLabelFlightState->setText(fs->toString());
    if(fs->state != FlightState::State::Undefined) mLabelFlightState->setStyleSheet(""); else mLabelFlightState->setStyleSheet(getBackgroundCss(true, false));
}

void ControlWidget::slotSetFlightStateRestriction(const FlightStateRestriction* const fsr)
{
    //qDebug() << "ControlWidget::slotSetFlightStateRestriction(): flightstaterestriction changed to" << fsr->toString();
    mLabelFlightStateRestriction->setText(fsr->toString().replace("Restriction", ""));
}

void ControlWidget::slotUpdatePose(const Pose * const pose)
{
    const QVector3D position = pose->getPosition();
    mLabelPoseOgreX->setText(QString("%1").arg(position.x(), 4, 'f', 3, '0'));
    mLabelPoseOgreY->setText(QString("%1").arg(position.y(), 4, 'f', 3, '0'));
    mLabelPoseOgreZ->setText(QString("%1").arg(position.z(), 4, 'f', 3, '0'));

    static QString deg;
    deg.sprintf("%c", 176);
    deg.prepend("%1");

    mLabelPitch->setText(deg.arg(pose->getPitchDegrees(), 3, 'f', 2, '0'));
    mLabelRoll->setText(deg.arg(pose->getRollDegrees(), 3, 'f', 2, '0'));
    mLabelYaw->setText(deg.arg(pose->getYawDegrees(), 3, 'f', 2, '0'));

    mCompass->setValue(pose->getYawDegrees()+180.0f);

    if(!mBtnEnableScanning->isEnabled() && pose->isSufficientlyPreciseForSensorFusion())
    {
        mBtnEnableScanning->setEnabled(true);
    }
}

void ControlWidget::slotUpdateVehicleStatus(const VehicleStatus *const vs)
{
    const int secsPassed = vs->missionRunTime / 1000.0f;
    const int secs = secsPassed % 60;
    const int mins = (secsPassed+1) / 60;
    const int hours = (mins+1) / 60;

    mLabelMissionRunTime->setText(
            QString("%1:%2:%3").arg(QString::number(hours), 2, '0').arg(QString::number(mins), 2, '0').arg(QString::number(secs), 2, '0')
            );

    mLabelBatteryVoltage->setText(QString::number(vs->batteryVoltage, 'f', 2) + " V");
    if(vs->batteryVoltage > 13.5) mLabelBatteryVoltage->setStyleSheet(""); else mLabelBatteryVoltage->setStyleSheet(getBackgroundCss(true, false));

    mLabelBarometricHeight->setText(QString::number(vs->barometricHeight));
}


QString ControlWidget::getBackgroundCss(const bool& error, const bool& dark)
{
    QColor bgColor = error ? QColor(255,80,80) : QColor(80,255,80);

    if(dark) bgColor = bgColor.darker(150);

    return QString("background-color:%1;").arg(bgColor.name());
}

void ControlWidget::slotUpdateInsStatus(const GnssStatus* const gnssStatus)
{
    mLabelGnssMode->setText(gnssStatus->getPvtMode());
    if(gnssStatus->pvtMode == GnssStatus::PvtMode::RtkFixed) mLabelGnssMode->setStyleSheet(""); else mLabelGnssMode->setStyleSheet(getBackgroundCss());

    mLabelInsIntegrationMode->setText(gnssStatus->getIntegrationMode());
    if(gnssStatus->integrationMode == GnssStatus::IntegrationMode::Loosely_INS || gnssStatus->integrationMode == GnssStatus::IntegrationMode::Loosely_INS_and_GNSS)
        mLabelInsIntegrationMode->setStyleSheet("");
    else mLabelInsIntegrationMode->setStyleSheet(getBackgroundCss());

    mLabelInsInfo->setText(gnssStatus->getInfoRichText());
    mLabelInsInfo->setToolTip(mLabelInsInfo->text());

    mLabelInsError->setText(gnssStatus->getError());
    if(gnssStatus->error == GnssStatus::Error::NoError) mLabelInsError->setStyleSheet(""); else mLabelInsError->setStyleSheet(getBackgroundCss());

    mLabelGnssNumSats->setText(QString::number(gnssStatus->numSatellitesUsed));
    if(gnssStatus->numSatellitesUsed > 5) mLabelGnssNumSats->setStyleSheet(""); else mLabelGnssNumSats->setStyleSheet(getBackgroundCss());

    mLabelGnssAge->setText(QString::number(gnssStatus->gnssAge));
    if(gnssStatus->gnssAge < 1) mLabelGnssAge->setStyleSheet(""); else mLabelGnssAge->setStyleSheet(getBackgroundCss());

    mLabelGnssCorrAge->setText(QString::number(((float)gnssStatus->meanCorrAge) / 10.0));
    if(((float)gnssStatus->meanCorrAge)/10.0 < 5) mLabelGnssCorrAge->setStyleSheet(""); else mLabelGnssCorrAge->setStyleSheet(getBackgroundCss());

    mLabelInsCpuLoad->setText(QString::number(gnssStatus->cpuLoad));
    if(gnssStatus->cpuLoad < 80) mLabelInsCpuLoad->setStyleSheet(""); else mLabelInsCpuLoad->setStyleSheet(getBackgroundCss());

    mLabelInsCovariances->setText(QString::number(gnssStatus->covariances, 'f', 4));
    if(gnssStatus->covariances < 1.0) mLabelInsCovariances->setStyleSheet(""); else mLabelInsCovariances->setStyleSheet(getBackgroundCss());
}

void ControlWidget::slotWayPointLoad()
{
    const QString fileName = QFileDialog::getOpenFileName(this, "Load WayPoints");
    if(!mWayPointList.loadFromFile(fileName))
    {
        QMessageBox::critical(this, "File Error", "Couldn't read file!");
    }

    updateWayPointTable();
    slotEmitWaypoints();
}

void ControlWidget::slotWayPointSave()
{
    const QString fileName = QFileDialog::getSaveFileName(this, "Save WayPoints");
    if(!mWayPointList.saveToFile(fileName))
            QMessageBox::critical(this, "File Error", "Couldn't open file for writing!");
}

// Sets the table to the contents of mWayPointList
void ControlWidget::updateWayPointTable()
{
    mWayPointTable->blockSignals(true);

    mWayPointTable->clear();
    mWayPointTable->setRowCount(0);

    for(int i=0;i<mWayPointList.size();i++)
    {
        const WayPoint waypoint = mWayPointList.at(i);

        mWayPointTable->insertRow(i);

        mWayPointTable->setItem(i, 0, new QTableWidgetItem(QString::number(waypoint.x())));
        mWayPointTable->setItem(i, 1, new QTableWidgetItem(QString::number(waypoint.y())));
        mWayPointTable->setItem(i, 2, new QTableWidgetItem(QString::number(waypoint.z())));
    }

    mWayPointTable->blockSignals(false);
    mWayPointTable->resizeRowsToContents();

    mGroupBoxWayPoints->setTitle(QString("%1 Waypoints").arg(mWayPointTable->rowCount()));
}

void ControlWidget::slotEmitWaypoints()
{
    emit wayPoints(mWayPointList.list(), WayPointListSource::WayPointListSourceControlWidget);
}


/*
  ###########################################################################
  Slots that are called when a button is pressed, emits signals with
  modification requests for the waypoint list
  ###########################################################################
*/

void ControlWidget::slotWayPointPrepend()
{
    WayPoint wpt(QVector3D(mSpinBoxWptX->value(), mSpinBoxWptY->value(), mSpinBoxWptZ->value()));
    mWayPointList.prepend(wpt);
    updateWayPointTable();
    slotEmitWaypoints();
}

void ControlWidget::slotWayPointAppend()
{
    WayPoint wpt(QVector3D(mSpinBoxWptX->value(), mSpinBoxWptY->value(), mSpinBoxWptZ->value()));
    mWayPointList.append(wpt);
    updateWayPointTable();
    slotEmitWaypoints();
}

void ControlWidget::slotWayPointDelete()
{
    const QList<QTableWidgetItem *> items = mWayPointTable->selectedItems();
    if(items.size())
    {
        const quint32 index = items.at(0)->row();
        mWayPointList.remove(index);
        updateWayPointTable();
        slotEmitWaypoints();
    }
}

void ControlWidget::slotWayPointClear()
{
    mWayPointList.clear();
    updateWayPointTable();
    slotEmitWaypoints();
}

void ControlWidget::slotWayPointChanged(int row, int /*column*/)
{
    WayPoint wpt = mWayPointList.at(row);
    bool okX, okY, okZ;
    wpt.setX(mWayPointTable->item(row, 0)->text().toFloat(&okX));
    wpt.setY(mWayPointTable->item(row, 0)->text().toFloat(&okY));
    wpt.setZ(mWayPointTable->item(row, 0)->text().toFloat(&okZ));

    if(okX && okY && okZ)
    {
        // If all cells could be parsed, update waypoint list and emit
        mWayPointList.setWayPoint(row, wpt);
        slotEmitWaypoints();
    }
    else
    {
        // if parsing failed, switch back to old values.
        updateWayPointTable();
    }
}

/*
  ###########################################################################
  Slots that are called when a the waypoint list is changed externally, will
  sync the gui with the list.
  ###########################################################################
*/

void ControlWidget::slotSetWayPoints(const QList<WayPoint>* const wayPoints, const WayPointListSource source)
{
    if(source != WayPointListSource::WayPointListSourceControlWidget)
    {
        mWayPointList.setList(wayPoints);
        updateWayPointTable();
    }
}
