#include "logplayer.h"
#include "ui_logplayer.h"
#include <QMenu>
#include <QMatrix4x4>
#include <QDataStream>

LogPlayer::LogPlayer(QWidget *parent) : QDockWidget(parent), ui(new Ui::LogPlayer)
{
    ui->setupUi(this);
    mProgressBarTow = new ProgressBar;
    ui->verticalLayout->insertWidget(2, mProgressBarTow);

    //setMaximumSize(minimumSize());

    mTimerAnimation = new QTimer(this);
    connect(mTimerAnimation, SIGNAL(timeout()), SLOT(slotPlay()));

    mSensorFuser = new SensorFuser(1);
    //mSensorFuser->setMaximumFusableRayLength(20.0f);
    mSbfParser = new SbfParser(this);

    // Allow user to choose to step until a specific datasource was processed
    mStepUntilLogType = LogTypeInvalid;
    mStepSignalMapper = new QSignalMapper(this);
    mStepMenu = new QMenu(this);

    mStepMenu->addAction("Any", mStepSignalMapper, SLOT(map()));
    connect(mStepMenu->actions().last(), SIGNAL(triggered()), mStepSignalMapper, SLOT(map()));
    mStepSignalMapper->setMapping(mStepMenu->actions().last(), (int)LogTypeInvalid);

    mStepMenu->addAction("GNSS", mStepSignalMapper, SLOT(map()));
    connect(mStepMenu->actions().last(), SIGNAL(triggered()), mStepSignalMapper, SLOT(map()));
    mStepSignalMapper->setMapping(mStepMenu->actions().last(), (int)LogTypeGnss);

    mStepMenu->addAction("LIDAR");
    connect(mStepMenu->actions().last(), SIGNAL(triggered()), mStepSignalMapper, SLOT(map()));
    mStepSignalMapper->setMapping(mStepMenu->actions().last(), (int)LogTypeLaser);

    mStepMenu->addAction("FlightController");
    connect(mStepMenu->actions().last(), SIGNAL(triggered()), mStepSignalMapper, SLOT(map()));
    mStepSignalMapper->setMapping(mStepMenu->actions().last(), (int)LogTypeFlightController);

    ui->mPushButtonStepForward->setMenu(mStepMenu);
    connect(mStepSignalMapper, SIGNAL(mapped(int)), this, SLOT(slotStepDataSourceChanged(int)));

    // Connect UI...
    connect(ui->mPushButtonOpenLogs, SIGNAL(clicked()), SLOT(slotOpenLogFiles()));
    connect(ui->mPushButtonRewind, SIGNAL(clicked()), SLOT(slotRewind()));
    connect(ui->mPushButtonGoTo, SIGNAL(clicked()), SLOT(slotGoToTow()));
    connect(ui->mPushButtonPlay, SIGNAL(clicked()), SLOT(slotPlay()));
    connect(ui->mPushButtonStepForward, SIGNAL(clicked()), SLOT(slotStepUntilDataSourceProcessed()));

    connect(mProgressBarTow, SIGNAL(seekToTow(qint32)), SLOT(slotGoToTow(qint32)));

    // emit fused lidarpoints
    connect(mSensorFuser, SIGNAL(scanData(float*const,quint32,QVector3D*const)), SIGNAL(scanData(float*const,quint32,QVector3D*const)));

    // For visualizing interpolated poses - disabled by default
    connect(mSensorFuser, SIGNAL(vehiclePose(Pose*const)), SIGNAL(vehiclePose(Pose*const)));

    connect(mSbfParser, SIGNAL(status(const GnssStatus* const)), SIGNAL(gnssStatus(const GnssStatus* const)));
    connect(mSbfParser, SIGNAL(message(LogImportance,QString,QString)), SIGNAL(message(LogImportance,QString,QString)));
    connect(mSbfParser, SIGNAL(newVehiclePose(const Pose* const)), SIGNAL(vehiclePose(const Pose* const)));
    connect(mSbfParser, SIGNAL(newVehiclePoseSensorFuser(const Pose* const)), mSensorFuser, SLOT(slotNewVehiclePose(const Pose* const)));
    connect(mSbfParser, SIGNAL(processedPacket(qint32,const char*,quint16)), SLOT(slotNewSbfTime(qint32,const char*,quint16)));
    connect(mSbfParser, SIGNAL(scanFinished(const quint32&)), mSensorFuser, SLOT(slotScanFinished(const quint32&)));

    // Actually invoke slotLaserScannerRelativePoseChanged() AFTER our parent
    // has connected our signals, so the values are propagated to sensorfuser.
    QTimer::singleShot(1000, this, SLOT(slotLaserScannerRelativePoseChanged()));
}

LogPlayer::~LogPlayer()
{
    delete mSensorFuser;
    delete ui;
    delete mStepMenu;
    delete mStepSignalMapper;
}

bool LogPlayer::slotOpenLogFiles()
{
    // We're opening new logfiles. That means we should clear the old ones!
    // GNSS and FlightController are not dynamic on the heap, so we don't care
    while(mLogsLaser.size()) delete mLogsLaser.takeLast();

    // SensorFuser keeps pointers to the relative sensor poses that we store.
    // When we delete those poses, tell sensorfuser to also delete the pointers.
    mSensorFuser->slotClearData();
    mRelativeLaserPoses.clear();

    QString logFileName;
    QString fileNameOfNextLogFile;

    logFileName = QFileDialog::getOpenFileName(this, "Select SBF log", QString(), "SBF Data (*.sbf)");

    QFile logFileSbf(logFileName);
    if(!logFileSbf.open(QIODevice::ReadOnly))
    {
        emit message(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Unable to open SBF log file");
    }
    else
    {
        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), QString("Reading SBF log file %1...").arg(logFileName));
        mLogGnss.data = logFileSbf.readAll();
    }

    // We try to open the flightcontroller log. But even if this fails, do not abort, as SBF only is still something we can work with for playing back
    fileNameOfNextLogFile = fileNameOfNextLogFile.replace("gnssdata.sbf", "flightcontroller.flt");
    logFileName = QFileDialog::getOpenFileName(this, "Select flightcontroller log", fileNameOfNextLogFile, "Flightcontroller Log Data (*.flt)");
    QFile logFileFlightController(logFileName);
    if(!logFileFlightController.open(QIODevice::ReadOnly))
    {
        emit message(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), QString("Unable to open FlightController log file %1").arg(logFileName));
    }
    else
    {
        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), QString("Reading FlightController log file %1...").arg(logFileName));
        mLogFlightController.data = logFileFlightController.readAll();
    }

    // We try to open the laser logs. But even if this fails, do not abort, as SBF only is still something we can work with for playing back
    fileNameOfNextLogFile = logFileName.replace("flightcontroller.flt", "scannerdata.lsr");
    quint32 laserScannerNumber = 0;
    do
    {
        const QString fileNameOfNextLaserLog = fileNameOfNextLogFile + QString::number(laserScannerNumber);
        logFileName = QFileDialog::getOpenFileName(this, "Select laser log", fileNameOfNextLaserLog, QString("Laser Data (*.lsr%1)").arg(laserScannerNumber));
        QFile logFileLaser(logFileName);
        if(!logFileLaser.open(QIODevice::ReadOnly))
        {
            emit message(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), QString("Unable to open Laser log file %1").arg(logFileName));
        }
        else
        {
            emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), QString("Reading Laser log file %1...").arg(logFileName));
            mLogsLaser.append(new LogData(logFileLaser.readAll()));

            // Append a default pose. Maybe overwritten later, using a pose found in a logfile.
            mRelativeLaserPoses.append(
                        Pose(
                            QVector3D(      // Offset from vehicle center to Laser Source. In Vehicle Reference Frame: Like OpenGL, red arm forward pointing to screen
                                            +0.00,      // From vehicle left/right to laser, positive is moved to right "wing"
                                            -0.04,      // From vehicle up/down to laser, negative is down to laser
                                            -0.14),     // From vehicle 14cm forward, towards the front arm.
                            +000.0,         // No yawing
                            -090.0,         // 90 deg pitched down
                            +000.0,         // No rolling
                            10             // Use 10 msec TOW, so that the relative pose is always older than whatever new pose coming in. Don't use 0, as that would be set to current TOW, which might be newer due to clock offsets.
                            )
                        );
        }
        laserScannerNumber++;
    } while(!logFileName.isNull());

    slotRewind();

    return true;
}

void LogPlayer::slotRewind()
{
    mTimePlaybackStartReal = QTime(); // set invalid

    mLogGnss.cursor = 0;
    mLogFlightController.cursor = 0;
    for(int i=0;i<mLogsLaser.size();i++) mLogsLaser.at(i)->cursor = 0;

    // Whats the minimum TOW in our data? Set progressbar accordingly
    qint32 towStart;
    getNextDataSource(&towStart);

    // Whats the maximum TOW in our data? Set progressbar accordingly
    const qint32 towStop = std::max(getLastTow(LogTypeGnss), std::max(getLastTow(LogTypeLaser), getLastTow(LogTypeFlightController)));

    qDebug() << "LogPlayer::slotRewind(): this file contains data between" << towStart << "and" << towStop << "- length in seconds:" << (towStop - towStart)/1000;
    mProgressBarTow->setRange(towStart, towStop);
    mProgressBarTow->setValue(towStart);
}

qint32 LogPlayer::getLastTow(const DataSource& source)
{
    qint32 tow = -1;
    switch(source.type)
    {

    case LogTypeGnss:
    {
        qint32 lastSearchIndex = -1;

        // Search backwards as long as we cannot find a valid SBF packe to extract TOW from
        while(lastSearchIndex > -2 && tow < 0)
        {
            QByteArray lastPacketSbf = mLogGnss.data.right(mLogGnss.data.size() - mLogGnss.data.lastIndexOf("$@", lastSearchIndex));
            mSbfParser->getNextValidPacketInfo(lastPacketSbf, 0, 0, &tow);

            // Where to search next if TOW couldn't be extracted
            lastSearchIndex = mLogGnss.data.size() - lastPacketSbf.size() - 2;
        }
    }
    break;

    case LogTypeLaser:
    {
        // Technically, the laser log also contains RELATIVESCANNERPOSE packets. But these have no TOW
        // and are really unlikely to appear at the end anyway.
        const QByteArray magic("LASER");
        if(source.index != -1)
        {
            // A laserscanner-index is defined, so getLastTow() of THAT scanner.
            Q_ASSERT(source.index > 0 && source.index < mLogsLaser.size());
            const int lastPos = mLogsLaser.at(source.index)->data.lastIndexOf(magic);

            if(lastPos != -1)
                tow = *((qint32*)(mLogsLaser.at(source.index)->data.data() + lastPos + 5 + sizeof(quint16)));
        }
        else
        {
            // No laserscanner index defined, so getLastTow() of all scanners.
            for(int i=0;i<mLogsLaser.size();i++)
            {
                const int lastPos = mLogsLaser.at(i)->data.lastIndexOf(magic);
        //        qDebug() << "LogPlayer::getLastTow(): last laser packet startpos is" << lastPos;

                if(lastPos == -1)
                    continue;
                else
                    tow = std::max(tow, *((qint32*)(mLogsLaser.at(i)->data.data() + lastPos + 5 + sizeof(quint16))));
            }
        }
    }
    break;

    case LogTypeFlightController:
    {
        qint32 lastSearchIndex = -1;

        // Search backwards as long as we cannot find a valid FlightController packet to extract TOW from
        while(lastSearchIndex > -2 && (tow < 0 || tow > 7 * 24 * 3600 * 1000))
        {
            QByteArray lastPacket = mLogFlightController.data.right(mLogFlightController.data.size() - mLogFlightController.data.lastIndexOf("FLTCLR", lastSearchIndex));

            if(lastPacket.isEmpty()) return -1;

            QDataStream ds(lastPacket);
            ds.skipRawData(6); // FLTCLR
            FlightControllerValues fcv;
            ds >> fcv;

            tow = fcv.timestamp;

            // Where to search next if values couldn't be extracted
            lastSearchIndex = mLogFlightController.data.size() - lastPacket.size() - 6;
        }
    }
    break;

    default:
        Q_ASSERT("unknown datasource!");
        break;

    }

    return tow;
}

qint32 LogPlayer::getNextTow(const DataSource& source)
{
    qint32 tow = -1;

    switch(source.type)
    {

    case LogTypeGnss:
    {
        if(!mSbfParser->getNextValidPacketInfo(mLogGnss.data, mLogGnss.cursor, 0, &tow))
            tow = -1;
    }
    break;

    case LogTypeLaser:
    {
        const Data nextPacket = getNextPacket(source);

        if(nextPacket.size)
        {
            tow = *((qint32*)(nextPacket.data + 5 + sizeof(quint16)));
        }
    }
    break;

    case LogTypeFlightController:
    {
        const Data nextPacket = getNextPacket(source);

        // If we can extract something, fine. If not, we might just not have any laser data at all. In that case we'll return -1
        if(nextPacket.size)
        {
            const QByteArray data(nextPacket.data + 6, nextPacket.size); // + FLTCLR
            QDataStream ds(data);
            FlightControllerValues fcv;
            ds >> fcv;
            tow = fcv.timestamp;
        }
    }
    break;

    default:
        Q_ASSERT("datasource not implemented");
        break;
    }

    return tow;
}

LogPlayer::Data LogPlayer::getNextPacket(const DataSource& source)
{
    Data result;

    switch(source.type)
    {
    case LogTypeLaser:
    {
        if(source.index != -1)
        {
            // A laserscanner-index is defined, so getLastTow() of THAT scanner.
            Q_ASSERT(source.index > 0 && source.index < mLogsLaser.size());

            // If we have no data, just return an empty Data
            if(mLogsLaser.at(source.index)->cursor < 0)
                return result;

            result.data = mLogsLaser.at(source.index)->data.data() + mLogsLaser.at(source.index)->cursor;
            result.size = *((quint16*)mLogsLaser.at(source.index)->data.constData() + mLogsLaser.at(source.index)->cursor + 5);

    //        qDebug() << "laser packet size:" << result.size;
        }
        else
        {
            // No laserscanner index defined!
            Q_ASSERT(false);
        }
    }
    break;

    case LogTypeFlightController:
    {
        // check if uninitialized and out-of-bounds conditions
        if(!mLogFlightController.data.size())
            return result;

        // check if past the last packet
        if(mLogFlightController.data.indexOf("FLTCLR", mLogFlightController.cursor) < 0)
            return result;

        // We can be called with ESTIMATED mIndexFlightController-values. So, search for the next packet's beginning
//        qint32 oldIndex = mIndexFlightController;
        mLogFlightController.cursor = mLogFlightController.data.indexOf("FLTCLR", mLogFlightController.cursor);
        // search for packet-end right after its beginning, otherwise we find out own beginning.
        const qint32 posPacketEnd = mLogFlightController.data.indexOf("FLTCLR", mLogFlightController.cursor + 1);
//        qDebug() << "LogPlayer::getNextPacket(): FLT, index was" <<oldIndex << "found packet from" << mIndexFlightController << "to" << posPacketEnd;

//        result = mLogFlightController.data.mid(mIndexFlightController, posPacketEnd - mIndexFlightController);
        result.data = mLogFlightController.data.data() + mLogFlightController.cursor;
        result.size = posPacketEnd - mLogFlightController.cursor;
    }
    break;

    default:
        Q_ASSERT("datasource not implemented");
        break;
    }

    return result;
}

LogPlayer::DataSource LogPlayer::getNextDataSource(qint32* tow)
{
    // Retrieve TOWs from all devices to figure out who to process next
    qint32 towSbf, towLaser, towFlightController;

    towSbf = getNextTow(DataSource(LogTypeGnss));
    towSbf = towSbf < 0 ? towSbf = INT_MAX : towSbf;

    towFlightController = getNextTow(DataSource(LogTypeFlightController));
    towFlightController = towFlightController < 0 ? towFlightController = INT_MAX : towFlightController;

    qint32 towMin = std::min(towSbf, towFlightController);

    quint32 laserWithMinTow = 0;
    for(int i=0;i<mLogsLaser.size();i++)
    {
        // Whats the TOW of Laser i?
        towLaser = getNextTow(DataSource(LogTypeLaser, i));
        towLaser = towLaser < 0 ? towLaser = INT_MAX : towLaser;

        // Make sure we remember the laser with the lowest TOW found
        if(towLaser < towMin)
            laserWithMinTow = i;

        towMin = std::min(towMin, towLaser);
    }

    if(tow) *tow = towMin == INT_MAX ? -1 : towMin;

    //qDebug() << "LogPlayer::getNextDataSource(): sbf" << towSbf << "lidar" << towLaser << "fc" << towFlightController;

    // If twoSbf and towFlightController are equal, process sbf first, because fc is based on sbf data.
    // That just makes more sense when stepping through the data.
    if(towMin == INT_MAX)
    {
        return DataSource(LogTypeInvalid);
    }
    else if(towMin == towSbf)
    {
        return DataSource(LogTypeGnss);
    }
    else if(towMin == towFlightController)
    {
        return DataSource(LogTypeFlightController);
    }
    else
    {
        return DataSource(LogTypeLaser, laserWithMinTow);
    }

    Q_ASSERT(false && "LogPlayer::getNextDataSource(): illegal state!");
    return LogTypeInvalid;
}

bool LogPlayer::slotStepUntilDataSourceProcessed()
{
    if(mStepUntilLogType == LogTypeInvalid)
    {
        slotStepForward(LogTypeInvalid);
        return true;
    }

    DataSource processedSource;
    do
    {
        processedSource = slotStepForward();
    }
    while(processedSource.type != mStepUntilLogType && processedSource.type != LogTypeInvalid);
    qDebug() << "LogPlayer::slotStepUntilDataSourceProcessed(): processed datasource" << processedSource.type << "- done!";
}

LogPlayer::DataSource LogPlayer::slotStepForward(DataSource source)
{
    if(source.type == LogTypeInvalid) source = getNextDataSource();

    switch(source.type)
    {
    case LogTypeGnss:
    {
        // process SBF
        //qDebug() << "LogPlayer::slotStepForward(): processing" << tow << "sbf";

        // If the mLogGnss.data.at(mIndexSbf) doesn't start with $@, we need to search for a SYNC!
        quint32 offsetToValidPacket;
        mSbfParser->getNextValidPacketInfo(mLogGnss.data, mLogGnss.cursor, &offsetToValidPacket);
        if(mLogGnss.cursor != offsetToValidPacket && offsetToValidPacket - mLogGnss.cursor != 4)
        {
            qDebug() << __PRETTY_FUNCTION__ << "sbf should've started at" << mLogGnss.cursor << "- but did start at" << offsetToValidPacket << "- fixing!";
            mLogGnss.cursor = offsetToValidPacket;
        }

        mLogGnss.cursor += mSbfParser->processNextValidPacket(mLogGnss.data, mLogGnss.cursor);
    }
    break;

    case LogTypeLaser:
    {
        // Process laser data. The source.index must be correct!
        Data packet = getNextPacket(source);
        mLogsLaser.at(source.index)->cursor += packet.size;
        //qDebug() << "LogPlayer::slotStepForward(): processing" << tow << "lidar";
        processPacket(source, packet);
    }
    break;

    case LogTypeFlightController:
    {
        // process FlightController data
        Data packet = getNextPacket(LogTypeFlightController);
        mLogFlightController.cursor += packet.size;
        //qDebug() << "LogPlayer::slotStepForward(): processing" << tow << "fc";
        processPacket(source, packet);
    }
    break;

    default:
    {
        qDebug() << "LogPlayer::slotStepForward(): seems I'm at the end, cannot fetch further Sbf, FlightController or Laser packets from logs";
        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Reached end of log data.");
        return LogTypeInvalid;
    }
    break;

    }

    return source;
}

void LogPlayer::processPacket(const LogPlayer::DataSource& source, const LogPlayer::Data& packet)
{
    switch(source.type)
    {
    case LogTypeLaser:
    {
        qint32 tow = 0;

        if(QByteArray(packet.data, 5) == QByteArray("LASER"))
        {
            // LASER PacketLengthInBytes(quint16) TOW(qint32) StartIndex(quint16) N-DISTANCES(quint16)

            const quint16 length = *((quint16*)(packet.data + 5));

            tow = *((qint32*)(packet.data + 5 + sizeof(length)));

            const quint16 indexStart = *((quint16*)(packet.data + 5 + sizeof(length) + sizeof(tow)));

            const quint16 rayBytes = length
                    - 5               // LASER
                    - sizeof(quint16) // length of whole packet
                    - sizeof(qint32)  // tow
                    - sizeof(quint16);// indexStart

            std::vector<quint16>* data = new std::vector<quint16>(indexStart + (rayBytes / sizeof(quint16)), 1);

            // Copy the distances from the packet to the vector
            memcpy(
                        &data->at(indexStart),
                        packet.data + 5 + sizeof(length) + sizeof(tow) + sizeof(indexStart),
                        rayBytes
                        );

            emit newRayData(&mRelativeLaserPoses[source.index], tow, data);
            mSensorFuser->slotNewScanData(tow, &mRelativeLaserPoses[source.index], data);

        }
        else if(QByteArray(packet.data, 5) == QByteArray("RPOSE"))
        {
            // RPOSE PacketLengthInBytes(quint16) TOW(qint32) QDataStreamedPoseMatrix
            const quint16 length = *((quint16*)(packet.data + 5));

            tow = *((qint32*)(packet.data + 5 + sizeof(length)));

            const QByteArray byteArrayStreamedPoseMatrix(
                        packet.data + 5 + sizeof(length) + sizeof(tow),
                        length - 5 - sizeof(length) - sizeof(tow));

            QMatrix4x4 matrixRelativePose;
            QDataStream ds(byteArrayStreamedPoseMatrix);
            ds >> matrixRelativePose;
            qDebug() << "reconstructed relative scanner" << source.index << "pose" << matrixRelativePose;
            mRelativeLaserPoses[source.index] = Pose(matrixRelativePose, tow);
        }

        mProgressBarTow->setFormat("TOW %v L");
        mProgressBarTow->setValue(tow);
    }
    break;

    case LogTypeFlightController:
    {
        QByteArray data(packet.data, packet.size);
        QDataStream ds(data);
        ds.skipRawData(6); // FLTCLR
        FlightControllerValues fcv;
        ds >> fcv;

        if(!(
                    fcv.controllerThrust.hasSameWeights(&mFlightControllerValues.controllerThrust)
                    &&
                    fcv.controllerYaw.hasSameWeights(&mFlightControllerValues.controllerYaw)
                    &&
                    fcv.controllerPitch.hasSameWeights(&mFlightControllerValues.controllerPitch)
                    &&
                    fcv.controllerRoll.hasSameWeights(&mFlightControllerValues.controllerRoll)
             ))
        {
            // mFlightControllerValues needs to be set before emitting!
            mFlightControllerValues = fcv;
            emit flightControllerWeightsChanged();
        }

        mFlightControllerValues = fcv;

        qDebug() << "FLTCLR:" << mFlightControllerValues.timestamp;

        emit flightControllerValues(&mFlightControllerValues);

        mProgressBarTow->setFormat("TOW %v F");
        mProgressBarTow->setValue(mFlightControllerValues.timestamp);
    }
    break;
    }

}

void LogPlayer::slotPlay()
{
    if(ui->mPushButtonPlay->isChecked())
    {
        qint32 minTowBefore;
        const DataSource ds = getNextDataSource(&minTowBefore);

        if(!mTimePlaybackStartReal.isValid())
        {
            // The first time slotPlay() has been called after loading/rewinding logdata.
            // Take note of GNSS-TOW and real time, so we can synchronize them
            mTimePlaybackStartReal = QTime::currentTime();
            mTimePlaybackStartTow = minTowBefore;
            qDebug() << "LogPlayer::slotPlay(): starting playback, realtime" << mTimePlaybackStartReal << "tow" << mTimePlaybackStartTow;
        }

        if(slotStepForward(ds).type == LogTypeInvalid)
        {
            // Playback failed, we're probably at the end
            qDebug() << "LogPlayer::slotPlay(): slotStepForward() failed, we're probably at the end, stopping timer.";
            emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Stepping forward failed, stopping playback timer.");
            mTimerAnimation->stop();
            ui->mPushButtonPlay->setChecked(false);
            return;
        }

        qint32 minTowAfter;
        getNextDataSource(&minTowAfter);

        mTimerAnimation->stop();

        if(minTowAfter > 0)
        {
            // Packets in the SBF stream are not guaranteed to be in chronological order, especially
            // ExtEvent-packets don't let this assumption hold. For this reason, we might have to deal
            // with negative intervals, which we just set to 0 here.

            // How many milliseconds have elapsed between the first packet to be played and the upcoming one?
            qint32 towElapsedAtNextPacket = minTowAfter - mTimePlaybackStartTow;

            // When should this upcoming packet be played in real time?
            QTime timeOfNextPacketReal = mTimePlaybackStartReal.addMSecs(towElapsedAtNextPacket * ui->mSpinBoxTimeFactor->value());
            qint32 timeToSleep = QTime::currentTime().msecsTo(timeOfNextPacketReal);

//            qDebug() << "LogPlayer::slotPlay(): we are are" << mTimePlaybackStartReal.msecsTo(QTime::currentTime()) << "real ms into playing, next packet comes at" << towElapsedAtNextPacket;
//            qDebug() << "LogPlayer::slotPlay(): playback started at real" << mTimePlaybackStartReal << "tow" << mTimePlaybackStartTow << ": tow of upcoming packet is" << minTowAfter << " factor is" << ui->mSpinBoxTimeFactor->value() << "- sleeping for" << timeToSleep << "ms";

            // Bound time to wait for corner and error-cases
            mTimerAnimation->start(qBound(0, timeToSleep, 5000));
        }
        else
        {
            emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Playback reached end of logfile, stopping.");
            qDebug() << "LogPlayer::slotPlay(): not restarting playtimer, next tow is -1";
            ui->mPushButtonPlay->setChecked(false);

            // Make sensorFuser fuse whatever is left in its queue.
            mSensorFuser->slotFlushData();
        }
    }
    else
    {
        mTimePlaybackStartReal = QTime();
        mTimerAnimation->stop();
    }
}

void LogPlayer::slotNewSbfTime(const qint32 tow,const char*,quint16)
{
    mProgressBarTow->setFormat("TOW %v G");
    mProgressBarTow->setValue(tow);
}

void LogPlayer::slotGoToTow(qint32 towTarget)
{
    if(towTarget < 0)
    {
        bool ok = false;
        towTarget = QInputDialog::getInt(
                    this,
                    "Go To TOW", "Where do you want to seek to?",
                    mProgressBarTow->value(),
                    mProgressBarTow->minimum(),
                    mProgressBarTow->maximum(),
                    1000,
                    &ok,
                    /*Qt::WindowFlags flags = */0 );

        if(!ok) return;
    }

    // We want to point all data-cursors to the first packet with TOW >= towTarget.

    // First, do SBF:
    qint32 indexSbf = mLogGnss.data.size() / 2;
    qint32 stepSize = indexSbf / 2;
    qint32 tow0 = -1, tow1 = -1, tow2 = -1;
    qDebug() << "LogPlayer::slotGoToTow(): towTgt is" << towTarget;
    while(mSbfParser->getNextValidPacketInfo(mLogGnss.data, indexSbf, 0, &tow0))
    {
        qDebug() << "LogPlayer::slotGoToTow(): stepsize is" << stepSize << "indexSbf is" << indexSbf << "of" << mLogGnss.data.size();

        // We're done if we reached the target tow OR the tow doesn't change anymore OR we toggle between tow TOWs
        if(tow0 == towTarget || (tow1 != tow0 && tow0 == tow2))
        {
            break;
        }

        if(tow0 > towTarget)
        {
            qDebug() << "LogPlayer::slotGoToTow(): back, towSbf is" << tow0;
            indexSbf = qBound(0, indexSbf - stepSize, mLogGnss.data.size());
        }
        else if(tow0 < towTarget)
        {
            qDebug() << "LogPlayer::slotGoToTow(): frwd, towSbf is" << tow0;
            indexSbf = qBound(0, indexSbf + stepSize, mLogGnss.data.size());
        }

        tow2 = tow1;
        tow1 = tow0;

        // Step at least one packet, which is at least 16 bytes
        stepSize = std::max(16, stepSize/2);
    }

    mLogGnss.cursor = indexSbf;
    mSbfParser->getNextValidPacketInfo(mLogGnss.data, mLogGnss.cursor, 0, &tow0);
    qDebug() << "LogPlayer::slotGoToTow(): SBF: reached TOW" << tow0 << ", targeted was" << towTarget;

    // LASER
    for(int i=0;i<mLogsLaser.size();i++)
    {
        const qint32 laserLogSize = mLogsLaser[i]->data.size();
        qint32 dataCursor = laserLogSize / 2;
        stepSize = dataCursor / 2;
        tow0 = 0; tow1 = 0; tow2 = 0;
        while(tow0 >= 0)
        {
            tow0 = getNextTow(LogTypeLaser);

            // We're done if
            // - we reached the target tow
            // - we toggle between TOWs
            // - we reached beginning or end of data
            if(tow0 == towTarget || (tow1 != tow0 && tow0 == tow2) || dataCursor == 0 || dataCursor == laserLogSize)
            {
                break;
            }

            if(tow0 > towTarget)
            {
                qDebug() << "LogPlayer::slotGoToTow(): back, towLaser is" << tow0;

                // We need to go back stepsize bytes, or exactly one laser packet if stepsize
                // bytes is below the packet's size. Thus, determine the minimum size to move.
                const qint32 bytesToMoveForPreviousPacket = dataCursor - mLogsLaser[i]->data.lastIndexOf("LASER", dataCursor-1);
                const qint32 bytesToMove = std::max(stepSize, bytesToMoveForPreviousPacket);
                qDebug() << "LogPlayer::slotGoToTow(): stepsize is" << stepSize << "bytestomove" << bytesToMove << "indexLaser is" << dataCursor << "of" << mLogsLaser[i]->data.size();
                dataCursor = qBound(0, dataCursor - bytesToMove, mLogsLaser[i]->data.size());
            }
            else if(tow0 < towTarget)
            {
                qDebug() << "LogPlayer::slotGoToTow(): frwd, towLaser is" << tow0;

                // We need to go forward stepsize bytes, or exactly one laser packet if stepsize
                // bytes is below the packet's size. Thus, determine the minimum size to move.
                const qint32 bytesToMoveForNextPacket = mLogsLaser[i]->data.indexOf("LASER", dataCursor+1) - dataCursor;
                const qint32 bytesToMove = std::max(stepSize, bytesToMoveForNextPacket);
                qDebug() << "LogPlayer::slotGoToTow(): stepsize is" << stepSize << "bytestomove" << bytesToMove << "indexLaser is" << dataCursor << "of" << mLogsLaser[i]->data.size();
                dataCursor = qBound(0, dataCursor + bytesToMove, mLogsLaser[i]->data.size());
            }

            tow2 = tow1;
            tow1 = tow0;

            // Step at least one packet, which is around 1500 bytes
            stepSize = /*std::max(500, */stepSize/2;//);
        }
        mLogsLaser[i]->cursor = dataCursor;
        qDebug() << "LogPlayer::slotGoToTow(): LASER" << i << ": reached TOW" << tow0 << ", targeted was" << towTarget;
    }

    // FlightController
    mLogFlightController.cursor = mLogFlightController.data.size() / 2;
    stepSize = mLogFlightController.cursor / 2;
    tow0 = 0; tow1 = 0; tow2 = 0;
    const qint32 flightControllerPacketSize = getNextPacket(LogTypeFlightController).size;
    while(mLogFlightController.data.size() && tow0 >= 0)
    {
        tow0 = getNextTow(LogTypeFlightController);
        qDebug() << "LogPlayer::slotGoToTow(): sizeofPacket" << 6+sizeof(FlightControllerValues) << "stepsize is" << stepSize << "indexFltClr is" << mLogFlightController.cursor << "of" << mLogFlightController.data.size();

        // We're done if we reached the target tow OR the tow doesn't change anymore OR we toggle between tow TOWs
        if(tow0 == towTarget || (tow0 == tow2 && tow1 < tow0))
        {
            break;
        }

        if(tow0 > towTarget)
        {
            qDebug() << "LogPlayer::slotGoToTow(): towFltClr is" << tow0 << "going backward" << stepSize << "bytes...";
            mLogFlightController.cursor = qBound(0, mLogFlightController.cursor - stepSize, mLogFlightController.data.size());
        }
        else if(tow0 < towTarget)
        {
            qDebug() << "LogPlayer::slotGoToTow(): towFltClr is" << tow0 << "going forwards" << stepSize << "bytes...";
            mLogFlightController.cursor = qBound(0, mLogFlightController.cursor + stepSize, mLogFlightController.data.size());
        }

        tow2 = tow1;
        tow1 = tow0;

        // Don't let stepsize go below the packetSize, else jumping stepSize will not get new packets
        stepSize = std::max(flightControllerPacketSize, stepSize/2);
    }

    // Clear SensorFuser data, because otherwise the next data isn't guaranteed to come in in chronological order
    mSensorFuser->slotClearData();

    qDebug() << "LogPlayer::slotGoToTow(): FLTCLR: reached TOW" << tow0 << ", targeted was" << towTarget;

    // TODO: if currently playing, reset mTimePlaybackStart* and friends

    mProgressBarTow->setValue(towTarget);
}

/*void LogPlayer::keyPressEvent(QKeyEvent* event)
{
    if(event->key() == Qt::Key_Space)
    {
        ui->mPushButtonPlay->setChecked(!ui->mPushButtonPlay->isChecked());
        slotPlay();
    }
}*/

void LogPlayer::slotStepDataSourceChanged(const int logType)
{
    // When pressing STEP the next time, step util we process a packet from datasource
    mStepUntilLogType = (LogType)logType;

    switch(mStepUntilLogType)
    {
        case LogTypeGnss: ui->mPushButtonStepForward->setText("Step/G"); break;
        case LogTypeLaser: ui->mPushButtonStepForward->setText("Step/L"); break;
        case LogTypeFlightController: ui->mPushButtonStepForward->setText("Step/F"); break;
        case LogTypeInvalid: ui->mPushButtonStepForward->setText("Step/A"); break;
    }

    // The menu was used to select a source. Lets process this packet!
    slotStepUntilDataSourceProcessed();
}
