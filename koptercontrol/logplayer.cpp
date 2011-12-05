#include "logplayer.h"

LogPlayer::LogPlayer(int argc, char **argv) : QCoreApplication(argc, argv)
{
    const QStringList arguments = QCoreApplication::arguments();

    if(arguments.size() != 3)
    {
        qDebug() << "LogPlayer::LogPlayer() usage:" << arguments.at(0) << "inputfile outputfile";
        exit(0);
    }

    mLaserScanner = new LaserScanner(
                QString("/does/not/exist"),
                Pose(
                    QVector3D(      // Offset from Antenna to Laser Source. In Vehicle Reference Frame: Like OpenGL, red arm forward pointing to screen
                        +0.09,      // From antenna positive is right to laser
                        -0.38,      // From Antenna negative is down to laser
                        -0.11),     // From Antenna negative forward to laser
                    +000.0,         // No yawing
                    -090.0,         // 90 deg pitched down
                    +000.0,         // No rolling
                    10             // Use 10 msec TOW, so that the relative pose is always older than whatever new pose coming in. Don't use 0, as that would be set to current TOW, which might be newer due to clock offsets.
                    )
                );

    // Otherwise, LogPlayer won't accept poses.
    mLaserScanner->slotEnableScanning(true);

    mSensorFuser = new SensorFuser(mLaserScanner, static_cast<SensorFuser::Behavior>(SensorFuser::FuseData));

    mPlyManager = new PlyManager(arguments.at(2), PlyManager::DataWrite);
    connect(mSensorFuser, SIGNAL(newScannedPoints(QVector<QVector3D>, QVector3D)), mPlyManager, SLOT(slotNewPoints(QVector<QVector3D>, QVector3D)));

    slotProcessLog(arguments.at(1));
}

LogPlayer::~LogPlayer()
{
    delete mLaserScanner;
    delete mSensorFuser;
    delete mPlyManager;
}

bool LogPlayer::slotProcessLogLine(const QString& line)
{
    if(line.contains("scannerdata"))
    {
        const QStringList tokens = line.split(' ');
        bool success = false;
        const qint32 timestamp = tokens.at(1).toInt(&success);
        if(!success) {qDebug() << "LogPlayer::processLogLine(): couldn't parse scannerdata-timestamp."; return false;}

        std::vector<long>* data = new std::vector<long>;
        for(int i=2;i<tokens.size();i++)
        {
            data->push_back(tokens.at(i).toInt(&success));
            if(!success) {qDebug() << "LogPlayer::processLogLine(): couldn't parse scannerdata-distance at index" << i; return false;}
        }

        mSensorFuser->slotNewScanData(timestamp, data);
    }
    else if(line.contains("extevent"))
    {
        bool success = false;
        mSensorFuser->slotScanFinished(line.split(' ').at(1).toInt(&success));
        if(!success) {qDebug() << "LogPlayer::processLogLine(): couldn't parse extevent"; return false;}
    }
    else if(line.contains("pose"))
    {
        mSensorFuser->slotNewVehiclePose(Pose(line));
    }
    else
    {
        qDebug() << "LogPlayer::processLogLine(): couldn't parse line" << line;
        return false;
    }

    return true;
}

bool LogPlayer::slotProcessLog(const QString& fileName)
{
    QTime startTime;
    startTime.start();

    QFile file(fileName);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "LogPlayer::processLog(): cannot open file" << fileName << "for reading.";
        return false;
    }

    quint32 lineNumber = 0;
    QTextStream in(&file);
    while (!in.atEnd())
    {
        const QString line = in.readLine();
        if(!slotProcessLogLine(line))
        {
            qDebug() << "LogPlayer::processLog(): trouble parsing" << fileName << "line" << lineNumber << ":" << line;
            return false;
        }
        lineNumber++;
    }

    qDebug() << "LogPlayer::processLog(): processing time was" << startTime.elapsed() << "msecs.";
    return true;
}

int main(int argc, char **argv)
{

    LogPlayer logPlayer(argc, argv);

    return 0;
}
