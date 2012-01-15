#ifndef LOGPLAYER_H
#define LOGPLAYER_H

#include <QtCore>
#include "laserscanner.h"
#include "sensorfuser.h"
#include <plymanager.h>

class LogPlayer : public QCoreApplication
{
    Q_OBJECT

private:
    LaserScanner* mLaserScanner;
    SensorFuser* mSensorFuser;
    PlyManager* mPlyManagerCloud;
    PlyManager* mPlyManagerTrajectory;

public:
    LogPlayer(int argc, char **argv);
    ~LogPlayer();

public slots:
    bool slotProcessLogLine(const QString& line);
    bool slotProcessLog(const QString& fileName);

signals:
};

#endif
