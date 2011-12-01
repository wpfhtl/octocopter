#include <QCoreApplication>
#include "laserscanner.h"
#include "sensorfuser.h"
#include <plymanager.h>

int main(int argc, char **argv)
{
    QCoreApplication a(argc, argv);

    const QStringList arguments = QCoreApplication::arguments();

    if(arguments.size() != 2)
    {
        qDebug() << "main(): please specify a logfile for replaying sensor fusion as the only argument.";
        return 0;
    }

    LaserScanner* laserScanner = new LaserScanner(
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

    // Otherwise, SensorFuser won't accept poses.
    laserScanner->slotEnableScanning(true);

    SensorFuser* sensorFuser = new SensorFuser(laserScanner, false);

    PlyManager plyManager("/mnt/temp/logreplay.ply", PlyManager::DataWrite);
    QObject::connect(sensorFuser, SIGNAL(newScannedPoints(QVector<QVector3D>, QVector3D)), &plyManager, SLOT(slotNewPoints(QVector<QVector3D>, QVector3D)));

    sensorFuser->processLog(arguments.last());

    delete sensorFuser;
    delete laserScanner;

    return 0;
}
