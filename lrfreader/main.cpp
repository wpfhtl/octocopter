#include "laserscanner.h"
#include "pose.h"
#include <QDebug>
#include <QTime>
#include <pthread.h>

int main(int argc, char *argv[])
{
    qDebug() << "main(): starting in process" << getpid() << "thread" << pthread_self();
    QCoreApplication app(argc, argv);
    LaserScanner* ls = new LaserScanner(
                "/dev/serial/by-id/usb-Hokuyo_Data_Flex_for_USB_URG-Series_USB_Driver-if00",
                Pose(
                    QVector3D(      // Offset from vehicle center to Laser Source. In Vehicle Reference Frame: Like OpenGL, red arm forward pointing to screen
                        +0.00,      // From vehicle left/right to laser, positive is moved to right "wing"
                        -0.04,      // From vehicle up/down to laser, negative is down to laser
                        -0.14),     // From vehicle 14cm forward, towards the front arm.
                    +000.0,         // No yawing
                    -090.0,         // 90 deg pitched down
                    +000.0,         // No rolling
                    10             // Use 10 msec TOW, so that the relative pose is always older than whatever new pose coming in. Don't use 0, as that would be set to current TOW, which might be newer due to clock offsets.
                    ),
                QString("lrfreader")
                );

    ls->slotEnableScanning(true);

    QTimer::singleShot(15000, ls, SLOT(slotDisableScanning()));

    return app.exec();
}
