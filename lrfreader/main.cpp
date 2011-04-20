#include "laserscanner.h"

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);

    LaserScanner ls("/dev/ttyACM1");
    ls.start();

    return app.exec();
}
