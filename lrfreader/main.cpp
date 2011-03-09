#include "laserscanner.h"

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);

    LaserScanner ls("/dev/ttyACM0");
    ls.start();

    return app.exec();
}
