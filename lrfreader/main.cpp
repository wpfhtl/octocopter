#include "laserscanner.h"

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);

    LaserScanner ls("/dev/serial/by-id/usb-Hokuyo_Data_Flex_for_USB_URG-Series_USB_Driver-if00");
    ls.run();

    return app.exec();
}
