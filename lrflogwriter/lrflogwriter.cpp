#include <lrflogwriter.h>

void LrfLogWriter::slotNewScanData(qint32 timestampScanner, std::vector<quint16> * const distances)
{
    quint16 startIndex = 0;
    while(distances->at(startIndex) == 1)
        startIndex++;

    quint16 stopIndex = distances->size() - 1;
    while(distances->at(stopIndex) == 1)
        stopIndex--;

    qDebug() << distances->size() << "distances, start is" << startIndex << "stop is" << stopIndex;

    printf("%d: ", timestampScanner);
    for(int i=startIndex;i<=stopIndex;i++)
        printf("%d ", distances->at(i));
    printf(".\n");
}

LrfLogWriter::LrfLogWriter(int argc, char **argv) : QCoreApplication(argc, argv)
{
    mScanner = new LaserScanner(
                "/dev/serial/by-id/usb-Hokuyo_Data_Flex_for_USB_URG-Series_USB_Driver-if00",
                Pose(),
                QString("logwriter-"));

    mScanner->slotSetScannerTimeStamp(1000000);

    mScanner->slotEnableScanning();

    connect(mScanner, SIGNAL(newScanData(qint32,std::vector<quint16>*const)), SLOT(slotNewScanData(qint32,std::vector<quint16>*const)));

    QTimer::singleShot(100, this, SLOT(quit()));
}

LrfLogWriter::~LrfLogWriter()
{
    delete mScanner;
}


int main(int argc, char **argv)
{

    LrfLogWriter llw(argc, argv);
    return llw.exec();
}
