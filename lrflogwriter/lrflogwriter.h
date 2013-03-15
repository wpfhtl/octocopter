#include <laserscanner.h>
#include <QDebug>

class LrfLogWriter : public QCoreApplication
{
    Q_OBJECT

private slots:
    void slotNewScanData(qint32 timestampScanner, std::vector<quint16> * const distances);

private:
    LaserScanner* mScanner;

public:
    LrfLogWriter(int argc, char **argv);
    ~LrfLogWriter();
};
