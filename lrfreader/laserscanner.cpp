#include "laserscanner.h"

LaserScanner::LaserScanner(const QString &deviceFileName)
{
    qDebug() << "LaserScanner::LaserScanner()";

    mDeviceFileName = deviceFileName;

    if (! mScanner.connect(mDeviceFileName.toAscii().constData()))
    {
      qDebug() << "LaserScanner::LaserScanner(): connecting to" << mDeviceFileName << "failed: UrgCtrl::connect gave" << mScanner.what();
      exit(1);
    }

    mScanner.setCaptureMode(IntensityCapture);

    //setTime(0);
}

void LaserScanner::setTime(const quint32& time)
{
    int response_msec = 0;
    int force_delay_msec = 0;

    qDebug() << getTime() << "LaserScanner::setTime(): setting time to" << time;

    bool ok = mScanner.setTimestamp(time, &response_msec, NULL);

    qDebug() << getTime() << "LaserScanner::setTime(): success:" << ok << "response_msec is" << response_msec;
}

LaserScanner::~LaserScanner()
{
    qDebug() << "LaserScanner::~LaserScanner()";

}

QString LaserScanner::getTime()
{
    return QTime::currentTime().toString("HH:mm:ss:zzz");
}

void LaserScanner::run()
{
    mStartUpTime = QDateTime::currentDateTime();

    while(true)
    {
      long timestamp = 0;

      timer.start();

      // Get data with intensity information
      int data_n = mScanner.captureWithIntensity(mScanDistances, mScanIntensities, &timestamp);

      if(data_n > 0)
      {
        printf("runtime %4d [s]: utm30lx time %ld [msec]\n",
               mStartUpTime.secsTo(QDateTime::currentDateTime()),
               timestamp);
      }
      else
      {
        usleep(25000);
      }
    }
}
