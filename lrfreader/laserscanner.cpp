#include "laserscanner.h"

LaserScanner::LaserScanner(const QString &deviceFileName)
{
    qDebug() << "LaserScanner::LaserScanner()";

    mDeviceFileName = deviceFileName;

    mSpeed = 2400.0f;
    mAngleStart = 45.0f;
    mAngleStop = 315.0f;
    mAngleStep = 0.25f;

    mTimerScan = 0;

    mCurrentScanAngle = mAngleStart;

    if (! mScanner.connect(mDeviceFileName.toAscii().constData()))
    {
      qDebug() << "LaserScanner::LaserScanner(): connecting to" << mDeviceFileName << "failed: UrgCtrl::connect gave" << mScanner.what();
      exit(1);
    }

    int scan_msec = mScanner.scanMsec();
    mScanner.setCaptureMode(IntensityCapture);








}

LaserScanner::~LaserScanner()
{
    qDebug() << "LaserScanner::~LaserScanner()";

}

void LaserScanner::run()
{
    //    enum {
    //      CaptureTimes = 10,
    //    };

    QTime timer;

    forever/*(int i = 0; i < CaptureTimes; ++i)*/
    {
      long timestamp = 0;

      timer.start();

      // Get data with intensity information
      int data_n = mScanner.captureWithIntensity(mScanDistances, mScanIntensities, &timestamp);

      if(data_n > 0)
      {
	int front_index = mScanner.rad2index(0.0);

	// Display
	// The distance data that are less than urg_minDistance() are shown as invalid value.
        printf("%d %d: %ld [mm] (%ld), %ld [msec]\n", timer.restart(), front_index, mScanDistances[front_index], mScanIntensities[front_index], timestamp);
      }
      else
      {
          qDebug() << "call took" << timer.restart() << "data_n was" << data_n << ", waiting" << mScanner.scanMsec()*1000 << "nanoseconds";
          usleep(mScanner.scanMsec()*1000);
      }
    }
}

void LaserScanner::slotDoScan()
{
}


void LaserScanner::slotSetScannerPose(const QVector3D &position, const QQuaternion &orientation)
{
    // This slot is called whenever the vehicle's position changes. Thus, we
    // should be called at least 25, at most mSpeed/60 times per second.
    QMutexLocker locker(&mMutex);
    mPosition = position;
    mOrientation = orientation;
}

void LaserScanner::slotPause(void)
{
    qDebug() << "LaserScanner::slotPause(): stopping scanner";
    mTimerScan->stop();
}

void LaserScanner::slotStart(void)
{
    qDebug() << "LaserScanner::slotStart(): starting scanner timer in thread" << currentThreadId();
    mTimerScan->start();
}

// Getters for the properties
float LaserScanner::range(void) const
{
    return mRange;
}

int LaserScanner::speed(void) const
{
    return (int)mSpeed;
}

int LaserScanner::angleStart(void) const
{
    return mAngleStart;
}

int LaserScanner::angleStop(void) const
{
    return mAngleStop;
}

float LaserScanner::angleStep(void) const
{
    return mAngleStep;
}

// Setters for the properties
void LaserScanner::setRange(float range)
{
    mRange = range;
    mRangeSquared = pow(mRange, 2.0);
}

void LaserScanner::setSpeed(int speed)
{
    mSpeed = speed;
}

void LaserScanner::setAngleStart(int angleStart)
{
    mAngleStart = angleStart;
}

void LaserScanner::setAngleStop(int angleStop)
{
    mAngleStop = angleStop;
}

void LaserScanner::setAngleStep(float angleStep)
{
    mAngleStep = angleStep;
}

void LaserScanner::setPosition(const QVector3D &position)
{
    QMutexLocker locker(&mMutex);
    mPosition = position;

}

void LaserScanner::setOrientation(const QQuaternion &orientation)
{
    QMutexLocker locker(&mMutex);
    mOrientation = orientation;
}

QVector3D LaserScanner::getPosition(void)
{
    return mPosition;
}

QQuaternion LaserScanner::getOrientation(void)
{
    return mOrientation;
}

bool LaserScanner::isScanning(void) const
{
    return true;
}
