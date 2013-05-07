#include "hokuyo.h"

#include <QDebug>

#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <poll.h>
#include <signal.h>

#include <algorithm>

#include <time.h>

#include <fcntl.h>

#include <sys/time.h>

//! Macro for throwing an exception with a message, passing args
#define HOKUYO_EXCEPT(except, msg, ...) \
  { \
    char buf[1000]; \
    snprintf(buf, 1000, msg " (in Hokuyo::%s) You may find further details at http://www.ros.org/wiki/hokuyo_node/Troubleshooting" , ##__VA_ARGS__, __FUNCTION__); \
    throw except(buf); \
  }

Hokuyo::Hokuyo(const QString &logFilePrefix) : QObject(), mState(State::Stopped),
    mDmin(0), mDmax(0), mAres(0), mAmin(0), mAmax(0), mAfrt(0), mRate(0), mLatencyOffset(0),
    mWrapped(0), mLastTime(0), mTimestampRepeatCount(0), mIsLatencyDetermined(false),
    mLaserFd(-1), mHeightOverGroundClockDivisor(0)
{
    mLogFile = new LogFile(logFilePrefix + QString("scannerdata.lsr"), LogFile::Encoding::Binary);

    qDebug() << "Hokuyo::Hokuyo(): registering meta type(s)";
    //qRegisterMetaType<std::vector<quint16>*const >("std::vector<quint16>*const");

    qRegisterMetaType<std::vector<quint16>*>("std::vector<quint16>*");
}

Hokuyo::~Hokuyo ()
{
    if(portOpen())
    {
        laserOff();
        close();
        qDebug() << "Hokuyo::~Hokuyo(): laser switched off, port closed.";
    }

    qDebug() << "Hokuyo::~Hokuyo(): laserscanner is shut down, now closing logfile...";
    delete mLogFile;
}

bool Hokuyo::open(const QString &portName)
{
    if(portOpen())
        close();

    // Make IO non blocking. This way there are no race conditions that cause blocking when a badly behaving process does a read at the same
    // time as us. Will need to switch to blocking for writes or errors occur just after a replug event.
    mLaserFd = ::open(qPrintable(portName), O_RDWR | O_NONBLOCK | O_NOCTTY);
    mReadBufferStart = mReadBufferEnd = 0;

    if(mLaserFd == -1)
    {
        QString extraMessage;
        switch (errno)
        {
        case EACCES:
            extraMessage = "You probably don't have premission to open the port for reading and writing.";
            break;
        case ENOENT:
            extraMessage = "The requested port does not exist. Is the hokuyo connected? Was the port name misspelled?";
            break;
        }

        qDebug() << "Holuyo::open(): failed to open port" << portName << strerror(errno) << errno << extraMessage;
        return false;
    }
    try
    {
        struct flock fl;
        fl.l_type   = F_WRLCK;
        fl.l_whence = SEEK_SET;
        fl.l_start = 0;
        fl.l_len   = 0;
        fl.l_pid   = getpid();

        if(fcntl(mLaserFd, F_SETLK, &fl) != 0)
            qFatal("Device %s is already locked. Try 'lsof | grep %s' to find other processes that currently have the port open.", qPrintable(portName), qPrintable(portName));

        // Settings for USB?
        struct termios newtio;
        tcgetattr(mLaserFd, &newtio);
        memset (&newtio.c_cc, 0, sizeof (newtio.c_cc));
        newtio.c_cflag = CS8 | CLOCAL | CREAD;
        newtio.c_iflag = IGNPAR;
        newtio.c_oflag = 0;
        newtio.c_lflag = 0;

        // activate new settings
        tcflush (mLaserFd, TCIFLUSH);
        if(tcsetattr (mLaserFd, TCSANOW, &newtio) < 0)
            qFatal("Unable to set serial port attributes. The port you specified (%s) may not be a serial port.", qPrintable(portName)); /// @todo tcsetattr returns true if at least one attribute was set. Hence, we might not have set everything on success.
        usleep (200000);

        // Some models (04LX) need to be told to go into SCIP2 mode...
        laserFlush();
        // Just in case a previous failure mode has left our Hokuyo
        // spewing data, we send reset the laser to be safe.
        try {
            reset();
        }
        catch (Exception &e)
        {
            // This might be a device that needs to be explicitely placed in
            // SCIP2 mode.
            // Note: Not tested: a device that is currently scanning in SCIP1.1
            // mode might not manage to switch to SCIP2.0.

            setToSCIP2(); // If this fails then it wasn't a device that could be switched to SCIP2.
            reset(); // If this one fails, it really is an error.
        }

        querySensorConfig();

        queryVersionInformation(); // In preparation for calls to get various parts of the version info.
    }
    catch (Exception& e)
    {
        // These exceptions mean something failed on open and we should close
        qDebug() << "Hokuyo::open(): exception while setting up communication";

        if(mLaserFd != -1)
            ::close(mLaserFd);
        mLaserFd = -1;
        return false;
    }

    return true;
}

void Hokuyo::reset ()
{
    Q_ASSERT(portOpen());

    laserFlush();
    try
    {
        sendCmd("TM2", 1000);
    }
    catch (Exception &e) {} // Ignore. If the laser was scanning TM2 would fail
    try
    {
        sendCmd("RS", 1000);
        mLastTime = 0; // RS resets the hokuyo clock.
        mWrapped = 0; // RS resets the hokuyo clock.
    }
    catch (Exception &e) {} // Ignore. If the command coincided with a scan we might get garbage.
    laserFlush();
    sendCmd("RS", 1000); // This one should just work.
}

void Hokuyo::close ()
{
    int retval = 0;

    if(portOpen())
    {
        // Try to be a good citizen and completely shut down the laser before we shutdown communication
        try
        {
            reset();
        }
        catch (Exception& e) {
            //Exceptions here can be safely ignored since we are closing the port anyways
        }

        retval = ::close(mLaserFd); // Automatically releases the lock.
    }

    mLaserFd = -1;

    if(retval != 0)
        qFatal("Failed to close port properly -- error = %d: %s\n", errno, strerror(errno));
}


void Hokuyo::setToSCIP2()
{
    Q_ASSERT(portOpen());
    const char * cmd = "SCIP2.0";
    char buf[100];
    laserWrite(cmd);
    laserWrite("\n");

    laserReadline(buf, 100, 1000);
    qDebug() << "Hokuyo::setToSCIP2(): laser protocol changed to" << buf;
}

int Hokuyo::sendCmd(const char* cmd, int timeout)
{
    Q_ASSERT(portOpen());

    char buf[100];

    laserWrite(cmd);
    laserWrite("\n");

    laserReadlineAfter(buf, 100, cmd, timeout);
    laserReadline(buf, 100, timeout);

    if(!checkSum(buf,4))
        HOKUYO_EXCEPT(CorruptedDataException, "Checksum failed on status code.");

    buf[2] = 0;

    if(buf[0] - '0' >= 0 && buf[0] - '0' <= 9 && buf[1] - '0' >= 0 && buf[1] - '0' <= 9)
        return (buf[0] - '0') * 10 + (buf[1] - '0');
    else
    {
        HOKUYO_EXCEPT(Exception, "Hokuyo error code returned. Cmd: %s --  Error: %s", cmd, buf);
    }
}

void Hokuyo::getConfig(HokuyoConfig& config)
{
    config.angleStart  =  (mAmin - mAfrt) * (2.0*M_PI)/(mAres);
    config.angleStop  =  (mAmax - mAfrt) * (2.0*M_PI)/(mAres);
    config.angleIncrement =  (2.0*M_PI)/(mAres);
    config.timeIncrement = (60.0f)/(float)(mRate * mAres);
    config.timeBetweenScans = 60.0f/((float)(mRate));
    config.rangeMin  =  mDmin / 1000.0f;
    config.rangeMax  =  mDmax / 1000.0f;

    qDebug() << "Hokuyo::getConfig():" << config.toString();
}



int Hokuyo::laserWrite(const char* msg)
{
    // IO is currently non-blocking. This is what we want for the more common read case.
    const int originalFlags = fcntl(mLaserFd,F_GETFL,0);
    fcntl(mLaserFd, F_SETFL, originalFlags & ~O_NONBLOCK); // @todo can we make this all work in non-blocking?
    const ssize_t length = strlen(msg);
    const ssize_t retval = write(mLaserFd, msg, length);
    const int fputserrno = errno;
    fcntl(mLaserFd, F_SETFL, originalFlags | O_NONBLOCK);
    errno = fputserrno; // Don't want to see the fcntl errno below.

    if(retval != -1)
    {
        return retval;
    }
    else
    {
        HOKUYO_EXCEPT(Exception, "fputs failed -- Error = %d: %s", errno, strerror(errno));
    }
}

int Hokuyo::laserFlush()
{
    const int retval = tcflush(mLaserFd, TCIOFLUSH);
    if(retval != 0)  HOKUYO_EXCEPT(Exception, "tcflush failed");
    mReadBufferStart = 0;
    mReadBufferEnd = 0;
    return retval;
}

int Hokuyo::laserReadline(char *buf, int len, int timeout)
{
    int current = 0;

    struct pollfd ufd[1];
    int retval;
    ufd[0].fd = mLaserFd;
    ufd[0].events = POLLIN;

    if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.

    while(true)
    {
        if(mReadBufferStart == mReadBufferEnd) // Need to read?
        {
            if((retval = poll(ufd, 1, timeout)) < 0)
                HOKUYO_EXCEPT(Exception, "poll failed   --  error = %d: %s", errno, strerror(errno));

            if(retval == 0)
                HOKUYO_EXCEPT(TimeoutException, "timeout reached");

            if(ufd[0].revents & POLLERR)
                HOKUYO_EXCEPT(Exception, "error on socket, possibly unplugged");

            const int bytes = read(mLaserFd, mReadBuffer, sizeof(mReadBuffer));

            if(bytes == -1 && errno != EAGAIN && errno != EWOULDBLOCK)
                HOKUYO_EXCEPT(Exception, "read failed");

            mReadBufferStart = 0;
            mReadBufferEnd = bytes;
        }

        while (mReadBufferEnd != mReadBufferStart)
        {
            if(current == len - 1)
            {
                buf[current] = 0;
                HOKUYO_EXCEPT(Exception, "buffer filled without end of line being found");
            }

            buf[current] = mReadBuffer[mReadBufferStart++];
            if(buf[current++] == '\n')
            {
                buf[current] = 0;
                return current;
            }
        }
    }
}


char* Hokuyo::laserReadlineAfter(char* buf, int len, const char* str, int timeout)
{
    buf[0] = 0;
    char* ind = &buf[0];

    int bytes_read = 0;
    int skipped = 0;

    while ((strncmp(buf, str, strlen(str))) != 0)
    {
        bytes_read = laserReadline(buf,len,timeout);

        if((skipped += bytes_read) > mMaximumBytesToSkipInResponse)
            HOKUYO_EXCEPT(Exception, "too many bytes skipped while searching for match");
    }

    return ind += strlen(str);
}



void Hokuyo::querySensorConfig()
{
    Q_ASSERT(portOpen());

    if(sendCmd("PP",1000) != 0)
        HOKUYO_EXCEPT(Exception, "Error requesting configuration information");

    char buf[100];
    char* ind;

    ind = laserReadlineAfter(buf,100,"DMIN:",-1);
    sscanf(ind, "%d", &mDmin);

    ind = laserReadlineAfter(buf,100,"DMAX:",-1);
    sscanf(ind, "%d", &mDmax);

    ind = laserReadlineAfter(buf,100,"ARES:",-1);
    sscanf(ind, "%d", &mAres);

    ind = laserReadlineAfter(buf,100,"AMIN:",-1);
    sscanf(ind, "%d", &mAmin);

    ind = laserReadlineAfter(buf,100,"AMAX:",-1);
    sscanf(ind, "%d", &mAmax);

    ind = laserReadlineAfter(buf,100,"AFRT:",-1);
    sscanf(ind, "%d", &mAfrt);

    ind = laserReadlineAfter(buf,100,"SCAN:",-1);
    sscanf(ind, "%d", &mRate);

    qDebug() << "Hokuyo::querySensorConfig(): dmin (min dist)" << mDmin << "dmax (max dist)" << mDmax << "ares (num rays in 360 deg)" << mAres << "amin (first ray)" << mAmin << "amax (last ray)" << mAmax << "afrt (front ray)" << mAfrt << "mrate (rpm)" << mRate;

    return;
}

bool Hokuyo::checkSum(const char* buf, int buf_len)
{
    char sum = 0;
    for(int i = 0; i < buf_len - 2; i++)
        sum += (unsigned char)(buf[i]);

    if((sum & 63) + 0x30 == buf[buf_len - 2])
        return true;
    else
        return false;
}

qint64 Hokuyo::readScannerTimeStamp(int timeout)
{
    char buf[100];

    laserReadline(buf, 100, timeout);
    if(!checkSum(buf, 6))
        HOKUYO_EXCEPT(CorruptedDataException, "Checksum failed on time stamp.");

    const quint32 laserTime = ((buf[0]-0x30) << 18) | ((buf[1]-0x30) << 12) | ((buf[2]-0x30) << 6) | (buf[3] - 0x30);

    if(laserTime == mLastTime)
    {
        if(++mTimestampRepeatCount > 2)
        {
            HOKUYO_EXCEPT(RepeatedTimeException, "The timestamp has not changed for %d reads", mTimestampRepeatCount);
        }
        else if(mTimestampRepeatCount > 0)
            qDebug() << "Hokuyo::readScannerTimeStamp(): the timestamp has not changed for" << mTimestampRepeatCount << "reads. Ignoring for now.";
    }
    else
    {
        mTimestampRepeatCount = 0;
    }

    if(laserTime < mLastTime)
        mWrapped++;

    mLastTime = laserTime;

    const qint64 scannerTimeInMicroSeconds = (qint64)((mWrapped << 24) | laserTime)*(1000);
//    qDebug() << "Hokuyo::readScannerTimeStamp(): scanner timestamp is" << scannerTimeInMicroSeconds << "usec, which is" << scannerTimeInMicroSeconds/1000000 << "s.";
    return scannerTimeInMicroSeconds;
}

void Hokuyo::readData(LaserScan& scan, bool hasIntensitiy, int timeout)
{
    scan.ranges.clear();
    scan.intensities.clear();

    int bytesPerRay = 3;
    if(hasIntensitiy)
        bytesPerRay = 6;

    char buf[100];

    int ind = 0;

    scan.timeStampScanner = readScannerTimeStamp(timeout);

    int bytes;

    for(;;)
    {
        bytes = laserReadline(&buf[ind], 100 - ind, timeout);

        if(bytes == 1)          // This is \n\n so we should be done
            return;

        if(!checkSum(&buf[ind], bytes))
            HOKUYO_EXCEPT(CorruptedDataException, "Checksum failed on data read.");

        bytes += ind - 2;

        // Read as many ranges as we can get
        for (int j = 0; j < bytes - (bytes % bytesPerRay); j+=bytesPerRay)
        {
            if(scan.ranges.size() < mMaximumRaysRead)
            {
                scan.ranges.push_back(((buf[j+0]-0x30) << 12) | ((buf[j+1]-0x30) << 6) | (buf[j+2]-0x30));

                if(hasIntensitiy)
                {
                    scan.intensities.push_back(((buf[j+3]-0x30) << 12) | ((buf[j+4]-0x30) << 6) | (buf[j+5]-0x30));
                }
            }
            else
            {
                HOKUYO_EXCEPT(CorruptedDataException, "Got more readings than expected");
            }
        }

        // Shuffle remaining bytes to front of buffer to get them on the next loop
        ind = 0;
        for (int j = bytes - (bytes % bytesPerRay); j < bytes ; j++)
            buf[ind++] = buf[j];
    }
}

void Hokuyo::readData(std::vector<quint16>* const distances, qint32& timeStampScanner, int timeout)
{
    distances->clear();

    int bytesPerRay = 3;

    char buf[100];

    int ind = 0;

    timeStampScanner = readScannerTimeStamp(timeout);

    int bytes;

    for(;;)
    {
        bytes = laserReadline(&buf[ind], 100 - ind, timeout);

        if(bytes == 1)          // This is \n\n so we should be done
            return;

        if(!checkSum(&buf[ind], bytes))
            HOKUYO_EXCEPT(CorruptedDataException, "Checksum failed on data read.");

        bytes += ind - 2;

        // Read as many ranges as we can get
        for (int j = 0; j < bytes - (bytes % bytesPerRay); j+=bytesPerRay)
        {
            if(distances->size() < mMaximumRaysRead)
            {
                distances->push_back(((buf[j+0]-0x30) << 12) | ((buf[j+1]-0x30) << 6) | (buf[j+2]-0x30));
            }
            else
            {
                HOKUYO_EXCEPT(CorruptedDataException, "Got more readings than expected");
            }
        }

        // Shuffle remaining bytes to front of buffer to get them on the next loop
        ind = 0;
        for (int j = bytes - (bytes % bytesPerRay); j < bytes ; j++)
            buf[ind++] = buf[j];
    }
}

int Hokuyo::pollScan(LaserScan& scan, double min_ang, double max_ang, int cluster, int timeout)
{
    Q_ASSERT(portOpen());

    int status;

    // Always clear ranges/intensities so we can return easily in case of erro
    scan.ranges.clear();
    scan.intensities.clear();

    // clustering of 0 and 1 are actually the same
    if(cluster == 0)
        cluster = 1;

    int min_i = (int)(mAfrt + min_ang*mAres/(2.0*M_PI));
    int max_i = (int)(mAfrt + max_ang*mAres/(2.0*M_PI));

    char cmdbuf[mMaximumCommandLength];

    sprintf(cmdbuf,"GD%.4d%.4d%.2d", min_i, max_i, cluster);

    status = sendCmd(cmdbuf, timeout);

    scan.timeStampSystem = getSystemTime() + mLatencyOffset;

    if(status != 0)
        return status;

    /*
    // Populate configuration
    scan.config.angleStart  =  (min_i - mAfrt) * (2.0*M_PI)/(mAres);
    scan.config.angleStop  =  (max_i - mAfrt) * (2.0*M_PI)/(mAres);
    scan.config.angleIncrement =  cluster*(2.0*M_PI)/(mAres);
    scan.config.timeIncrement = (60.0)/(double)(mRate * mAres);
    scan.config.timeBetweenScans = 0.0;
    scan.config.rangeMin  =  mDmin / 1000.0;
    scan.config.rangeMax  =  mDmax / 1000.0;
    */

    readData(scan, false, timeout);

    long long inc = (long long)(min_i * (60.0)/(double)(mRate * mAres) * 1000000000);

    scan.timeStampSystem += inc;
    scan.timeStampScanner += inc;

    return 0;
}

int Hokuyo::laserOn()
{
    int res = sendCmd("BM",1000);
    if(res == 1)
        HOKUYO_EXCEPT(Exception, "Unable to control laser due to malfunction, error %d", res);
    return res;
}

int Hokuyo::laserOff()
{
    return sendCmd("QT",1000);
}

int Hokuyo::stopScanning()
{
    try {
        return laserOff();
    }
    catch (Exception &e)
    {
        // Ignore exception because we might have gotten part of a scan
        // instead of the expected response, which shows up as a bad checksum.
        laserFlush();
    }
    return laserOff(); // This one should work because the scan is stopped.
}

int Hokuyo::requestScans(bool intensity, double angleStart, double angleStop, int cluster, int skip, int count, int timeout)
{
    //! @todo check that values are within range?
    Q_ASSERT(portOpen());

    if(cluster == 0) cluster = 1;

    int rayStart = (int)(mAfrt + angleStart * mAres / (2.0*M_PI));
    int rayStop =  (int)(mAfrt + angleStop  * mAres / (2.0*M_PI));

    char cmdbuf[mMaximumCommandLength];

    const char intensityChar = intensity ? 'E' : 'D';

    sprintf(cmdbuf,"M%c%.4d%.4d%.2d%.1d%.2d", intensityChar, rayStart, rayStop, cluster, skip, count);
    qDebug() << "Hokuyo::requestScans(): requesting scans using" << cmdbuf << ":" << (intensity ? "with" : "without") << "intensity, rayStart" << rayStart << "rayStop" << rayStop << "cluster" << cluster << "skip" << skip << "count" << count << ":" << cmdbuf;

    return sendCmd(cmdbuf, timeout);
}

bool Hokuyo::isIntensitySupported()
{
    LaserScan scan;

    Q_ASSERT(portOpen());

    // Try an intensity command.
    try
    {
        requestScans(1, 0, 0, 0, 0, 1);
        serviceScan(scan, 1000);
        return true;
    }
    catch (Exception &e) {}

    // Try a non intensity command.
    try
    {
        requestScans(0, 0, 0, 0, 0, 1);
        serviceScan(scan, 1000);
        return false;
    }
    catch (Exception &e)
    {
        HOKUYO_EXCEPT(Exception, "Exception while trying to determine if intensity scans are supported.")
    }
}

int Hokuyo::serviceScan(LaserScan& scan, int timeout)
{
    Q_ASSERT(portOpen());

    // Always clear ranges/intensities so we can return easily in case of error
    scan.ranges.clear();
    scan.intensities.clear();

    char buf[100];

    bool intensity = false;
    int min_i;
    int max_i;
    int cluster;
    int skip;
    int left;

    char* ind;

    int status = -1;

    do {
        ind = laserReadlineAfter(buf, 100, "M",timeout);
        scan.timeStampSystem = getSystemTime() + mLatencyOffset/1000;

        if(ind[0] == 'D')
            intensity = false;
        else if(ind[0] == 'E')
            intensity = true;
        else
            continue;

        ind++;

        sscanf(ind, "%4d%4d%2d%1d%2d", &min_i, &max_i, &cluster, &skip, &left);
        laserReadline(buf,100,timeout);

        buf[4] = 0;

        if(!checkSum(buf, 4))
            HOKUYO_EXCEPT(CorruptedDataException, "Checksum failed on status code: %s", buf);

        sscanf(buf, "%2d", &status);

        if(status != 99)
            return status;

    } while(status != 99);

    /* we don't need the config in every scan!
    scan.config.angleStart  =  (min_i - mAfrt) * (2.0*M_PI)/(mAres);
    scan.config.angleStop  =  (max_i - mAfrt) * (2.0*M_PI)/(mAres);
    scan.config.angleIncrement =  cluster*(2.0*M_PI)/(mAres);
    scan.config.timeIncrement = (60.0)/(double)(mRate * mAres);
    scan.config.timeBetweenScans = (60.0 * (skip + 1))/((double)(mRate));
    scan.config.rangeMin  =  mDmin / 1000.0;
    scan.config.rangeMax  =  mDmax / 1000.0;*/

    readData(scan, intensity, timeout);

    const long long inc = (long long)(min_i * (60.0)/(double)(mRate * mAres) * 1000000);

    scan.timeStampSystem += inc;
    scan.timeStampScanner += inc;

    //  printf("Scan took %lli.\n", -scan.system_time_stamp + timeHelper() + offset_);

    return 0;
}


int Hokuyo::serviceScan(std::vector<quint16>* const distances, qint32& towScanBeginning, int timeout)
{
    Q_ASSERT(portOpen());

    // Always clear ranges/intensities so we can return easily in case of error
    distances->clear();

    char buf[100];

    bool intensity = false;
    int rayIndexFirst;
    int rayIndexStop;
    int cluster;
    int skip;
    int left;

    char* ind;

    int status = -1;

    do {
        ind = laserReadlineAfter(buf, 100, "M",timeout);
        towScanBeginning = GnssTime::currentTow() + mLatencyOffset/1000;

        if(ind[0] == 'D')
            intensity = false;
        else if(ind[0] == 'E')
            intensity = true;
        else
            continue;

        ind++;

        sscanf(ind, "%4d%4d%2d%1d%2d", &rayIndexFirst, &rayIndexStop, &cluster, &skip, &left);
        laserReadline(buf,100,timeout);

        buf[4] = 0;

        if(!checkSum(buf, 4))
            HOKUYO_EXCEPT(CorruptedDataException, "Checksum failed on status code: %s", buf);

        sscanf(buf, "%2d", &status);

        if(status != 99)
            return status;

    } while(status != 99);

    //readData(scan, intensity, timeout);
    qint32 timeStampScanner;
    readData(distances, timeStampScanner, timeout);

    // Move timestamp according to first ray scanned. Not sure whether this is correct, but since rayIndexFirst
    // is always 0 in our application, it doesn't matter for us.
    towScanBeginning += (long long)(rayIndexFirst * (60.0f)/(mRate * mAres) * 1000000);

    return 0;
}

void Hokuyo::slotProcessScans()
{
    qDebug() << GnssTime::currentTow() << "Hokuyo::slotProcessScans(): starting in process" << getpid() << "thread" << pthread_self();

    mState = State::Scanning;

    HokuyoConfig config;
    getConfig(config);

    // If the scanner's latency hasn't been determined by now, this is a great moment to do so:
    // When the GNSS receiver gets perfect reception for the first time, we're called to enable
    // scanning. This means the amount of data on the USB bus is now at its highest, so we should
    // determine latency in this state.
    if(!isLatencyDetermined())
    {
        qDebug() << "Hokuyo::slotProcessScans(): determining latency before first laser activation...";
        determineLatency(false, config.angleStart, config.angleStop);
    }

    qDebug() << "Hokuyo::slotProcessScans(): requesting scans...";
    requestScans(false, config.angleStart, config.angleStop);

    qDebug() << "Hokuyo::slotProcessScans(): done requesting scans.";
    do
    {
        mHeightOverGroundClockDivisor++;
        mHeightOverGroundClockDivisor %= (500 / 25); // Emit heightOverGround twice per second

        std::vector<quint16> * distances = new std::vector<quint16>;
        qint32 towScanBeginning;

        try
        {
            //QTime t;t.start();
            const int status = serviceScan(distances, towScanBeginning);

            //qDebug() << "Hokuyo::slotProcessScans(): serviceScan() took" << t.elapsed() << "ms for" << distances->size() <<"rays, scan started at time" << towScanBeginning;

            if(status != 0)
            {
                qDebug() << "Hokuyo::slotProcessScans(): Error getting scan:" << status;
                break;
            }
        }
        catch (CorruptedDataException &e)
        {
            qDebug() << "Hokuyo::slotProcessScans(): Skipping corrupted data";
            continue;
        }
        catch (Exception& e)
        {
            qDebug() << "Hokuyo::slotProcessScans(): Exception thrown while trying to get scan:" << e.what();
            close();
        }

        // Every full moon, emit the distance from vehicle center to the ground in meters (scanner to vehicle center is 3cm)
        if(mHeightOverGroundClockDivisor == 0 && distances->size() > 540) emit heightOverGround((*distances)[540]/1000.0f + 0.03f);

        const qint32 timeStampScanMiddle = towScanBeginning + 9;

        // Always write log data in binary format for later replay. Format is:
        // PackageLengthInBytes(quint16) TOW(qint32) StartIndex(quint16) N-DISTANCES(quint16)
        // StarIndex denotes the start of usable data (not 1s)

        // A usual dataset contains 200 1's at the beginning and 200 1's at the end.
        // We RLE-compress the leading 1s and drop the trailing 1s
        quint16 indexFirst = 0;
        while((*distances)[indexFirst] <= 20)
            indexFirst++;

        quint16 indexLast = distances->size()-1;
        while((*distances)[indexLast] <= 20)
            indexLast--;

        Q_ASSERT(indexFirst < indexLast);

        const quint32 numberOfDistanceBytesToWrite = sizeof(quint16) * ((indexLast - indexFirst) + 1);
        const char* distanceBytesToWrite = (const char*)(distances->data() + indexFirst);

        // Write the total amount of bytes of this scan into the stream
        const quint16 length = 5 // LASER
                + sizeof(quint16) // length at beginning
                + sizeof(qint32) // timeStampScanMiddle
                + sizeof(quint16) // indexFirst
                + numberOfDistanceBytesToWrite; // number of bytes for the distance-data

        QByteArray magic("LASER");

        mLogFile->write(magic.constData(), magic.size());
        mLogFile->write((const char*)&length, sizeof(length));
        mLogFile->write((const char*)&timeStampScanMiddle, sizeof(timeStampScanMiddle));
        mLogFile->write((const char*)&indexFirst, sizeof(indexFirst));

        QString debugString;
        for(int i=0;i<distances->size();i++)
            debugString.append(QString::number((*distances)[i] + ','));

        qDebug() << "Hokuyo::slotProcessScans(): incoming:" << debugString;

        qDebug() << "Hokuyo::slotProcessScans(): got" << distances->size() << "rays at" << distances->data() << ", indexFirst" << indexFirst << "indexLast" << indexLast << "writing" << numberOfDistanceBytesToWrite << "bytes starting at" << distanceBytesToWrite;
        qDebug() << "Hokuyo::slotProcessScans():"
                    << "index" << indexFirst-1 << ":" << (*distances)[indexFirst-1]
                    << "index" << indexFirst+0 << ":" << (*distances)[indexFirst+0]
                    << "index" << indexFirst+1 << ":" << (*distances)[indexFirst+1];
        qDebug() << "Hokuyo::slotProcessScans():"
                    << "index" << indexLast-1 << ":" << (*distances)[indexLast-1]
                    << "index" << indexLast+0 << ":" << (*distances)[indexLast+0]
                    << "index" << indexLast+1 << ":" << (*distances)[indexLast+1];

        // Instead of looping through the indices, lets write everything at once.
        mLogFile->write(distanceBytesToWrite, numberOfDistanceBytesToWrite);

        // With this call, we GIVE UP OWNERSHIP of the data. It might get deleted immediately!
        emit newScanData(timeStampScanMiddle, distances);
    } while (mState == State::Scanning);

    Q_ASSERT(mState == State::StopRequested);

    qDebug() << "Hokuyo::slotProcessScans(): stop requested, switching laser off...";

    try
    {
        stopScanning(); // This actually just calls laser Off internally.
    } catch (Exception &e)
    {
        qDebug() << "Hokuyo::slotProcessScans(): Exception thrown while trying to stop scan:" << e.what();
    }

    qDebug() << "Hokuyo::slotProcessScans(): switched laser off, setting state to stopped";
    mState = State::Stopped;

    qDebug() << "Hokuyo::slotProcessScans(): emiting finished().";
    emit finished();
    qDebug() << "Hokuyo::slotProcessScans(): done.";
}

void Hokuyo::slotStopScanning()
{
    mState = State::StopRequested;
}

void Hokuyo::queryVersionInformation()
{
    Q_ASSERT(portOpen());

    if(sendCmd("VV",1000) != 0)
        HOKUYO_EXCEPT(Exception, "Error requesting version information");

    char buf[100];
    mVendorName = laserReadlineAfter(buf, 100, "VEND:");
    mVendorName = mVendorName.left(mVendorName.length() - 3);

    mProductName = laserReadlineAfter(buf, 100, "PROD:");
    mProductName = mProductName.left(mProductName.length() - 3);

    mFirmwareVersion = laserReadlineAfter(buf, 100, "FIRM:");
    mFirmwareVersion = mFirmwareVersion.left(mFirmwareVersion.length() - 3);

    mProtocolVersion = laserReadlineAfter(buf, 100, "PROT:");
    mProtocolVersion = mProtocolVersion.left(mProtocolVersion.length() - 3);

    // This crazy naming scheme is for backward compatibility. Initially
    // the serial number always started with an H. Then it got changed to a
    // zero. For a while the driver was removing the leading zero in the
    // serial number. This is fine as long as it is indeed a zero in front.
    // The current behavior is backward compatible but will accomodate full
    // length serial numbers.
    mSerialNumber = laserReadlineAfter(buf, 100, "SERI:");
    mSerialNumber = mSerialNumber.left(mSerialNumber.length() - 3);
    if(mSerialNumber[0] == '0')
        mSerialNumber[0] = 'H';
    else if(mSerialNumber[0] != 'H')
        mSerialNumber = 'H' + mSerialNumber;

    qDebug() << "Hokuyo::queryVersionInformation(): using scanner vendor" << mVendorName << "product" << mProductName << "serialnumber" << mSerialNumber << "firmware" << mFirmwareVersion << "protocol" << mProtocolVersion;
}



QString Hokuyo::getID()
{
    Q_ASSERT(portOpen());

    return mSerialNumber;
}



QString Hokuyo::getFirmwareVersion()
{
    Q_ASSERT(portOpen());

    return mFirmwareVersion;
}

QString Hokuyo::getProtocolVersion()
{
    Q_ASSERT(portOpen());

    return mProtocolVersion;
}



QString Hokuyo::getVendorName()
{
    Q_ASSERT(portOpen());
    return mVendorName;
}



QString Hokuyo::getProductName()
{
    Q_ASSERT(portOpen());

    return mProductName;
}



QString Hokuyo::getStatus()
{
    Q_ASSERT(portOpen());

    if(sendCmd("II",1000) != 0)
        HOKUYO_EXCEPT(Exception, "Error requesting device information information");

    char buf[100];
    char* stat = laserReadlineAfter(buf, 100, "STAT:");

    QString statstr(stat);
    statstr = statstr.left(statstr.length() - 3);

    return statstr;
}

template <class C>
C median(std::vector<C> &v)
{
    std::vector<qint64>::iterator start  = v.begin();
    std::vector<qint64>::iterator end    = v.end();
    std::vector<qint64>::iterator median = start + (end - start) / 2;
    //std::vector<qint64>::iterator quarter1 = median - (end - start) / 4;
    //std::vector<qint64>::iterator quarter2 = median + (end - start) / 4;
    std::nth_element(start, median, end);
    //qint64 medianval = *median;
    //std::nth_element(start, quarter1, end);
    //qint64 quarter1val = *quarter1;
    //std::nth_element(start, quarter2, end);
    //qint64 quarter2val = *quarter2;
    return *median;
}

qint64 Hokuyo::getClockOffset(quint8 numberOfMeasurements, qint16 timeout)
{
    numberOfMeasurements = qBound(1, (int)numberOfMeasurements, 20);

    std::vector<qint64> clockOffsets(numberOfMeasurements);

    sendCmd("TM0", timeout);

    for (quint8 i = 0; i < numberOfMeasurements; i++)
    {
        const qint64 prestamp = getSystemTime();
        sendCmd("TM1", timeout);
        const qint64 hokuyostamp = readScannerTimeStamp();
        const qint64 poststamp = getSystemTime();
        clockOffsets[i] = hokuyostamp - (prestamp + poststamp) / 2;
        //printf("%lli %lli %lli", hokuyostamp, prestamp, poststamp);
    }

    sendCmd("TM2",timeout);

    return median(clockOffsets);
}

qint64 Hokuyo::getScannerStampToSystemStampOffset(bool intensity, double angleMin, double angleMax, quint16 clustering, quint16 skip, quint8 numberOfMeasurements, qint16 timeout)
{
    Q_ASSERT(portOpen());

    numberOfMeasurements = qBound(1, (int)numberOfMeasurements, 50);

    std::vector<qint64> offset(numberOfMeasurements);

    if(requestScans(intensity, angleMin, angleMax, clustering, skip, numberOfMeasurements, timeout) != 0)
    {
        HOKUYO_EXCEPT(Exception, "Error requesting scan while caliblating time.");
        return 1;
    }

    LaserScan scan;
    for(quint8 i = 0; i < numberOfMeasurements; i++)
    {
        serviceScan(scan, timeout);
        //printf("%lli %lli\n", scan.self_time_stamp, scan.system_time_stamp);
        offset[i] = scan.timeStampScanner - scan.timeStampSystem;
    }

    return median(offset);
}

void Hokuyo::determineLatency(bool intensity, double angleMin, double angleMax, quint16 clustering, quint16 skip, quint8 numberOfMeasurements, qint16 timeout)
{
    Q_ASSERT(portOpen());

    numberOfMeasurements = qBound(10, (int)numberOfMeasurements, 50);

    const qint64 start = getClockOffset(1, timeout);
    qint64 pre = 0;
    std::vector<qint64> samples(numberOfMeasurements);
    for (int i = 0; i < numberOfMeasurements; i++)
    {
        qint64 scan = getScannerStampToSystemStampOffset(intensity, angleMin, angleMax, clustering, skip, 1, timeout) - start;
        qint64 post = getClockOffset(1, timeout) - start;
        samples[i] = scan - (post+pre)/2;
        qDebug() << "Hokuyo::determineLatency(): scan" << scan << "pre" << pre << "post" << post << "sample" << samples[i];
        //printf("%lli %lli %lli %lli %lli\n", samples[i], post, pre, scan, pre - post);
        pre = post;
    }

    mLatencyOffset = median(samples);
    mIsLatencyDetermined = true;
    qDebug() << "Hokuyo::determineLatency(): latency determined to be" << mLatencyOffset << "us";
}

// Returns microseconds since unix epoch
qint64 Hokuyo::getSystemTime()
{
    struct timeval timeofday;
    gettimeofday(&timeofday,NULL);
    const qint64 systemTimeUsec = (qint64)(timeofday.tv_sec) * 1000000 + (qint64)(timeofday.tv_usec);
//    qDebug() << "Hokuyo::getSystemTime(): system time is" << systemTimeUsec << "usec or" << systemTimeUsec / 1000 << "msec or" << systemTimeUsec/1000000 << "seconds";
    return systemTimeUsec;
}
