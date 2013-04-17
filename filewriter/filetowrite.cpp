#include "filetowrite.h"
#include <sys/time.h>

FileToWrite::FileToWrite(const quint32 interval, const quint64 numBytes) : QObject()
{
    mMaxWriteTime = 0;

    mNumBytes = numBytes;

    data = new char[numBytes];
    memset(data, 0, numBytes);

    mLogFile = new LogFile(QString("%1 bytes every %2 ms.tst").arg(mNumBytes).arg(interval), LogFile::Encoding::Text);

    connect(&mTimer, SIGNAL(timeout()), SLOT(slotWrite()));
    mTimer.start(interval);

    qDebug() << "FileToWrite::FileToWrite(): file" << mLogFile->fileName() << "set up.";
}

FileToWrite::~FileToWrite()
{
    qDebug() << "FileToWrite::~FileToWrite(): wrote" << mLogFile->bytesWritten() << "bytes to" << mLogFile->fileName() << "max write time was" << mMaxWriteTime << "usec.";
    mTimer.stop();
    delete mLogFile;
}

void FileToWrite::slotWrite()
{
    struct timeval time;

    gettimeofday(&time, NULL);
    const double tStart = time.tv_sec+(time.tv_usec/1000000.0);

    mLogFile->write(data, mNumBytes);

    gettimeofday(&time, NULL);
    const double tStop = time.tv_sec+(time.tv_usec/1000000.0);

    const quint32 elapsed = (tStop-tStart)*1000000.0;

    mMaxWriteTime = std::max(elapsed, mMaxWriteTime);
    //if(elapsed > 0) qDebug() << mLogFile->fileName() << "took:" << elapsed << "ms.";
}
