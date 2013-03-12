#include "filetowrite.h"

FileToWrite::FileToWrite(const quint32 interval, const quint64 numBytes) : QObject()
{
    mMaxWriteTime = 0;

    mNumBytes = numBytes;

    data = new char[numBytes];
    memset(data, 0, numBytes);

    mLogFile = new LogFile(QString("%1 bytes every %2 ms").arg(mNumBytes).arg(interval), LogFile::Encoding::Text);

    connect(&mTimer, SIGNAL(timeout()), SLOT(slotWrite()));
    mTimer.start(interval);

    qDebug() << "FileToWrite::FileToWrite(): file" << mLogFile->fileName() << "set up.";
}

FileToWrite::~FileToWrite()
{
    qDebug() << "FileToWrite::~FileToWrite(): max write time for file" << mLogFile->fileName() << "was" << mMaxWriteTime << "ms.";
    mTimer.stop();
    delete mLogFile;
}

void FileToWrite::slotWrite()
{
    QTime now;
    now.start();
    mLogFile->write(data, mNumBytes);
    const quint32 elapsed = now.elapsed();
    mMaxWriteTime = std::max(elapsed, mMaxWriteTime);
    if(elapsed > 0)
        qDebug() << mLogFile->fileName() << "took:" << elapsed << "ms.";
}
