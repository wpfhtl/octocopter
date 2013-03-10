#include "filetowrite.h"

FileToWrite::FileToWrite(const quint32 interval, const quint64 numBytes) : QObject()
{
    mMaxWriteTime = 0;

    mNumBytes = numBytes;

    data = new char[numBytes];
    memset(data, 0, numBytes);

    mFile = new QFile(QString("%1 bytes every %2 ms").arg(mNumBytes).arg(interval));
    if(!mFile->open(QIODevice::WriteOnly))
        qFatal("Couldn't open file for writing");

    connect(&mTimer, SIGNAL(timeout()), SLOT(slotWrite()));
    mTimer.start(interval);

    qDebug() << "FileToWrite::FileToWrite(): file" << mFile->fileName() << "set up.";
}

FileToWrite::~FileToWrite()
{
    qDebug() << "FileToWrite::~FileToWrite(): max write time for file" << mFile->fileName() << "was" << mMaxWriteTime << "ms.";
    mTimer.stop();
    mFile->close();
    mFile->deleteLater();
}

void FileToWrite::slotWrite()
{
    QTime now;
    now.start();
    mFile->write(data, mNumBytes);
    const quint32 elapsed = now.elapsed();
    mMaxWriteTime = std::max(elapsed, mMaxWriteTime);
    qDebug() << mFile->fileName() << "took:" << elapsed << "ms.";
}
