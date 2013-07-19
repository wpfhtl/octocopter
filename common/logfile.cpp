#include "logfile.h"
#include <QtConcurrent/QtConcurrentRun>

LogFile::LogFile(const QString& fileName, const Encoding encoding, QObject *parent) : QObject(parent)
{
    mLogFile = new QFile(fileName);

    if(encoding == Encoding::Text)
    {
        if(!mLogFile->open(QIODevice::WriteOnly | QIODevice::Text))
            qFatal("LogFile::LogFile(): Couldn't open logfile %s for writing, exiting.", qPrintable(mLogFile->fileName()));
    }
    else if(encoding == Encoding::Binary)
    {
        if(!mLogFile->open(QIODevice::WriteOnly))
            qFatal("LogFile::LogFile(): Couldn't open logfile %s for writing, exiting.", qPrintable(mLogFile->fileName()));
    }

    mBytesWritten = 0;

    mBuffer = new QByteArray();
    mBuffer->reserve(1 * 1024 * 1024);

    mBufferBeingFlushed = new QByteArray();
    mBufferBeingFlushed->reserve(mBuffer->capacity());

    connect(&mTimer, &QTimer::timeout, this, &LogFile::slotCheckFlush);
    mTimer.start(3000);

    mTimeOfLastWrite = QTime::currentTime();
}

LogFile::~LogFile()
{
    if(mFuture.isRunning())
        mFuture.waitForFinished();

    flush();

    qDebug() << "LogFile::~LogFile():" << mBytesWritten << "bytes were written into" << mLogFile->fileName() << "- flushing completed.";

    if(mLogFile->size() != mBytesWritten)
        qDebug() << "LogFile::~LogFile(): WARNING:" << mBytesWritten << "were written into" << mLogFile->fileName() << "- but it contains" << mLogFile->size() << "bytes!";

    mLogFile->close();
    mLogFile->deleteLater();

    delete mBuffer;
    delete mBufferBeingFlushed;
}

void LogFile::write(const char* data, const quint32 length)
{
    QMutexLocker locker(&mMutex);
    mBuffer->append(data, length);
    mBytesWritten += length;
}

void LogFile::write(const QByteArray* const data)
{
    QMutexLocker locker(&mMutex);
    mBuffer->append(data->constData(), data->size());
    mBytesWritten += data->size();
}

void LogFile::write(const QByteArray& data)
{
    QMutexLocker locker(&mMutex);
    mBuffer->append(data.constData(), data.size());
    mBytesWritten += data.size();
}

void LogFile::slotCheckFlush()
{
    // This is called after every buffer appending. There's potentially multiple instances of LogFile.
    if(mTimeOfLastWrite.msecsTo(QTime::currentTime()) >= mTimer.interval() || mBuffer->size() > mBuffer->capacity() * 0.9f)
    {
        // flush in a separate thread!
        if(!mFuture.isRunning())
        {
            mFuture = QtConcurrent::run(this, &LogFile::flush);
        }
        else
        {
            // We would like to flush, but the last flush is still ongoing!
            // As QDebug() is re-routed to a LogFile, this can cause and endless loop!
            qDebug() << "LogFile::slotCheckFlush(): couldn't flush" << mBuffer->size() << "bytes into" << mLogFile->fileName() << "because flush() is still running!";
        }
    }
}

void LogFile::flush()
{
    QByteArray* temp;

    QTime t; t.start();
    qDebug() << "LogFile::flush(): flushing" << mLogFile->fileName() << "...";

    // Lock mutex and swap buffers before for flushing...
    mMutex.lock();

    temp = mBufferBeingFlushed;
    mBufferBeingFlushed = mBuffer;
    mBuffer = temp;
    mTimeOfLastWrite = QTime::currentTime();

    // Unlock mutex, so, callers can write() again.
    mMutex.unlock();

    // Flush
    mLogFile->write(mBufferBeingFlushed->constData(), mBufferBeingFlushed->size());
    mLogFile->flush();
    mBufferBeingFlushed->clear();

    qDebug() << "LogFile::flush(): done flushing" << mLogFile->fileName() << "after" << t.elapsed() << "ms";
}
