#include "logfile.h"

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

    mBufferCopy = new QByteArray();
    mBufferCopy->reserve(mBuffer->capacity());

    mTimer.setInterval(3000);
    connect(&mTimer, SIGNAL(timeout()), SLOT(slotCheckFlush()));

    mTimeOfLastWrite = QTime::currentTime();
}

LogFile::~LogFile()
{
    if(mFuture.isRunning())
        mFuture.waitForFinished();

    flush();

    if(mLogFile->size() != mBytesWritten)
        qDebug() << "LogFile::~LogFile(): WARNING:" << mBytesWritten << "were written into" << mLogFile->fileName() << "- but it contains" << mLogFile->size() << "bytes!";

    mLogFile->close();
    mLogFile->deleteLater();

    delete mBuffer;
    delete mBufferCopy;
}

void LogFile::write(const char* data, const quint32 length)
{
    QMutexLocker locker(&mMutex);
    mBuffer->append(data, length);
    slotCheckFlush();
    mBytesWritten += length;
}

void LogFile::write(const QByteArray* const data)
{
    QMutexLocker locker(&mMutex);
    mBuffer->append(data->constData(), data->size());
    slotCheckFlush();
    mBytesWritten += data->size();
}

void LogFile::write(const QByteArray& data)
{
    QMutexLocker locker(&mMutex);
    mBuffer->append(data.constData(), data.size());
    slotCheckFlush();
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
            // As QDebug() is re-routed to a LogFile, this can cause and endless loop!
            qDebug() << "LogFile::slotCheckFlush(): couldn't flush" << mBuffer->size() << "bytes into" << mLogFile->fileName() << "because flush() is still running!";
        }
    }
    else
    {
        mTimer.start();
    }
}

void LogFile::flush()
{
    // Lock mutex and make a private copy of the buffer for flushing...
    mMutex.lock();
    mBufferCopy->append(mBuffer->constData(), mBuffer->size());
    mTimeOfLastWrite = QTime::currentTime();
    mBuffer->clear();

    // Unlock mutex, so, callers can write() again.
    mMutex.unlock();

    // Flush
    mLogFile->write(mBufferCopy->constData(), mBufferCopy->size());
    mLogFile->flush();
    mBufferCopy->clear();
}
