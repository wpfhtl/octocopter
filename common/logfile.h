#ifndef LOGFILE_H
#define LOGFILE_H

#include <QObject>
#include <QString>
#include <QFuture>
#include <QMutex>
#include <QTimer>
#include <QTime>
#include <QFile>
#include <QtCore>

/*
 * This class was written to write logfiles to the SDHC card. We first used plain QFile::write(),
 * but that hangs when linux decides to flush to disk. This is inacceptable, as the hang can last
 * up to 3 seconds - and we're trying to control a helicopter...
 *
 * So, this class accepts soem bytes to be written to file and copies them into its own buffer.
 * After either some time has passed OR the buffer is almost full, it will copy the buffer and
 * write it to disk in another thread.
 */

class LogFile : public QObject
{
    Q_OBJECT

    // to lock buffers while we swap them before writing
    QMutex mMutex;

    QFuture<void> mFuture;
    QFile* mLogFile;
    QByteArray *mBuffer, *mBufferBeingFlushed;
    QTime mTimeOfLastWrite;
    QTimer mTimer;
    quint64 mBytesWritten;

    void flush();

private slots:
    void slotCheckFlush();

public:

    enum class Encoding
    {
        Text,
        Binary
    };

    LogFile(const QString& fileName, const Encoding encoding = Encoding::Text, QObject *parent = 0);
    ~LogFile();

    QString fileName() const { return mLogFile->fileName();}
    quint64 bytesWritten() const {return mBytesWritten; }

    void write(const char* data, const quint32 length);
    void write(const QByteArray* const data);
    void write(const QByteArray& data);
};

#endif
