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

class LogFile : public QObject
{
    Q_OBJECT

    // to lock buffers while we swap them before writing
    QMutex mMutex;

    QFuture<void> mFuture;
    QFile* mLogFile;
    QByteArray *mBuffer, *mBufferCopy;
    QTime mTimeOfLastWrite;
    QTimer mTimer;
    quint64 mBytesWritten;

    void flush();
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
    
signals:
    
public slots:
    
};

#endif
