#ifndef FILETOWRITE_H
#define FILETOWRITE_H

#include <QTime>
#include <QTimer>
#include <QFile>
#include <QDebug>
#include <QDataStream>

class FileToWrite : public QObject
{
    Q_OBJECT

public:
    FileToWrite(const quint32 interval, const quint64 numBytes);
    ~FileToWrite(void);

public slots:

private:
    char* data;
    quint32 mNumBytes;
    quint32 mMaxWriteTime;
    QTimer mTimer;
    QFile* mFile;

private slots:
    void slotWrite();

};

#endif
