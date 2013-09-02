#ifndef PLYMANAGER_H
#define PLYMANAGER_H

#include <QFile>
#include <QVector>
#include <QVector3D>
#include <QVector4D>
#include <QString>
#include <QTextStream>
#include <QDateTime>
#include <QDebug>
#include <QMessageBox>
#include <QProgressDialog>

class PointCloudReader
{
public:
    virtual bool open() = 0;
    virtual qint64 getNumberOfPoints() = 0;
    virtual qint64 readPoints(float* target, qint64 maxNumberOfPoints, QWidget* widget = nullptr) = 0;
    virtual void close() = 0;
};

class PointCloudWriter
{
public:
    virtual bool open() = 0;
    virtual bool writePoints(float* source, qint64 numberOfPoints, QWidget* widget = nullptr) = 0;
    virtual void close() = 0;
};

enum DataFormatPly {DataFormatAscii, DataFormatBinaryLittleEndian, DataFormatBinaryBigEndian};
class PointCloudReaderPly : public PointCloudReader
{
    qint64 mNumberOfPoints;
    QFile* mFile;
    DataFormatPly mDataFormat;

public:
    PointCloudReaderPly(const QString& fileName);
    ~PointCloudReaderPly();

    bool open();
    qint64 getNumberOfPoints();
    qint64 readPoints(float* target, qint64 maxNumberOfPoints, QWidget* widget = nullptr);
    void close();
};

class PointCloudWriterPly : public PointCloudWriter
{
    qint64 mTotalNumberOfPointsWrittenToThisFile;
    DataFormatPly mDataFormat;
    QFile* mFile;

public:
    PointCloudWriterPly(const QString& fileName);
    ~PointCloudWriterPly();

    bool open();
    bool writePoints(float* source, qint64 numberOfPoints, QWidget* widget = nullptr);
    void close();
    bool setDataFormat(const DataFormatPly dataFormat);

};

#endif
