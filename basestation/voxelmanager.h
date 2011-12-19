#ifndef VOXELMANAGER_H
#define VOXELMANAGER_H

#include <QObject>
#include <QDebug>
#include <QVector3D>

#include <math.h>
#include <cuda_runtime.h>

// Used to find holes in my environment. Stores voxels with 1 bit values, either occupied or empty.
// X is stored as columns, Y are the rows and Z are the layers/planes.

class VoxelManager : public QObject
{
    Q_OBJECT
private:
    quint8* mData;
    quint16 mResX, mResY, mResZ;
    QVector3D mBBoxMin, mBBoxMax;

    void initializeData();
    quint8* getParentVoxelAndBitMask(const QVector3D& position, quint8& bitMask);

public:
    VoxelManager(
        const quint16& resX,
        const quint16& resY,
        const quint16& resZ
        );

    ~VoxelManager();

    void setVolumeDataBasePointer(quint8* data) {mData = data;}
    quint8** getVolumeDataBasePointer() {return &mData;}

    bool isOccupied(const QVector3D& position);
    void setVoxelValue(const QVector3D& position, const bool& value);

    quint64 getVoxelCount() const {return mResX * mResY * mResZ;}
    quint64 getVolumeDataSize() const {return mResX * mResY * mResZ / 8;}
    quint64 getGroundPlanePixelCount() const {return mResX * mResZ;}

//    QVector3D getPhysicalSizeMin() const {return mBBoxMin;}
//    QVector3D getPhysicalSizeMax() const {return mBBoxMax;}
    void slotSetScanVolume(const QVector3D& bBoxMin, const QVector3D& bBoxMax);

signals:

public slots:

};

#endif // VOXELMANAGER_H
