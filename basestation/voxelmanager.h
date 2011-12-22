#ifndef VOXELMANAGER_H
#define VOXELMANAGER_H

#include <QObject>
#include <QDebug>
#include <QVector3D>

#include <math.h>
#include <cuda_runtime.h>

// This class managed a 3D occupancy grid which is fed by incoming points from the rover. When new
// waypoints are to be generated, we "fall through" this grid from top to bottom. If we make it
// through to the bottom, we need to inspect this column further. So far the basic idea...
//
// Stores voxels with 1 bit values, either occupied or empty.
// OLD:
// X is stored as columns, Y are the rows and Z are the layers/planes from back to front
//
// NEW:
// Data is arranged so that falling down through the grid (-Y) means running through memory as
// linearly as possible. This should yield better memory locality in our algorithm on the GPU.
// Y decreases, then X decreases, then Z decreases.

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

    quint64 getTotalOccupancy();

    bool isOccupied(const QVector3D& position);
    bool setVoxelValue(const QVector3D& position, const bool& value);

    quint16 getResolutionX() {return mResX;} const
    quint16 getResolutionY() {return mResY;} const
    quint16 getResolutionZ() {return mResZ;} const

    quint64 getVoxelCount() const {return mResX * mResY * mResZ;}
    quint64 getVolumeDataSize() const {return mResX * mResY * mResZ / 8;}
    quint64 getGroundPlanePixelCount() const {return mResX * mResZ;}

//    QVector3D getPhysicalSizeMin() const {return mBBoxMin;}
//    QVector3D getPhysicalSizeMax() const {return mBBoxMax;}
    void slotSetScanVolume(const QVector3D& bBoxMin, const QVector3D& bBoxMax);

};

#endif // VOXELMANAGER_H
