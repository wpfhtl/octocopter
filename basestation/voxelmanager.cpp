#include "voxelmanager.h"

VoxelManager::VoxelManager(
    const quint16& resX,
    const quint16& resY,
    const quint16& resZ) :
    QObject(),
    mData(0),
    mResX(resX),
    mResY(resY),
    mResZ(resZ)
{
    // WARNING: resX, Y and Z are tested only for powers-of-two!
    qDebug() << "VoxelManager::VoxelManager(): resolution is" << mResX << "x" << mResY << "x" << mResZ << "so volume data needs" << ((mResX * mResY * mResZ) / 8) / 1048576 << "mb memory.";
}

VoxelManager::~VoxelManager()
{
//    if(mData) delete[] mData;
    if(mData) cudaFreeHost(mData);
}

// A Parentvoxel consists of one byte, making up 8 childvoxels with each of its bits.
// This function will return the parentVoxel pointer and set the bitmask if @position was valid.
quint8* VoxelManager::getParentVoxelAndBitMask(const QVector3D& position, quint8& bitMask)
{
    // Check that we are initialized
    Q_ASSERT(mData != 0);

    // Check that position is within our bounds
    if(mBBoxMin.x() >= position.x() || mBBoxMin.y() >= position.y() || mBBoxMin.z() >= position.z())
    {
        qDebug() << "VoxelManager::getParentVoxelAndBitMask(): point" << position << "is too small for bounding box from" << mBBoxMin << "to" << mBBoxMax;
        bitMask = 0;
        return 0;
    }
    else if(mBBoxMax.x() <= position.x() || mBBoxMax.y() <= position.y() || mBBoxMax.z() <= position.z())
    {
        qDebug() << "VoxelManager::getParentVoxelAndBitMask(): point" << position << "is too large for bounding box from" << mBBoxMin << "to" << mBBoxMax;
        bitMask = 0;
        return 0;
    }

    const float posX = (position.x() - mBBoxMin.x()) / (mBBoxMax.x() - mBBoxMin.x()) * mResX;
    const float posY = (position.y() - mBBoxMin.y()) / (mBBoxMax.y() - mBBoxMin.y()) * mResY;
    const float posZ = (position.z() - mBBoxMin.z()) / (mBBoxMax.z() - mBBoxMin.z()) * mResZ;

    // Should never assert because position was within bounds
    if(posX > mResX) qDebug() << "VoxelManager::getParentVoxelAndBitMask(): point" << position << "in grid from" << mBBoxMin << "to" << mBBoxMax << "leads to grid posX" << posX << "of" << mResX;
    if(posY > mResY) qDebug() << "VoxelManager::getParentVoxelAndBitMask(): point" << position << "in grid from" << mBBoxMin << "to" << mBBoxMax << "leads to grid posY" << posY << "of" << mResY;
    if(posZ > mResZ) qDebug() << "VoxelManager::getParentVoxelAndBitMask(): point" << position << "in grid from" << mBBoxMin << "to" << mBBoxMax << "leads to grid posZ" << posZ << "of" << mResZ;

    const qint32 offset = (posX*posY*posZ) + (posY*posX) + posY;
    Q_ASSERT(offset < mResX * mResY * mResZ);

    quint8 bitShift = 0;
    if(posY - (int)posY > 0.5) bitShift += 1;
    if(posX - (int)posX > 0.5) bitShift += 2;
    if(posZ - (int)posZ > 0.5) bitShift += 4;

    bitMask = 1 << bitShift;

    return mData + getVolumeDataSize() - offset;

    /* XYZ version, changed to align data to fall of gravity (-Y)
    const qint32 offset = (posX*posY*posZ) + (posX*posY) + posX;
    Q_ASSERT(offset < mResX * mResY * mResZ);

    quint8 bitShift = 0;
    if(posX - (int)posX > 0.5) bitShift += 1;
    if(posY - (int)posY > 0.5) bitShift += 2;
    if(posZ - (int)posZ > 0.5) bitShift += 4;

    bitMask = 1 << bitShift;

    return mData + offset;*/
}

bool VoxelManager::isOccupied(const QVector3D& position)
{
    // If position is not in our data, 0 will be returned. That's great failing :)
    quint8 bitMask;
    return(*getParentVoxelAndBitMask(position, bitMask) & bitMask != 0);
}

bool VoxelManager::setVoxelValue(const QVector3D& position, const bool& value)
{
    quint8 bitMask;
    quint8* parentVoxel = getParentVoxelAndBitMask(position, bitMask);

    if(!parentVoxel)
    {
        qDebug() << "VoxelManager::setVoxelValue(): cannot set voxel value at" << position << "as my grid spans from" << mBBoxMin << "to" << mBBoxMax;
        return false;
    }

    if(value)
        *parentVoxel = *parentVoxel | bitMask;
    else
        *parentVoxel = *parentVoxel & !bitMask;

    return true;
}

void VoxelManager::slotSetScanVolume(const QVector3D& bBoxMin, const QVector3D& bBoxMax)
{
    mBBoxMin = bBoxMin;
    mBBoxMax = bBoxMax;
    initializeData();
}

quint64 VoxelManager::getTotalOccupancy()
{
    quint64 sum = 0;

    for(qint64 i=0;i<getVolumeDataSize();i++)
    {
        sum += mData[i];
    }

    return sum;
}

void VoxelManager::initializeData()
{
    qDebug() << "VoxelManager::initializeData(): clearing occupancy grid...";

    // Initialization may happen at any time and is easy to do, because the resolutions
    // for X, Y and Z remain constant, just the bounding boxes can change. But those
    // don't influence our data structure, so we just memset our buffer and are done.

    if(mData) memset(mData, 0, getVolumeDataSize());
}
