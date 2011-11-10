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
    qDebug() << "VoxelManager::VoxelManager(): resolution is" << resX << resY << resZ << "and needs" << (mResX * mResY * mResZ) / 8 / 1048576 << "MB memory.";
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
    Q_ASSERT(mData = 0);

    // Check that position is within our bounds
    Q_ASSERT(mBBoxMin.x() <= position.x() && mBBoxMin.y() <= position.y() && mBBoxMin.z() <= position.z());
    Q_ASSERT(mBBoxMax.x() >= position.x() && mBBoxMax.y() >= position.y() && mBBoxMin.z() >= position.z());

    const float posX = position.x() / (mBBoxMax.x() - mBBoxMin.x()) * mResX;
    const float posY = position.y() / (mBBoxMax.y() - mBBoxMin.y()) * mResY;
    const float posZ = position.z() / (mBBoxMax.z() - mBBoxMin.z()) * mResZ;

    // Should never assert because posiioin was within bounds
    Q_ASSERT(posX < mResX);
    Q_ASSERT(posY < mResY);
    Q_ASSERT(posZ < mResZ);

    const quint32 offset = (posX*posY*posZ) + (posX*posY) + posX;
    Q_ASSERT(offset < mResX * mResY * mResZ);

    quint8 bitShift = 0;
    if(posX - (int)posX > 0.5) bitShift += 1;
    if(posY - (int)posY > 0.5) bitShift += 2;
    if(posZ - (int)posZ > 0.5) bitShift += 4;

    bitMask = 1 << bitShift;

    return mData + offset;
}

bool VoxelManager::isOccupied(const QVector3D& position)
{
    quint8 bitMask;
    return(*getParentVoxelAndBitMask(position, bitMask) & bitMask != 0);
}

void VoxelManager::setVoxelValue(const QVector3D& position, const bool& value)
{
    quint8 bitMask;
    quint8* parentVoxel = getParentVoxelAndBitMask(position, bitMask);

    if(value)
        *parentVoxel = *parentVoxel | bitMask;
    else
        *parentVoxel = *parentVoxel & !bitMask;
}

void VoxelManager::slotSetScanVolume(const QVector3D& bBoxMin, const QVector3D& bBoxMax)
{
    mBBoxMin = bBoxMin;
    mBBoxMax = bBoxMax;
    initializeData();
}

void VoxelManager::initializeData()
{
    // Reserve and initialize memory for all our parentvoxels (=bytes). Each Parentvoxel
    // contains 8 subvoxels (leafs), which are the 8 bits of its byte.
    const quint64 dataSize = mResX * mResY * mResZ / 8;

//  CPU version
//    if(mData) delete[] mData;
//    mData = new quint8[dataSize];
    if(mData) cudaFreeHost(mData);
    if(cudaHostAlloc(&mData, dataSize, cudaHostAllocMapped | cudaHostAllocWriteCombined) != cudaSuccess) qFatal("VoxelManager::initializeData(): couldn't allocate %d bytes of pinned memory, exiting.", dataSize);
    memset(mData, 0, dataSize);
}
