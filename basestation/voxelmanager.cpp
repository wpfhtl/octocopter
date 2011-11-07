#include "voxelmanager.h"

VoxelManager::VoxelManager(const QVector3D& physicalSize, const quint16& resX, const quint16& resY, const quint16& resZ, QObject *parent = 0) : QObject(parent)
{
    mPhysicalSize = physicalSize;

    mResX = resX;
    mResY = resY;
    mResZ = resZ;

    // Reserve memory for all our parentvoxels (=bytes)
    mData = new quint8[(resX * resY * resZ) / 8];
}

VoxelManager::~VoxelManager()
{
    delete[] mData;
}

// A Parentvoxel consists of one byte, making up 8 childvoxels with each of its bits.
quint8* VoxelManager::getParentVoxelAndBit(const QVector3D& position, quint8& bit)
{
    const float posX = position.x() / mPhysicalSize.x() * mResX;
    const float posY = position.y() / mPhysicalSize.y() * mResY;
    const float posZ = position.z() / mPhysicalSize.z() * mResZ;

    Q_ASSERT(posX < mResX);
    Q_ASSERT(posY < mResY);
    Q_ASSERT(posZ < mResZ);

    const quint32 offset = (posX*posY*posZ) + (posX*posY + posX);
    Q_ASSERT(offset < mResX * mResY * mResZ);

    if(posX - (int)posX > 0.5) 0;
    if(posY - (int)posY > 0.5) 0;
    if(posZ - (int)posZ > 0.5) 0;

    return mData + offset;
}

bool VoxelManager::isOccupied(const QVector3D& position)
{
    quint8* parentVoxel = getParentVoxel(position);
}

void VoxelManager::setVoxelValue(const QVector3D& position, const bool& value)
{
    //return *(mData + getOffsetForField(sizeX, sizeY, sizeZ));
}
