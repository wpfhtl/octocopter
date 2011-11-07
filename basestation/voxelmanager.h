#ifndef VOXELMANAGER_H
#define VOXELMANAGER_H

#include <QObject>
#include <QVector3D>

// Used to find holes in my environment. Stores voxels with 1 bit values, either occupied or empty.
// X is stored as columns, Y are the rows and Z are the layers/planes.

class VoxelManager : public QObject
{
    Q_OBJECT
private:
    quint8* mData;
    quint16 mResX, mResY, mResZ;
    QVector3D mPhysicalSize;

public:
    VoxelManager(const QVector3D& physicalSize, const quint16& resX, const quint16& resY, const quint16& resZ, QObject *parent = 0);
    QVector3D getPhysicalSize() const {return mPhysicalSize;}

signals:

public slots:

};

#endif // VOXELMANAGER_H
