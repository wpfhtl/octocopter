#ifndef POINTCLOUDCUDA_H
#define POINTCLOUDCUDA_H

#include <QVector>
#include <QVector3D>
#include <QVector4D>
#include <QTime>

#include "common.h"
#include "cuda.h"
#include "vector_functions.h"
#include "pointcloud.h"
#include "pointcloudcuda.cuh"

// PointCloud is a container for points, storing them in a VBO on the GPU. Furthermore, a minimum distance
// between points can be specified. The VBO for storage is allocated once and cannot grow during the object's
// lifetime. It is filled using insertPoints(), which will simply insert the given points to the VBO and mark
// them as queued using indexes into this buffer.
//
// Once in a while, all queued points are intergrated into the already-reduced points located in other indexes
// in this buffer. During this process, the new points are removed if they are closer than minimumPointDistance
// apart from each other or already-reduced points. This way, we try to keep a sparse poitcloud without too
// much redundancy. The reduction is done on the GPU, the points are NOT kept on the host.
//
// PointClouds are rendered using OpenGL 4 core profile by giving out the VBO used for storage to a renderer.
//
// Deletion of single points is currently not implemented but could be done.

class PointCloudCuda : public PointCloud
{
    Q_OBJECT



public:
    PointCloudCuda(const QVector3D &min, const QVector3D &max, const quint32 maximumElementCount = 8 * 1024 * 1024); // 8 million points capacity
    ~PointCloudCuda();

    void updateVbo();

    // No two points closer than @distance will be inserted into this PointCloud
    void setMinimumPointDistance(const float &distance) { mParameters.minimumDistance = distance; }
    float getMinimumPointDistance() const { return mParameters.minimumDistance; }

    quint32 getNumberOfPoints(void) const { return mParameters.elementCount + mParameters.elementQueueCount; }

    void setGridSize(const quint16 x, const quint16 y, const quint16 z)
    {
        mParameters.gridSize.x = x;
        mParameters.gridSize.y = y;
        mParameters.gridSize.z = z;
    }

    const QVector<VboInfo>& getVboInfo() const { return mVboInfo; }

    quint32 getCapacity(void) const { return mParameters.capacity; }

    void setColor(const QColor& c) {mVboInfo[0].color = c;}

    bool importFromPly(const QString& fileName, QWidget* widget = 0);
    bool exportToPly(const QString& fileName, QWidget* widget = 0) const;

private:
    PointCloudParameters mParameters;

    // The VBO pointing to the points. To be passed to GlWidget for rendering. We currently use just one VBO.
    QVector<VboInfo> mVboInfo;

    bool mIsInitialized;

    float* mNewPoints;

    quint32 createVbo(quint32 size);

    float* mDevicePointSortedPos;

    unsigned int*  mDeviceMapGridCell;      // grid hash value for each particle
    unsigned int*  mDeviceMapPointIndex;    // particle index for each particle
    unsigned int*  mDeviceCellStart;        // index of start of each cell in sorted list
    unsigned int*  mDeviceCellStopp;          // index of end of cell

    struct cudaGraphicsResource *mCudaVboResource; // handles OpenGL-CUDA exchange

    // reduce the points if necessary. Use any method. A wrapper-method for all my recent attempts at reduction....
    bool reduce();

    // reduces the queued points against themselves, then merges all points and reduces again. Thin wrapper around reducePoints()
    quint32 reduceAllPointsUsingCollisions();
    quint32 reduceUsingSnapToGrid();

    quint32 reduceUsingCellMean();

    // reduces the given points against themselves
    quint32 reducePointRangeUsingCollisions(float *devicePoints, const quint32 numElements, const bool createBoundingBox);

    void freeResources();
    void setNullPointers();

public slots:

    // Tell the cloud to insert the given points. As you can see, the cloud may not change or even own the points.
    bool slotInsertPoints(const QVector<QVector3D>* const pointList);
    bool slotInsertPoints(const QVector<QVector4D>* const pointList);
    bool slotInsertPoints3(const float* const pointList, const quint32 numPoints);
    bool slotInsertPoints4(const float* const pointList, const quint32 numPoints);

    // Insert points from a range of a VBO
    bool slotInsertPoints(const VboInfo* const vboInfo, const quint32& firstPoint, const quint32& numPoints);

public slots:
    void slotInitialize();

    // Clears the datastructure, but does not destruct it. Points can still be inserted afterwards.
    void slotReset();
};

#endif
