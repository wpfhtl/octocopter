#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <QVector>
#include <QVector3D>
#include <QVector4D>
#include <QTime>

#include "common.h"
#include "cuda.h"
#include "vector_functions.h"
#include "pointcloud.cuh"

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

class PointCloud : public QObject
{
    Q_OBJECT

private:
    PointCloudParameters mParameters;

    // The VBO pointing to the points. To be passed to GlWidget for rendering.
    quint32 mVbo;

    bool mIsInitialized;

    QVector4D* mNewPoints;

    QVector3D mBBoxMin, mBBoxMax;

    QVector3D getWorldSize() { return mBBoxMax - mBBoxMin; }

    quint32 createVbo(quint32 size);

    float* mDevicePointSortedPos;

    unsigned int*  mDeviceMapGridCell;      // grid hash value for each particle
    unsigned int*  mDeviceMapPointIndex;    // particle index for each particle
    unsigned int*  mDeviceCellStart;        // index of start of each cell in sorted list
    unsigned int*  mDeviceCellStopp;          // index of end of cell

    struct cudaGraphicsResource *mCudaVboResource; // handles OpenGL-CUDA exchange

    // reduces the queued points against themselves, then merges all points and reduces again. Thin wrapper around reducePoints()
    void reduce();

    // reduces the given points against themselves
    quint32 reducePoints(float *devicePoints, const quint32 numElements, const bool createBoundingBox);

    void freeResources();
    void setNullPointers();


public:
    PointCloud(const QVector3D &min, const QVector3D &max, const quint32 maximumElementCount = 8 * 1024 * 1024); // 8 million points capacity
    ~PointCloud();

    bool insertPoints(const QVector<QVector3D>* const pointList);

    void updateVbo();

    // No two points closer than @distance will be inserted into this PointCloud
    void setMinimumPointDistance(const float &distance) { mParameters.minimumDistance = distance; }
    float getMinimumPointDistance() { return mParameters.minimumDistance; }

    unsigned int getNumberOfPoints(void) const { return mParameters.elementCount + mParameters.elementQueueCount; }

    void setGridSize(const QVector3D& gridSize)
    {
        mParameters.gridSize.x = gridSize.x();
        mParameters.gridSize.y = gridSize.y();
        mParameters.gridSize.z = gridSize.z();
    }

    /*
    // Returns the N nearest neighbors of a given point in space. Result is not sorted by distance to @point.
    QList<const QVector4D*> findNearestNeighbors(const QVector4D &point, const quint8 count) const;

    // Returns a list of QVector4Ds in @radius of @point.
    QList<const QVector4D*> findNeighborsWithinRadius(const QVector4D &point, const float radius) const;

    // Returns only the number of points in @radius of @point. More efficient than the methods above
    // if you're not interested in the points themselves
    quint32 numberOfNeighborsWithinRadius(const QVector4D &point, const float radius) const;

    // Yet more efficient: check whether there is AT LEAST ONE point within @radius of@point
    bool isNeighborWithinRadius(const QVector4D &point, const float radius) const;

    // Sort @list of points according to distance to @point
    void sortPointList(const QVector4D &point, QList<const QVector4D*>* list) const;
    */

public slots:
    void slotInitialize();

signals:
    void vboInfo(quint32 vbo, quint32 numElements);
};

#endif
