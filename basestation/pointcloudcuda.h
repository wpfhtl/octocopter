#ifndef POINTCLOUDCUDA_H
#define POINTCLOUDCUDA_H

#include <QVector>
#include <QVector3D>
#include <QVector4D>
#include <QTime>

#include "common.h"
#include "cudahelper.h"
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
    PointCloudCuda(const Box3D &boundingBox, const quint32 maximumElementCount = 8 * 1024 * 1024); // 8 million points capacity
    ~PointCloudCuda();

    void updateVbo();

    // No two points closer than @distance will be inserted into this PointCloud
    void setMinimumPointDistance(const float &distance);
    float getMinimumPointDistance() const { return mParameters.minimumDistance; }

    void setBoundingBox(const Box3D& box)
    {
        // We do not use the Base-Classes' mBBoxMin and mBBoxMax!
        mParameters.grid.worldMin = make_float3(box.min.x(), box.min.y(), box.min.z());
        mParameters.grid.worldMax = make_float3(box.max.x(), box.max.y(), box.max.z());
    }

    QVector3D getWorldSize() const { return CudaHelper::convert(mParameters.grid.worldMax) - CudaHelper::convert(mParameters.grid.worldMax);}

    cudaGraphicsResource** getCudaGraphicsResource() { return &mCudaVboResource;}

    quint32 getNumberOfPoints(void) const
    {
        return mParameters.elementCount + mParameters.elementQueueCount;
    }

    void setGridSize(const quint16 x, const quint16 y, const quint16 z)
    {
        mParameters.grid.cells.x = x;
        mParameters.grid.cells.y = y;
        mParameters.grid.cells.z = z;
    }

    Grid getGrid() const {return mParameters.grid;}

    const QVector<VboInfo>& getVboInfo() const { return mVboInfo; }

    quint32 getCapacity(void) const { return mParameters.capacity; }

    void setColor(const QColor& c) {mVboInfo[0].color = c;}

    bool importFromPly(const QString& fileName, QWidget* widget = 0);
    bool exportToPly(const QString& fileName, QWidget* widget = 0);

private:
    ParametersPointCloud mParameters;

    // The VBO pointing to the points. To be passed to GlWidget for rendering. We currently use just one VBO.
    QVector<VboInfo> mVboInfo;

    float* mNewPointsBuffer;
    quint32 mNewPointsBufferCursor;

    quint32 createVbo(quint32 size);

    // When someone re-sets minDist, we need to re-allocate the grid structures!
    bool mGridHasChanged;

    float* mDevicePointPos; // if its not null, then its pointing to the mapped VBO!
    // Needed, because we cannot sort particle positions according to their grid cell hash value in-place.
    // See sortPosAndVelAccordingToGridCellAndFillCellStartAndEndArraysD() for the reason.
    float* mDevicePointSortedPos;

    unsigned int*  mDeviceMapGridCell;      // grid hash value for each particle
    unsigned int*  mDeviceMapPointIndex;    // particle index for each particle
    unsigned int*  mDeviceCellStart;        // index of start of each cell in sorted list
    unsigned int*  mDeviceCellStopp;        // index of end of cell

    struct cudaGraphicsResource *mCudaVboResource; // handles OpenGL-CUDA exchange

    quint32 reducePoints(float* devicePoints, const quint32 numElements, const bool createBoundingBox);
    void freeResources();
    void initializeGrid();

public slots:
    // Tell the cloud to insert the given points. As you can see, the cloud may not change or even own the points.
    bool slotInsertPoints(const QVector<QVector3D>* const pointList);
    bool slotInsertPoints(const QVector<QVector4D>* const pointList);
    bool slotInsertPoints3(const float* const pointList, const quint32 numPoints);
    bool slotInsertPoints4(const float* const pointList, const quint32 numPoints);

    // Insert points from another PointCloudCuda
    void slotInsertPoints(PointCloud *const pointCloudSource, const quint32& firstPointToReadFromSrc = 0, quint32 numberOfPointsToCopy = 0);

    void slotInitialize();

    // Clears the datastructure, but does not destruct it. Points can still be inserted afterwards.
    void slotReset();

    // reduce the points if necessary. Use any method. A wrapper-method for all my recent attempts at reduction....
    void slotReduce();
};

#endif
