#include <GL/glew.h>

#include <cuda_gl_interop.h>
#include <cuda_runtime_api.h>

#include "pointcloudcuda.h"

PointCloudCuda::PointCloudCuda(const QVector3D &min, const QVector3D &max, const quint32 maximumElementCount) : PointCloud(min, max)
{
    mBBoxMin = min;
    mBBoxMax = max;

    mIsInitialized = false;

    mVboInfo.append(VboInfo());

    // This will be used to convert incoming points in formats other than float4 into the
    // right format. Here, we set the 4th component (w) to 1.0, because OpenGL needs it like that.
    mNewPoints = (float*)malloc(sizeof(QVector4D) * 4096);
    std::fill(mNewPoints + 0, mNewPoints + (4 * 4096), 1.0f);

    mParameters.minimumDistance = 0.02f;//1.0f/8.0f;
    mParameters.elementCount = 0;
    mParameters.elementQueueCount = 0;
    mParameters.remainder = 0;
    mParameters.capacity = maximumElementCount;
    mParameters.bBoxMin = make_float3(mBBoxMin.x(), mBBoxMin.y(), mBBoxMin.z());
    mParameters.bBoxMax = make_float3(mBBoxMax.x(), mBBoxMax.y(), mBBoxMax.z());
/*
    // The gridsize is a difficult topic:
    //
    // - if pointdiameter (=minimumDistance) is allowed to be larger than a gridcell in any dimension, then we cannot find
    //   all neighbors by looking at the (3*3*3)-1 = 26 cells immediately neighboring this one.
    //
    // - as pointsize becomes smaller than a gridcell, we will have to search through more points for each cell,
    //   possible slowing "sparsing" during insertion
    //
    // The memory requirement on the GPU is numberOfGridCells*8 bytes, so a 256*128*256 grid takes 64MB, while a
    // 512*256*512 grid takes 512MB. Given we also want to store points and do a particle simulation, 64MB is plenty :|
    //
    // Assuming a worldsize of 100*50*100 meters and a minimumDistance between points of 0.02m, we'd have to have a
    // gridCellEdgeLength of maximum 0.04m, giving 2500*1250*2500 = ~8 billion cells, which we might need to round up
    // to powers-of-two, giving 4096*2048*4096 cells. Thats too much.
    //
    // We set the size here in the constructor, but allow it to be overwritten from the outside. For clouds with less
    // points, less cells might also be ok.
    mParameters.gridSize = make_uint3(256, 128, 256);
*/
    setNullPointers();
}

PointCloudCuda::~PointCloudCuda()
{
    if(mIsInitialized) freeResources();

    free(mNewPoints);
}

void PointCloudCuda::slotInitialize()
{
    Q_ASSERT(!mIsInitialized);
/*
    // ...and thats what we do here: in mDeviceCellStart[17], you'll find
    // where in mDeviceGridParticleHash cell 17 starts!
    const unsigned int numberOfCells = mParameters.gridSize.x * mParameters.gridSize.y * mParameters.gridSize.z;

    cudaMalloc((void**)&mDeviceCellStart, numberOfCells*sizeof(uint));
    checkCudaSuccess("PointCloudCuda::slotInitialize(): memory allocation failed: cellStart");

    cudaMalloc((void**)&mDeviceCellStopp, numberOfCells*sizeof(uint));
    checkCudaSuccess("PointCloudCuda::slotInitialize(): memory allocation failed: cellStop");
*/

    // determine data-size of all points in GPU
    const unsigned int memSizePointQuadrupels = sizeof(float) * 4 * mParameters.capacity;
    // Allocate GPU data
    // Create VBO with point positions. This is later given to renderer for visualization
    mVboInfo[0].vbo = createVbo(memSizePointQuadrupels);

    qDebug() << "PointCloudCuda::initialize(): allocated" << memSizePointQuadrupels / (1024*1024) << "mb on the GPU";
    // For graphics interoperability, first register a resource for use with CUDA, then it can be mapped.
    // Registering can take a long time, so we do it here just once. Unregistering takes place when deallocating stuff.
    // During runtime, we just need to map and unmap the buffer to use it in CUDA
    cudaGraphicsGLRegisterBuffer(&mCudaVboResource, mVboInfo[0].vbo, cudaGraphicsRegisterFlagsWriteDiscard);
/*
    // Why are these sorted, and sorted according to what?
    cudaMalloc((void**)&mDevicePointSortedPos, memSizePointQuadrupels);
    checkCudaSuccess("PointCloudCuda::slotInitialize(): memory allocation failed: sortedPos");

    // These two are used to map from gridcell (=hash) to particle id (=index). If we also know in which
    // indices of these arrays grid cells start and end, we can quickly find particles in neighboring cells...
    cudaMalloc((void**)&mDeviceMapGridCell, mParameters.capacity * sizeof(uint));
    checkCudaSuccess("PointCloudCuda::slotInitialize(): memory allocation failed: mapGridCell");

    cudaMalloc((void**)&mDeviceMapPointIndex, mParameters.capacity * sizeof(uint));
    checkCudaSuccess("PointCloudCuda::slotInitialize(): memory allocation failed: mapPointIndex");

    qDebug() << "PointCloudCuda::initialize(): worldsize" << getWorldSize() << "and particle radius" << mParameters.minimumDistance << ": created system with" << mParameters.capacity << "particles and" << mParameters.gridSize.x << "*" << mParameters.gridSize.y << "*" << mParameters.gridSize.z << "cells";

    size_t memTotal, memFree;
    cudaMemGetInfo(&memFree, &memTotal);
    qDebug() << "PointCloudCuda::slotInitialize(): device has" << memFree / 1048576 << "of" << memTotal / 1048576 << "mb free";
*/
    mIsInitialized = true;
}


void PointCloudCuda::freeResources()
{
    Q_ASSERT(mIsInitialized);

    qDebug() << "PointCloudCuda::freeResources(): freeing allocated memory...";

    // Unregister the graphics resource so it is not accessible by CUDA unless registered again.
//    cudaGraphicsUnregisterResource(mCudaVboResource);
    qDebug() << "PointCloudCuda::freeResources(): done unregistering resource";

//    glDeleteBuffers(1, &mVboInfo[0].vbo);

    cudaDeviceSynchronize();

//    cudaFree(mDevicePointSortedPos);
    qDebug() << "PointCloudCuda::freeResources(): done freeing pointSortedPos";
//    cudaFree(mDeviceMapGridCell);
    qDebug() << "PointCloudCuda::freeResources(): done freeing mapGridCell";
//    cudaFree(mDeviceMapPointIndex);
    qDebug() << "PointCloudCuda::freeResources(): done freeing mapPointIndex";
//    cudaFree(mDeviceCellStart);
    qDebug() << "PointCloudCuda::freeResources(): done freeing cellStart";
//    cudaFree(mDeviceCellStopp);
    qDebug() << "PointCloudCuda::freeResources(): done freeing cellStop";

    qDebug() << "PointCloudCuda::freeResources(): done freeing newPoints";

    // this crashes. Why oh why?
//    cudaGraphicsUnregisterResource(mCudaVboResource);
    glDeleteBuffers(1, (const GLuint*)&mVboInfo[0].vbo);
    qDebug() << "PointCloudCuda::freeResources(): done freeing VBO";

    setNullPointers();

    mIsInitialized = false;

    qDebug() << "PointCloudCuda::freeResources(): done.";
}

void PointCloudCuda::setNullPointers()
{
//    mDevicePointSortedPos= 0;
//    mDeviceMapGridCell = 0;
//    mDeviceMapPointIndex = 0;
//    mDeviceCellStart = 0;
//    mDeviceCellStopp = 0;
    mVboInfo[0].vbo = 0;
}

bool PointCloudCuda::slotInsertPoints(const QVector<QVector3D>* const pointList)
{
    return slotInsertPoints3((float*)pointList->constData(), pointList->size());
}

bool PointCloudCuda::slotInsertPoints(const QVector<QVector4D>* const pointList)
{
    return slotInsertPoints4((float*)pointList->constData(), pointList->size());
}

bool PointCloudCuda::slotInsertPoints3(const float* const pointList, const quint32 numPoints)
{
    Q_ASSERT(numPoints < 4096); // max capacity of mNewPoints

    float* val = mNewPoints;

    for(int i=0;i<numPoints;i++)
    {
        *val++ = pointList[3*i+0];
        *val++ = pointList[3*i+1];
        *val++ = pointList[3*i+2];
//        *val++ = 1.0f;
        val++; // w is pre-set to 1.0 in c'tor, we don't touch it here.
    }

    return slotInsertPoints4(mNewPoints, numPoints);
}

bool PointCloudCuda::slotInsertPoints4(const float* const pointList, const quint32 numPoints)
{
    Q_ASSERT(mVboInfo[0].elementSize == 4);

    const quint32 numberOfPointsToAppend = qMin(mParameters.capacity - mParameters.elementQueueCount - mParameters.elementCount, numPoints);

    // upload the points into the device
    glBindBuffer(GL_ARRAY_BUFFER, mVboInfo[0].vbo);

    // overwrite parts of the buffer
    glBufferSubData(
                GL_ARRAY_BUFFER,
                (mParameters.elementCount + mParameters.elementQueueCount) * sizeof(float) * mVboInfo[0].elementSize,  // start
                numberOfPointsToAppend * sizeof(float) * mVboInfo[0].elementSize,         // size
                pointList);                            // source

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    mParameters.elementQueueCount += numberOfPointsToAppend;

    mVboInfo[0].size = mParameters.elementCount + mParameters.elementQueueCount;

//    qDebug() << "PointCloudCuda::slotInsertPoints4():" << mName << "inserted" << numberOfPointsToAppend << "points, vbo elements:" << mVboInfo[0].size << "elements:" << mParameters.elementCount << "queue:" << mParameters.elementQueueCount;

    reduce();

    return true;
}

bool PointCloudCuda::slotInsertPoints(const VboInfo* const vboInfo, const quint32& firstPoint, const quint32& numPoints)
{
    // Make sure the VBO layouts are compatible.
    Q_ASSERT(vboInfo->layoutMatches(&mVboInfo[0]));

    Q_ASSERT(vboInfo->elementSize == 4);
    Q_ASSERT(mVboInfo[0].elementSize == 4);

    Q_ASSERT(mVboInfo[0].vbo != vboInfo->vbo);

    const quint32 numberOfPointsToAppend = qMin(mParameters.capacity - mParameters.elementQueueCount - mParameters.elementCount, numPoints);

    // Copy numPoints from the given VBO into our own VBO
    glBindBuffer(GL_COPY_READ_BUFFER, vboInfo->vbo);
    glBindBuffer(GL_COPY_WRITE_BUFFER, mVboInfo[0].vbo);

    Q_ASSERT(mVboInfo[0].size == mParameters.elementQueueCount + mParameters.elementCount);

    glCopyBufferSubData(
                GL_COPY_READ_BUFFER,
                GL_COPY_WRITE_BUFFER,
                vboInfo->elementSize * sizeof(float) * firstPoint,      // where to start reading in src
                vboInfo->elementSize * sizeof(float) * mVboInfo[0].size,// where to start writing in dst
                vboInfo->elementSize * sizeof(float) * numberOfPointsToAppend        // number of bytes to copy
                );

    glBindBuffer(GL_COPY_READ_BUFFER, 0);
    glBindBuffer(GL_COPY_WRITE_BUFFER, 0);

    mParameters.elementQueueCount += numberOfPointsToAppend;

    mVboInfo[0].size = mParameters.elementCount + mParameters.elementQueueCount;


    qDebug() << "PointCloudCuda::slotInsertPoints():" << mName << "copying" << numberOfPointsToAppend << "elements /" << vboInfo->elementSize * sizeof(float) * numberOfPointsToAppend << "bytes";

    const quint32 oldNumPoints = mVboInfo[0].size;
    reduce();
    qDebug() << "PointCloudCuda::slotInsertPoints():" << mName << "reducing" << oldNumPoints << "to" << mParameters.elementCount << "points plus" << mParameters.elementQueueCount << "in queue.";

    return true;
}

bool PointCloudCuda::reduce()
{
    if(mParameters.elementQueueCount > mParameters.capacity / 100)
    {
        // Tell others about our new points. In the future, emit only AFTER reduction, so we reduce not all points twice.
        // But currently, our reduction does move points in our VBO almost randomly, so there is no guarantee that the
        // new points are in some well-defined region of the buffer after reduction.
        emit pointsInserted(&mVboInfo[0], mParameters.elementCount, mParameters.elementQueueCount);

//        reduceAllPointsUsingCollisions();
//        reduceUsingCellMean();
        reduceUsingSnapToGrid();
    }

    mVboInfo[0].size = mParameters.elementCount + mParameters.elementQueueCount;

    // If the cloud fills above 90%, thin it more by increasing minimumDistance
    if(mVboInfo[0].size > mParameters.capacity * 0.9f)
    {
        mParameters.minimumDistance *= 1.02;
        qDebug() << "PointCloudCuda::reduce(): after reduction, cloud is at" << ((float)mVboInfo[0].size / mParameters.capacity) * 100.0f << "% capacity, increasing min-distance to" << mParameters.minimumDistance;
    }
}

quint32 PointCloudCuda::reduceAllPointsUsingCollisions()
{
    // We want to remove all points that have been appended and have close neighbors in the pre-existing data.
    // When appended points have close neighbors in other appended points, we want to delete just one of both.
    float *devicePointsBase = (float*) mapGLBufferObject(&mCudaVboResource);
    float *devicePointsQueued = devicePointsBase + (mParameters.elementCount * 4);

    QTime time; // for profiling

    // Reduce the queued points
    time.start();
    quint32 numberOfQueuedPointsRemaining = reducePointRangeUsingCollisions(devicePointsQueued, mParameters.elementQueueCount, true);
    qDebug() << "PointCloudCuda::reduceAllPointsUsingCollisions(): reducing" << mParameters.elementQueueCount << "to" << numberOfQueuedPointsRemaining << "queued points took" << time.elapsed() << "ms";

    // Append the remaining queued points
    mParameters.elementQueueCount = 0;
    mParameters.elementCount += numberOfQueuedPointsRemaining;

    // Reduce all points
    time.start();
    quint32 numberOfPointsRemaining = reducePointRangeUsingCollisions(devicePointsBase, mParameters.elementCount, false);
    qDebug() << "PointCloudCuda::reduceAllPointsUsingCollisions(): reducing" << mParameters.elementCount << "to" << numberOfPointsRemaining << "points took" << time.elapsed() << "ms";
    mParameters.elementCount = numberOfPointsRemaining;

    // Unmap at end here to avoid unnecessary graphics/CUDA context switch.
    // Once unmapped, the resource may not be accessed by CUDA until they
    // are mapped again. This function provides the synchronization guarantee
    // that any CUDA work issued  before ::cudaGraphicsUnmapResources()
    // will complete before any subsequently issued graphics work begins.
    cudaGraphicsUnmapResources(1, &mCudaVboResource, 0);

    return numberOfPointsRemaining;
}

quint32 PointCloudCuda::reduceUsingCellMean()
{/*
    // We want to replace all points in a cell by their average position
    // First, we divide the whole pointcloud into cells, creatting very sparse clouds (as gridsize is limited and thus cellsize has a lower limit)
    // Later, we might partition the cloud into sub-cubes, cretae a grid for those and create more dense "sparse" clouds
    float *devicePoints = (float*) mapGLBufferObject(&mCudaVboResource);

    QTime time; // for profiling

    time.start();

    // Don't divide the whole region into our grid, but only the new data
    float3 bBoxMin, bBoxMax;
    getBoundingBox(devicePoints + mParameters.elementCount, mParameters.elementQueueCount, bBoxMin, bBoxMax);

    qDebug() << "PointCloudCuda::reducePointRangeUsingCollisions(): there are" << mParameters.elementQueueCount << "points with a bbox from"
             << bBoxMin.x << bBoxMin.y << bBoxMin.z << "to" << bBoxMax.x << bBoxMax.y << bBoxMax.z;

    // Define the new bounding box for all queued points
    mParameters.bBoxMin = bBoxMin;
    mParameters.bBoxMax = bBoxMax;


//    mParameters.bBoxMin = make_float3(mBBoxMin.x(), mBBoxMin.y(), mBBoxMin.z());
//    mParameters.bBoxMax = make_float3(mBBoxMax.x(), mBBoxMax.y(), mBBoxMax.z());

    mParameters.gridSize = make_uint3(512,32,512);

    setPointCloudParameters(&mParameters);

    // Build grid mapping grid cell to particle id
    computeMappingFromGridCellToPoint(mDeviceMapGridCell, mDeviceMapPointIndex, devicePoints, mParameters.elementCount + mParameters.elementQueueCount);

    // Sort this mapping according to grid cell
    sortMapAccordingToKeys(mDeviceMapGridCell, mDeviceMapPointIndex, mParameters.elementCount + mParameters.elementQueueCount);

    // Populate sorted positions buffer according to containing grid cell and fill cellStart and cellEnd tables
    sortPosAccordingToGridCellAndFillCellStartAndEndArrays(
                mDeviceCellStart,
                mDeviceCellStopp,
                mDevicePointSortedPos,
                mDeviceMapGridCell,
                mDeviceMapPointIndex,
                devicePoints,
                mParameters.elementCount + mParameters.elementQueueCount,
                mParameters.gridSize.x * mParameters.gridSize.y * mParameters.gridSize.z);

    quint32 numberOfPointsRemaining = replaceCellPointsByMeanValue(
                devicePoints,
                mDevicePointSortedPos,
                mDeviceCellStart,
                mDeviceCellStopp,
                mDeviceMapGridCell,
                mDeviceMapPointIndex,
                mParameters.elementCount + mParameters.elementQueueCount,
                mParameters.gridSize.x * mParameters.gridSize.y * mParameters.gridSize.z);

    qDebug() << "PointCloudCuda::reduceUsingCellMean(): reducing" << mParameters.elementCount + mParameters.elementQueueCount << "to" << numberOfPointsRemaining << "queued points took" << time.elapsed() << "ms";

    // Append the remaining queued points
    mParameters.elementQueueCount = 0;
    mParameters.elementCount = numberOfPointsRemaining;

    // Unmap at end here to avoid unnecessary graphics/CUDA context switch.
    // Once unmapped, the resource may not be accessed by CUDA until they
    // are mapped again. This function provides the synchronization guarantee
    // that any CUDA work issued  before ::cudaGraphicsUnmapResources()
    // will complete before any subsequently issued graphics work begins.
    cudaGraphicsUnmapResources(1, &mCudaVboResource, 0);

    return numberOfPointsRemaining;
    */
}


quint32 PointCloudCuda::reduceUsingSnapToGrid()
{
    // We want to remove all points that have been appended and have close neighbors in the pre-existing data.
    // When appended points have close neighbors in other appended points, we want to delete just one of both.
    float *devicePointsBase = (float*) mapGLBufferObject(&mCudaVboResource);

    QTime time; // for profiling

    // Reduce the queued points
//    time.start();
    quint32 numberOfQueuedPointsRemaining = snapToGridAndMakeUnique(devicePointsBase, mParameters.elementCount + mParameters.elementQueueCount, mParameters.minimumDistance);
//    qDebug() << "PointCloudCuda::reduceUsingSnapToGrid(): reducing" << mParameters.elementCount + mParameters.elementQueueCount << "to" << numberOfQueuedPointsRemaining << "queued points (dist" << mParameters.minimumDistance << ") took" << time.elapsed() << "ms";

    // Append the remaining queued points
    mParameters.elementQueueCount = 0;
    mParameters.elementCount = numberOfQueuedPointsRemaining;

    // Unmap at end here to avoid unnecessary graphics/CUDA context switch.
    // Once unmapped, the resource may not be accessed by CUDA until they
    // are mapped again. This function provides the synchronization guarantee
    // that any CUDA work issued  before ::cudaGraphicsUnmapResources()
    // will complete before any subsequently issued graphics work begins.
    cudaGraphicsUnmapResources(1, &mCudaVboResource, 0);

    return numberOfQueuedPointsRemaining;
}

quint32 PointCloudCuda::reducePointRangeUsingCollisions(float* devicePoints, const quint32 numElements, const bool createBoundingBox)
{/*
    qDebug() << "PointCloudCuda::reducePointRangeUsingCollisions(): reducing" << numElements << "points, creating bbox:" << createBoundingBox;

    checkCudaSuccess("PointCloudCuda::reducePointRangeUsingCollisions(): CUDA error before reduction");

    if(createBoundingBox)
    {
        // Determine bounding-box of given points
        float3 bBoxMin, bBoxMax;
        getBoundingBox(devicePoints, numElements, bBoxMin, bBoxMax);

        qDebug() << "PointCloudCuda::reducePointRangeUsingCollisions(): there are" << numElements << "points with a bbox from"
                 << bBoxMin.x << bBoxMin.y << bBoxMin.z << "to" << bBoxMax.x << bBoxMax.y << bBoxMax.z;

        // Define the new bounding box for all queued points
        mParameters.bBoxMin = bBoxMin;
        mParameters.bBoxMax = bBoxMax;
    }

    setPointCloudParameters(&mParameters);

    // Build grid mapping grid cell to particle id
    computeMappingFromGridCellToPoint(mDeviceMapGridCell, mDeviceMapPointIndex, devicePoints, numElements);

    // Sort this mapping according to grid cell
    sortMapAccordingToKeys(mDeviceMapGridCell, mDeviceMapPointIndex, numElements);

    // Populate sorted positions buffer according to containing grid cell and fill cellStart and cellEnd tables
    sortPosAccordingToGridCellAndFillCellStartAndEndArrays(
                mDeviceCellStart,
                mDeviceCellStopp,
                mDevicePointSortedPos,
                mDeviceMapGridCell,
                mDeviceMapPointIndex,
                devicePoints,
                numElements,
                mParameters.gridSize.x * mParameters.gridSize.y * mParameters.gridSize.z);

    // Mark redundant points by colliding with themselves using mParameters.minimumDistance
    markCollidingPoints(
                devicePoints,
                mDevicePointSortedPos,
                mDeviceMapPointIndex,
                mDeviceCellStart,
                mDeviceCellStopp,
                numElements,
                mParameters.gridSize.x * mParameters.gridSize.y * mParameters.gridSize.z);

    // Remove all points with values 0/0/0/0
    const quint32 remainingElements = removeZeroPoints(devicePoints, numElements);

    checkCudaSuccess("PointCloudCuda::reducePointRangeUsingCollisions(): CUDA error after reduction");

    size_t memTotal, memFree;
    cudaMemGetInfo(&memFree, &memTotal);
    qDebug() << "PointCloudCuda::reducePointRangeUsingCollisions(): device has" << memFree / 1048576 << "of" << memTotal / 1048576 << "mb free";

    if(createBoundingBox)
    {
        // Re-set the bounding box back to the whole pointcloud, not just the queued points
        mParameters.bBoxMin = make_float3(mBBoxMin.x(), mBBoxMin.y(), mBBoxMin.z());
        mParameters.bBoxMax = make_float3(mBBoxMax.x(), mBBoxMax.y(), mBBoxMax.z());
    }

    return remainingElements;*/
    return 0;
}

quint32 PointCloudCuda::createVbo(quint32 size)
{
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    checkCudaSuccess("PointCloudCuda::createVBO(): VBO allocation failed");
    return vbo;
}

void PointCloudCuda::slotReset()
{

}

bool PointCloudCuda::importFromPly(const QString& fileName, QWidget* widget)
{
    PlyManager pm(widget);

    if(!pm.open(fileName, PlyManager::DataLoadFromFile))
        return false;

    quint32 numPointsToLoad = pm.getNumberOfPoints();
    numPointsToLoad = qMin(mParameters.capacity - mParameters.elementQueueCount - mParameters.elementCount, numPointsToLoad);

    float* points = new float[4*numPointsToLoad];

    pm.loadPly4D(points, 0, numPointsToLoad);

    slotInsertPoints4(points, numPointsToLoad);

    delete points;

    return true;
}

bool PointCloudCuda::exportToPly(const QString& fileName, QWidget *widget) const
{
    PlyManager pm(widget);
    if(!pm.open(fileName, PlyManager::DataSaveToFile))
        return false;

    pm.writeHeader(getNumberOfPoints(), PlyManager::NormalsNotIncluded, PlyManager::DirectionNotIncluded);

    for(int i=0;i < mVboInfo.size();i++)
    {
        glBindBuffer(GL_ARRAY_BUFFER, mVboInfo[0].vbo);

        float* points = (float*)glMapBuffer(GL_ARRAY_BUFFER, GL_READ_ONLY);

        pm.savePly4D(points, mVboInfo.at(i).size);

        glUnmapBuffer(GL_ARRAY_BUFFER);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    return true;
}
