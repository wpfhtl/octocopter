#include <GL/glew.h>

#include <cuda_gl_interop.h>
#include <cuda_runtime_api.h>

#include "pointcloudcuda.h"

PointCloudCuda::PointCloudCuda(const QVector3D &min, const QVector3D &max, const quint32 maximumElementCount) : PointCloud(min, max)
{
    mVboInfo.append(VboInfo());

    mCudaVboResource = 0;

    mParameters.initialize();
    mParameters.grid.cells = make_uint3(256, 32, 256);
    mParameters.minimumDistance = 0.04f;
    mParameters.capacity = maximumElementCount;

    // Set to zero, so we dont cudaFree() them before first init
    mDeviceCellStart = 0;
    mDeviceCellStopp = 0;
    mDeviceMapGridCell = 0;
    mDeviceMapPointIndex = 0;

    mGridHasChanged = true;

    // This will be used to convert incoming points in formats other than float4 into the
    // right format. Here, we set the 4th component (w) to 1.0, because OpenGL needs it like that.
    mNewPointsBufferCursor = 0;
    mNewPointsBuffer = (float*)malloc(sizeof(QVector4D) * 4096);
    std::fill(mNewPointsBuffer + 0, mNewPointsBuffer + (4 * 4096), 1.0f);

    setBoundingBox(min, max);
}

PointCloudCuda::~PointCloudCuda()
{
    free(mNewPointsBuffer);

    // If it was ever used, unregister the graphics resource so it is not accessible by CUDA unless registered again.
    if(mCudaVboResource)
    {
        cudaSafeCall(cudaGraphicsUnregisterResource(mCudaVboResource));
        qDebug() << "PointCloudCuda::freeResources(): done unregistering resource";
    }

    glDeleteBuffers(1, &mVboInfo[0].vbo);
    cudaSafeCall(cudaFree(mDeviceMapGridCell));
    cudaSafeCall(cudaFree(mDeviceMapPointIndex));
    cudaSafeCall(cudaFree(mDeviceCellStart));
    cudaSafeCall(cudaFree(mDeviceCellStopp));

    qDebug() << "PointCloudCuda::freeResources(): done.";
}

void PointCloudCuda::slotInitialize()
{
    // determine data-size of all points in GPU
    const unsigned int memSizePointQuadrupels = sizeof(float) * 4 * mParameters.capacity;
    // Allocate GPU data
    // Create VBO with point positions. This is later given to renderer for visualization
    mVboInfo[0].vbo = createVbo(memSizePointQuadrupels);

    // Allocate storage for sorted points
    cudaSafeCall(cudaMalloc((void**)&mDevicePointSortedPos, memSizePointQuadrupels));
    qDebug() << "PointCloudCuda::initialize(): allocated" << (2 * memSizePointQuadrupels) / (1024*1024) << "mb on the GPU for cloud with max" << mParameters.capacity / (1024*1024) << "million points.";

    // For graphics interoperability, first register a resource for use with CUDA, then it can be mapped.
    // Registering can take a long time, so we do it here just once. Unregistering takes place when deallocating stuff.
    // During runtime, we just need to map and unmap the buffer to use it in CUDA
    cudaGraphicsGLRegisterBuffer(&mCudaVboResource, mVboInfo[0].vbo, cudaGraphicsRegisterFlagsNone); // WriteDiscard?
}

void PointCloudCuda::initializeGrid()
{
    Q_ASSERT(mGridHasChanged);

    mParameters.grid.cells = mParameters.grid.getOptimalResolution(mParameters.minimumDistance);
    copyParametersToGpu(&mParameters);

    const quint32 numberOfCells = mParameters.grid.cellCount();

    if(mDeviceCellStart) cudaSafeCall(cudaFree(mDeviceCellStart));
    cudaSafeCall(cudaMalloc((void**)&mDeviceCellStart, numberOfCells*sizeof(unsigned int)));
    if(mDeviceCellStopp) cudaSafeCall(cudaFree(mDeviceCellStopp));
    cudaSafeCall(cudaMalloc((void**)&mDeviceCellStopp, numberOfCells*sizeof(unsigned int)));

    // These two are used to map from gridcell (=hash) to particle id (=index). If we also know in which
    // indices of these arrays grid cells start and end, we can quickly find particles in neighboring cells...
    if(mDeviceMapGridCell) cudaSafeCall(cudaFree(mDeviceMapGridCell));
    cudaSafeCall(cudaMalloc((void**)&mDeviceMapGridCell, mParameters.capacity * sizeof(unsigned int)));
    if(mDeviceMapPointIndex) cudaSafeCall(cudaFree(mDeviceMapPointIndex));
    cudaSafeCall(cudaMalloc((void**)&mDeviceMapPointIndex, mParameters.capacity * sizeof(unsigned int)));

    qDebug() << "PointCloudCuda::initializeGrid(): worldSize" << getWorldSize() << "minDist" << mParameters.minimumDistance << ": created grid with" << mParameters.grid.cells.x << "*" << mParameters.grid.cells.y << "*" << mParameters.grid.cells.z << "cells";

    size_t memTotal, memFree;
    cudaMemGetInfo(&memFree, &memTotal);
    qDebug() << "PointCloudCuda::initializeGrid(): device has" << memFree / 1048576 << "of" << memTotal / 1048576 << "mb free";

    mGridHasChanged = false;
}

void PointCloudCuda::setMinimumPointDistance(const float &distance)
{
    mParameters.minimumDistance = distance;

    // When minDist changes, we should recalculate the grid, so that collisions can be reliably found in own and neighboring cells!
    mGridHasChanged = true;
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

    float* val = mNewPointsBuffer + (mNewPointsBufferCursor * 4);

    for(int i=0;i<numPoints;i++)
    {
        *val++ = pointList[3*i+0];
        *val++ = pointList[3*i+1];
        *val++ = pointList[3*i+2];
//        *val++ = 1.0f;
        val++; // w is pre-set to 1.0 in c'tor, we don't touch it here.
    }

    float* x = mNewPointsBuffer + (mNewPointsBufferCursor * 4);
    for(int i=0;i<10;i++)
        qDebug() << "point" << i << ":" << *x++ << *x++ << *x++ << *x++;

    mNewPointsBufferCursor += numPoints;

    if(mNewPointsBufferCursor > 3000)
    {
        slotInsertPoints4(mNewPointsBuffer, mNewPointsBufferCursor);
        mNewPointsBufferCursor = 0;
    }
    else
    {
        // TODO: set up a qtimer to flush the buffer, just in case no new data comes in.
    }

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

    mVboInfo[0].size = getNumberOfPoints();

//    qDebug() << "PointCloudCuda::slotInsertPoints4():" << mName << "inserted" << numberOfPointsToAppend << "points, vbo elements:" << mVboInfo[0].size << "elements:" << mParameters.elementCount << "queue:" << mParameters.elementQueueCount;

//    if(mParameters.elementQueueCount > mParameters.capacity / 20) slotReduce();

    return true;
}

bool PointCloudCuda::slotInsertPoints(const VboInfo* const vboInfo, const quint32& firstPoint, const quint32& numPoints)
{
    Q_ASSERT(false);
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
//    if(mParameters.elementQueueCount > mParameters.capacity / 100) slotReduce();

    qDebug() << "PointCloudCuda::slotInsertPoints():" << mName << "reducing" << oldNumPoints << "to" << mParameters.elementCount << "points plus" << mParameters.elementQueueCount << "in queue.";

    return true;
}

void PointCloudCuda::slotInsertPoints(PointCloud *const pointCloudSource, const quint32& firstPointToReadFromSrc, quint32 numberOfPointsToCopy)
{
    // Make sure the VBO layouts are compatible.
    Q_ASSERT(pointCloudSource->getVboInfo()[0].layoutMatches(&mVboInfo[0]));

    QTime time; time.start();
    float *devicePointsBaseSrc = (float*) mapGLBufferObject(((PointCloudCuda*)pointCloudSource)->getCudaGraphicsResource());
    float *devicePointsBaseDst = (float*) mapGLBufferObject(getCudaGraphicsResource());

    // By default, copy all points from pointCloudSource
    if(numberOfPointsToCopy == 0) numberOfPointsToCopy = pointCloudSource->getNumberOfPoints();

    const quint32 numberOfPointsBeforeCopy = getNumberOfPoints();

    quint32 numberOfPointsProcessedInSrc = 0;
    // We only fill our cloud up to 95% capacity. Otherwise, we'd have maaaany iterations filling it completely, then reducing to 99.99%, refilling, 99.991%, ...

    qDebug() << "PointCloudCuda::slotInsertPoints(): current occupancy:" <<getNumberOfPoints() << "/" << mParameters.capacity << ": there are" << numberOfPointsToCopy << "to insert";

    while(mParameters.elementCount < mParameters.capacity * 0.95f && numberOfPointsProcessedInSrc < pointCloudSource->getNumberOfPoints() - firstPointToReadFromSrc && numberOfPointsProcessedInSrc < numberOfPointsToCopy)
    {
        const quint32 freeSpaceInDst = mParameters.capacity - mParameters.elementCount - mParameters.elementQueueCount;

        const quint32 numberOfPointsToCopyInThisIteration = std::min(
                    std::min(
                        freeSpaceInDst, // space left in dst
                        pointCloudSource->getNumberOfPoints() - numberOfPointsProcessedInSrc - firstPointToReadFromSrc), // points left in src
                    numberOfPointsToCopy - numberOfPointsProcessedInSrc // number of points left to copy
                    );

        qDebug() << "PointCloudCuda::slotInsertPoints():" << numberOfPointsProcessedInSrc << "points inserted from source, space left in dst:" << freeSpaceInDst << "- inserting" << numberOfPointsToCopyInThisIteration << "points from src.";

        if(mAcceptPointsOutsideBoundingBox)
        {
            qDebug() << "PointCloudCuda::slotInsertPoints(): mAcceptPointsOutsideBoundingBox is true, copying all given points.";
            mParameters.elementCount += copyPoints(
                        devicePointsBaseDst + ((numberOfPointsProcessedInSrc) * sizeof(float) * mVboInfo[0].elementSize),
                        devicePointsBaseSrc + ((firstPointToReadFromSrc + numberOfPointsProcessedInSrc) * sizeof(float) * mVboInfo[0].elementSize),
                        numberOfPointsToCopyInThisIteration);
        }
        else
        {
            qDebug() << "PointCloudCuda::slotInsertPoints(): mAcceptPointsOutsideBoundingBox is false, copying only points in bbox.";
            mParameters.elementCount += copyPointsInBoundingBox(
                        devicePointsBaseDst + (getNumberOfPoints() * mVboInfo[0].elementSize),
                        devicePointsBaseSrc + ((firstPointToReadFromSrc + numberOfPointsProcessedInSrc) * mVboInfo[0].elementSize),
                        mParameters.grid.worldMin,
                        mParameters.grid.worldMax,
                        numberOfPointsToCopyInThisIteration);
        }

        // TODO: REDUCE!
        mParameters.elementCount = snapToGridAndMakeUnique(devicePointsBaseDst, getNumberOfPoints(), mParameters.minimumDistance);
        mParameters.elementQueueCount = 0;
        qDebug() << "PointCloudCuda::slotInsertPoints(): after reducing points using snapToGridAndMakeUnique(), cloud contains" << mParameters.elementCount << "of" << mParameters.capacity << "points";

        numberOfPointsProcessedInSrc += numberOfPointsToCopyInThisIteration;
    }

    mVboInfo[0].size = getNumberOfPoints();

    if(numberOfPointsProcessedInSrc < numberOfPointsToCopy)
        qDebug() << "PointCloudCuda::slotInsertPoints(): didn't insert all points from src (due to insufficient destination capacity of" << mParameters.capacity << "?)";

    cudaGraphicsUnmapResources(1, getCudaGraphicsResource(), 0);
    cudaGraphicsUnmapResources(1, ((PointCloudCuda*)pointCloudSource)->getCudaGraphicsResource(), 0);

    qDebug() << "PointCloudCuda::slotInsertPoints(): took" << time.elapsed() << "ms.";

    emit pointsInserted(this, numberOfPointsBeforeCopy, getNumberOfPoints() - numberOfPointsBeforeCopy);
}

void PointCloudCuda::slotReduce()
{
    if(mGridHasChanged) initializeGrid();

    // Don't reduce pathetic amounts of points
    if(mParameters.elementQueueCount < 1000) return;

    emit pointsInserted(this, mParameters.elementCount, mParameters.elementQueueCount);

    // We want to remove all points that have been appended and have close neighbors in the pre-existing data.
    // When appended points have close neighbors in other appended points, we want to delete just one of both.
    float *devicePointsBase = (float*) mapGLBufferObject(&mCudaVboResource);
    float *devicePointsQueued = devicePointsBase + (mParameters.elementCount * 4);

    QTime time; // for profiling

    // Reduce the queued points
    time.start();
    quint32 numberOfQueuedPointsRemaining = reducePoints(devicePointsQueued, mParameters.elementQueueCount, /*true*/false);
    qDebug() << "PointCloudCuda::reduce(): reducing" << mParameters.elementQueueCount << "to" << numberOfQueuedPointsRemaining << "queued points took" << time.elapsed() << "ms";

    // Append the remaining queued points
    mParameters.elementQueueCount = 0;
    mParameters.elementCount += numberOfQueuedPointsRemaining;

    // Reduce all points
    time.start();
    quint32 numberOfPointsRemaining = reducePoints(devicePointsBase, mParameters.elementCount, false);
    qDebug() << "PointCloudCuda::reduce(): reducing" << mParameters.elementCount << "to" << numberOfPointsRemaining << "points took" << time.elapsed() << "ms";
    mParameters.elementCount = numberOfPointsRemaining;

    // Unmap at end here to avoid unnecessary graphics/CUDA context switch.
    // Once unmapped, the resource may not be accessed by CUDA until they
    // are mapped again. This function provides the synchronization guarantee
    // that any CUDA work issued  before ::cudaGraphicsUnmapResources()
    // will complete before any subsequently issued graphics work begins.
    cudaGraphicsUnmapResources(1, &mCudaVboResource, 0);
}

quint32 PointCloudCuda::reducePoints(float* devicePoints, const quint32 numElements, const bool createBoundingBox)
{
    qDebug() << "PointCloudCuda::reducePoints(): reducing" << numElements << "points, creating bbox:" << createBoundingBox;

//    checkCudaSuccess("PointCloudCuda::reducePoints(): CUDA error before reduction");

    if(createBoundingBox)
    {
        // Determine bounding-box of given points
        float3 bBoxMin, bBoxMax;
        getBoundingBox(devicePoints, numElements, bBoxMin, bBoxMax);

        qDebug() << "PointCloudCuda::reduceQueuedPoints(): there are" << numElements << "points with a bbox from"
                 << bBoxMin.x << bBoxMin.y << bBoxMin.z << "to" << bBoxMax.x << bBoxMax.y << bBoxMax.z;

        // Define the new bounding box for all queued points
        mParameters.grid.worldMin = bBoxMin;
        mParameters.grid.worldMax = bBoxMax;
    }

    copyParametersToGpu(&mParameters);

    ParametersPointCloud* paramsOnGpu;
    getDeviceAddressOfParametersPointCloud(&paramsOnGpu);

    // Build grid mapping grid cell to particle id
//    computeMappingFromGridCellToPoint(mDeviceMapGridCell, mDeviceMapPointIndex, devicePoints, numElements);
    computeMappingFromPointToGridCell(
            mDeviceMapGridCell,
            mDeviceMapPointIndex,
            devicePoints,
            &paramsOnGpu->grid,
            numElements);

    // Sort this mapping according to grid cell
    sortMapAccordingToKeys(mDeviceMapGridCell, mDeviceMapPointIndex, numElements);

    // Populate sorted positions buffer according to containing grid cell and fill cellStart and cellEnd tables
    /*sortPosAccordingToGridCellAndFillCellStartAndEndArrays(
                mDeviceCellStart,
                mDeviceCellStopp,
                mDevicePointSortedPos,
                mDeviceMapGridCell,
                mDeviceMapPointIndex,
                devicePoints,
                numElements,
                mParameters.grid.cellCount());*/

    sortParticlePosAndVelAccordingToGridCellAndFillCellStartAndEndArrays(
                mDeviceCellStart,                           // output: At which index in mDeviceMapParticleIndex does cell X start?
                mDeviceCellStopp,                             // output: At which index in mDeviceMapParticleIndex does cell X end?
                mDevicePointSortedPos,                   // output: The particle positions, sorted by gridcell
                0,                   // output: The particle velocities, sorted by gridcell
                mDeviceMapGridCell,                 // input:  The key - part of the particle gridcell->index map, unsorted
                mDeviceMapPointIndex,                    // input:  The value-part of the particle gridcell->index map, unsorted
                devicePoints,                    // input:  The particle-positions, unsorted
                0,                         // input:  The particle-velocities, unsorted
                numElements,        // input:  The number of particles
                mParameters.grid.cellCount()  // input: Number of grid cells
                );

    // Mark redundant points by colliding with themselves using mParameters.minimumDistance
    markCollidingPoints(
                devicePoints,
                mDevicePointSortedPos,
                mDeviceMapPointIndex,
                mDeviceCellStart,
                mDeviceCellStopp,
                numElements);

    // Remove all points with values 0/0/0/0
    const quint32 remainingElements = removeClearedPoints(devicePoints, numElements);

    size_t memTotal, memFree;
    cudaMemGetInfo(&memFree, &memTotal);
    qDebug() << "PointCloudCuda::reducePoints(): device has" << memFree / 1048576 << "of" << memTotal / 1048576 << "mb free";

    if(createBoundingBox)
    {
        // Re-set the bounding box back to the whole pointcloud, not just the queued points
        mParameters.grid.worldMin = make_float3(mBBoxMin.x(), mBBoxMin.y(), mBBoxMin.z());
        mParameters.grid.worldMax = make_float3(mBBoxMax.x(), mBBoxMax.y(), mBBoxMax.z());
    }

    return remainingElements;
}





quint32 PointCloudCuda::reducePointRangeUsingCollisions(float* devicePoints, const quint32 numElements, const bool createBoundingBox)
{/*
    qDebug() << "PointCloudCuda::reducePointRangeUsingCollisions(): reducing" << numElements << "points, creating bbox:" << createBoundingBox;

    if(createBoundingBox)
    {
        // Determine bounding-box of given points
        float3 bBoxMin, bBoxMax;
        getBoundingBox(devicePoints, numElements, bBoxMin, bBoxMax);

        qDebug() << "PointCloudCuda::reducePointRangeUsingCollisions(): there are" << numElements << "points with a bbox from"
                 << bBoxMin.x << bBoxMin.y << bBoxMin.z << "to" << bBoxMax.x << bBoxMax.y << bBoxMax.z;

        // Define the new bounding box for all queued points
        mParameters.grid.worldMin = bBoxMin;
        mParameters.grid.worldMax = bBoxMax;
    }

    copyParametersToGpu(&mParameters);

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
                mParameters.grid.cellCount());

    // Mark redundant points by colliding with themselves using mParameters.minimumDistance
    markCollidingPoints(
                devicePoints,
                mDevicePointSortedPos,
                mDeviceMapPointIndex,
                mDeviceCellStart,
                mDeviceCellStopp,
                numElements,
                mParameters.grid.cellCount());

    // Remove all points with values 0/0/0/0
    const quint32 remainingElements = removeClearedPoints(devicePoints, numElements);

    size_t memTotal, memFree;
    cudaMemGetInfo(&memFree, &memTotal);
    qDebug() << "PointCloudCuda::reducePointRangeUsingCollisions(): device has" << memFree / 1048576 << "of" << memTotal / 1048576 << "mb free";

    if(createBoundingBox)
    {
        // Re-set the bounding box back to the whole pointcloud, not just the queued points
        mParameters.grid.worldMin = make_float3(mBBoxMin.x(), mBBoxMin.y(), mBBoxMin.z());
        mParameters.grid.worldMax = make_float3(mBBoxMax.x(), mBBoxMax.y(), mBBoxMax.z());
    }

    return remainingElements;*/
    return 0;
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




quint32 PointCloudCuda::reduceToCellMean()
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
    mParameters.grid.worldMin = bBoxMin;
    mParameters.grid.worldMax = bBoxMax;


//    mParameters.grid.worldMin = make_float3(mBBoxMin.x(), mBBoxMin.y(), mBBoxMin.z());
//    mParameters.grid.worldMax = make_float3(mBBoxMax.x(), mBBoxMax.y(), mBBoxMax.z());

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
/*
quint32 PointCloudCuda::trimToBoundingBox()
{
    // We want to remove all points that have been appended and have close neighbors in the pre-existing data.
    // When appended points have close neighbors in other appended points, we want to delete just one of both.
    float *devicePointsBase = (float*)mapGLBufferObject(&mCudaVboResource);

//    QTime time; // for profiling

    // Reduce the queued points
//    time.start();
    quint32 numberOfPointsRemaining = removePointsOutsideBoundingBox(devicePointsBase, mParameters.elementCount + mParameters.elementQueueCount, mParameters.grid.worldMin, mParameters.grid.worldMax);
//    qDebug() << "PointCloudCuda::reduceUsingSnapToGrid(): reducing" << mParameters.elementCount + mParameters.elementQueueCount << "to" << numberOfQueuedPointsRemaining << "queued points (dist" << mParameters.minimumDistance << ") took" << time.elapsed() << "ms";

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
}*/

quint32 PointCloudCuda::reduceUsingSnapToGrid(float* devicePoints, quint32 numberOfPoints)
{/*
    // We want to remove all points that have been appended and have close neighbors in the pre-existing data.
    // When appended points have close neighbors in other appended points, we want to delete just one of both.
    float* devicePointsBase;

    if(devicePoints == 0)
        *devicePointsBase = (float*)mapGLBufferObject(&mCudaVboResource);
    else
        *devicePointsBase = devicePoints;

    if(numberOfPoints == 0)
        numberOfPoints = mParameters.elementCount + mParameters.elementQueueCount;

    quint32 numberOfQueuedPointsRemaining = snapToGridAndMakeUnique(devicePointsBase, numberOfPoints, mParameters.minimumDistance);

    // Append the remaining queued points
    mParameters.elementQueueCount = 0;
    mParameters.elementCount = numberOfQueuedPointsRemaining;

    if(devicePoints == 0) cudaGraphicsUnmapResources(1, &mCudaVboResource, 0);

    return numberOfQueuedPointsRemaining;*/
}

quint32 PointCloudCuda::createVbo(quint32 size)
{
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    return vbo;
}

void PointCloudCuda::slotReset()
{
    mParameters.elementQueueCount = 0;
    mParameters.elementCount = 0;
    mVboInfo[0].size = 0;
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
