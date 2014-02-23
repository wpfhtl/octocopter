#include <cuda_gl_interop.h>
#include <cuda_runtime_api.h>
#include "cudahelper.cuh"
#include "pointcloudcuda.h"

PointCloudCuda::PointCloudCuda(const Box3D& boundingBox, const quint32 maximumElementCount, const QString name) : PointCloud()
{
    mRenderInfoList.append(new RenderInfo());
    qDebug() << "renderinfolistsize of cloud" << name << mRenderInfoList.size();

    mIsInitialized = false;

    mName = name;
    mCudaVboResource = 0;

    mParameters.initialize();
    mParameters.grid.worldMin = make_float3(boundingBox.min.x(), boundingBox.min.y(), boundingBox.min.z());
    mParameters.grid.worldMax = make_float3(boundingBox.max.x(), boundingBox.max.y(), boundingBox.max.z());
    mParameters.grid.cells = make_uint3(256, 32, 256);
    mParameters.minimumDistance = 0.02f;
    mParameters.capacity = maximumElementCount;

    mDevicePointPos = nullptr;

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

    setBoundingBox(boundingBox);
}

PointCloudCuda::~PointCloudCuda()
{
    free(mNewPointsBuffer);

    if(CudaHelper::isDeviceSupported)
    {
        // If it was ever used, unregister the graphics resource so it is not accessible by CUDA unless registered again.
        if(mCudaVboResource)
        {
            qDebug() << __PRETTY_FUNCTION__ << mName << "unregistering resource...";
            cudaSafeCall(cudaGraphicsUnregisterResource(mCudaVboResource));
        }

        glDeleteBuffers(1, &mRenderInfoList[0]->vbo);
        cudaSafeCall(cudaFree(mDeviceMapGridCell));
        cudaSafeCall(cudaFree(mDeviceMapPointIndex));
        cudaSafeCall(cudaFree(mDeviceCellStart));
        cudaSafeCall(cudaFree(mDeviceCellStopp));

        qDebug() << __PRETTY_FUNCTION__ << mName << "done.";
    }
}

void PointCloudCuda::initialize()
{
    qDebug() << __PRETTY_FUNCTION__ << mName;
    initializeOpenGLFunctions();

    // determine data-size of all points in GPU
    const unsigned int memSizePointQuadrupels = sizeof(float) * 4 * mParameters.capacity;
    // Allocate GPU data
    // Create VBO with point positions. This is later given to renderer for visualization
    mRenderInfoList[0]->vbo = createVbo(memSizePointQuadrupels);

    if(CudaHelper::isDeviceSupported)
    {
        // Allocate storage for sorted points
        cudaSafeCall(cudaMalloc((void**)&mDevicePointSortedPos, memSizePointQuadrupels));
        qDebug() << __PRETTY_FUNCTION__ << mName << "allocated" << (2 * memSizePointQuadrupels) / (1024*1024) << "mb on the GPU for cloud with max" << mParameters.capacity / (1024*1024) << "million points.";

        // For graphics interoperability, first register a resource for use with CUDA, then it can be mapped.
        // Registering can take a long time, so we do it here just once. Unregistering takes place when deallocating stuff.
        // During runtime, we just need to map and unmap the buffer to use it in CUDA
        cudaGraphicsGLRegisterBuffer(&mCudaVboResource, mRenderInfoList[0]->vbo, cudaGraphicsRegisterFlagsNone);
    }

    mIsInitialized = true;
}

void PointCloudCuda::initializeGrid()
{
    qDebug() << __PRETTY_FUNCTION__ << mName;

    Q_ASSERT(mGridHasChanged);

    if(!CudaHelper::isDeviceSupported)
    {
        qDebug() << "PointCloudCuda::initializeGrid(): device not supported, returning.";
        return;
    }

    mParameters.grid.cells = mParameters.grid.getOptimumResolution(mParameters.minimumDistance);
    copyParametersToGpu(&mParameters);

    const quint32 numberOfCells = mParameters.grid.getCellCount();

    if(mDeviceCellStart) cudaSafeCall(cudaFree(mDeviceCellStart));
    cudaSafeCall(cudaMalloc((void**)&mDeviceCellStart, numberOfCells * sizeof(unsigned int)));
    if(mDeviceCellStopp) cudaSafeCall(cudaFree(mDeviceCellStopp));
    cudaSafeCall(cudaMalloc((void**)&mDeviceCellStopp, numberOfCells * sizeof(unsigned int)));

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
    qDebug() << __PRETTY_FUNCTION__ << mName << distance;

    mParameters.minimumDistance = distance;

    // When minDist changes, we should recalculate the grid, so that collisions can be reliably found in own and neighboring cells!
    mGridHasChanged = true;
}

bool PointCloudCuda::slotInsertPoints(const QVector<QVector3D>* const pointList)
{
    Q_ASSERT(false);
    return slotInsertPoints3((float*)pointList->constData(), pointList->size());
}

bool PointCloudCuda::slotInsertPoints(const QVector<QVector4D>* const pointList)
{
    Q_ASSERT(false);
    return slotInsertPoints4((float*)pointList->constData(), pointList->size());
}

bool PointCloudCuda::slotInsertPoints3(const float* const pointList, const quint32 numPoints)
{
    Q_ASSERT(false);
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
    if(!mIsInitialized) initialize();

    Q_ASSERT(mRenderInfoList[0]->elementSize == 4);

    const quint32 numberOfPointsToAppendHonoringEndOfBuffer = qMin(
                mParameters.capacity - ((mParameters.elementCount + mParameters.elementQueueCount) % mParameters.capacity),
                numPoints
                );

    glBindBuffer(GL_ARRAY_BUFFER, mRenderInfoList[0]->vbo);

    // overwrite parts of the buffer
    glBufferSubData(
            GL_ARRAY_BUFFER,
            ((mParameters.elementCount + mParameters.elementQueueCount) % mParameters.capacity) * sizeof(float) * mRenderInfoList[0]->elementSize,  // start
            numberOfPointsToAppendHonoringEndOfBuffer * sizeof(float) * mRenderInfoList[0]->elementSize,         // size
            pointList);                            // source

    mParameters.elementQueueCount += numberOfPointsToAppendHonoringEndOfBuffer;

    if(numberOfPointsToAppendHonoringEndOfBuffer != numPoints)
    {
        // We just wrote until the end of the buffer and haven't inserted all points yet. Let's
        // restart at the beginning of the buffer and write the remaining points
        qDebug() << __PRETTY_FUNCTION__ << "overwriting begining of buffer - free refill!!!";

        const quint32 numberOfPointsLeftToInsert = numPoints - numberOfPointsToAppendHonoringEndOfBuffer;

        glBufferSubData(
                GL_ARRAY_BUFFER,
                0,          // start
                numberOfPointsLeftToInsert * sizeof(float) * mRenderInfoList[0]->elementSize, // size
                pointList + (mRenderInfoList[0]->elementSize * numberOfPointsToAppendHonoringEndOfBuffer));                // source

        // We should now mark all points as being in the queue!
        mParameters.elementQueueCount += mParameters.elementCount + numberOfPointsLeftToInsert;
        mParameters.elementCount = 0;
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    mRenderInfoList[0]->size = getNumberOfPointsStored();

    emit parameters(&mParameters);

//    qDebug() << "PointCloudCuda::slotInsertPoints4():" << mName << "inserted" << numberOfPointsToAppend << "points, vbo elements:" << mVboInfo[0].size << "elements:" << mParameters.elementCount << "queue:" << mParameters.elementQueueCount;

    return true;
}

void PointCloudCuda::slotInsertPoints(PointCloudCuda *const pointCloudSource, const quint32& firstPointToReadFromSrc, quint32 numberOfPointsToCopy)
{
    qDebug() << __PRETTY_FUNCTION__ << mName << "receiving points from cloud" << pointCloudSource->mName;

    if(pointCloudSource->mParameters.minimumDistance >= mParameters.minimumDistance)
        qDebug() << __PRETTY_FUNCTION__ << mName << "ERROR, cloud" << pointCloudSource->mName << "is less dense than we are, this should be the other way around!";

    if(!mIsInitialized) initialize();

    // Make sure the VBO layouts are compatible.
    Q_ASSERT(pointCloudSource->getRenderInfo()->at(0)->layoutMatches(mRenderInfoList[0]));

    if(!CudaHelper::isDeviceSupported)
    {
        qDebug() << __PRETTY_FUNCTION__ << mName << "device not supported, not transforming points to sparse cloud.";
        return;
    }

    QTime time; time.start();

    const bool haveToMapPointPosOfPointCloudSource = pointCloudSource->checkAndMapPointsToCuda();
    Q_ASSERT(pointCloudSource->mDevicePointPos != nullptr);
    float *devicePointsBaseSrc = pointCloudSource->mDevicePointPos;

    qDebug() << __PRETTY_FUNCTION__ << mName << "pcd src is mapped, now mapping my gl buffer...";

    const bool haveToMapPointPos = checkAndMapPointsToCuda();

    // By default, copy all points from pointCloudSource
    if(numberOfPointsToCopy == 0)
    {
        numberOfPointsToCopy = pointCloudSource->getNumberOfPointsStored();
        qDebug() << __PRETTY_FUNCTION__ << mName << "numberOfPointsToCopy is 0, copying all" << numberOfPointsToCopy << "points from" << pointCloudSource->mName;
    }

    const quint32 numberOfPointsBeforeCopy = getNumberOfPointsStored();

    quint32 numberOfPointsProcessedInSrc = 0;
    // We only fill our cloud up to 95% capacity. Otherwise, we'd have maaaany iterations filling it completely, then reducing to 99.99%, refilling, 99.991%, ...
    qDebug() << __PRETTY_FUNCTION__ << mName << "current occupancy:" << (float)getNumberOfPointsStored() / (float)mParameters.capacity << ": there are" << numberOfPointsToCopy << "to insert";
    while(mParameters.elementCount < mParameters.capacity * 0.98f && numberOfPointsProcessedInSrc < pointCloudSource->getNumberOfPointsStored() - firstPointToReadFromSrc && numberOfPointsProcessedInSrc < numberOfPointsToCopy)
    {
        const quint32 freeSpaceInDst = mParameters.capacity - mParameters.elementCount - mParameters.elementQueueCount;

        const quint32 numberOfPointsToCopyInThisIteration = std::min(
                    std::min(
                        freeSpaceInDst, // space left in dst
                        pointCloudSource->getNumberOfPointsStored() - numberOfPointsProcessedInSrc - firstPointToReadFromSrc), // points left in src
                    numberOfPointsToCopy - numberOfPointsProcessedInSrc // number of points left to copy
                    );

        qDebug() << __PRETTY_FUNCTION__ << mName << numberOfPointsProcessedInSrc << "points inserted from source, space left in dst:" << freeSpaceInDst << "- inserting" << numberOfPointsToCopyInThisIteration << "points from src.";

        if(mOutlierTreatment == OutlierTreatment::Remove)
        {
            qDebug() << __PRETTY_FUNCTION__ << mName << "PointCloudCuda::slotInsertPoints(): mAcceptPointsOutsideBoundingBox is false, copying only points in bbox.";
            mParameters.elementQueueCount += copyPointsInBoundingBox(
                        mDevicePointPos + (getNumberOfPointsStored() * mRenderInfoList[0]->elementSize),
                        devicePointsBaseSrc + ((firstPointToReadFromSrc + numberOfPointsProcessedInSrc) * mRenderInfoList[0]->elementSize),
                        mParameters.grid.worldMin,
                        mParameters.grid.worldMax,
                        numberOfPointsToCopyInThisIteration);

        }
        else
        {
            qDebug() << __PRETTY_FUNCTION__ << mName << "mAcceptPointsOutsideBoundingBox is true, copying all given points.";
            mParameters.elementQueueCount += copyPoints(
                        mDevicePointPos + ((numberOfPointsProcessedInSrc) * sizeof(float) * mRenderInfoList[0]->elementSize),
                        devicePointsBaseSrc + ((firstPointToReadFromSrc + numberOfPointsProcessedInSrc) * sizeof(float) * mRenderInfoList[0]->elementSize),
                        numberOfPointsToCopyInThisIteration);
        }

        qDebug() << __PRETTY_FUNCTION__ << mName << "copying done, now reducing...";
        slotReduce();
        qDebug() << __PRETTY_FUNCTION__ << mName << "after reducing points, cloud contains" << mParameters.elementCount << "of" << mParameters.capacity << "points";

        numberOfPointsProcessedInSrc += numberOfPointsToCopyInThisIteration;
    }

    mRenderInfoList[0]->size = getNumberOfPointsStored();

    if(numberOfPointsProcessedInSrc < numberOfPointsToCopy)
        qDebug() << "PointCloudCuda::slotInsertPoints(): didn't insert all points from src (due to insufficient destination capacity of" << mParameters.capacity << "?)";

    if(haveToMapPointPos) checkAndUnmapPointsFromCuda();

    if(haveToMapPointPosOfPointCloudSource) pointCloudSource->checkAndUnmapPointsFromCuda();

    qDebug() << "PointCloudCuda::slotInsertPoints(): took" << time.elapsed() << "ms.";

    emit pointsInserted(this, numberOfPointsBeforeCopy, getNumberOfPointsStored() - numberOfPointsBeforeCopy);
    emit parameters(&mParameters);
}

/*
 * This function reduces all queued points against themselves AND then reduces all points against
 * each other.
 * After processing, it marks the reduced, queued points as existing points, so calling it multiple
 * times doesn't do anything helpful.
 * The reduction is stable, meaning that points that are NOT deleted will stay in the same order.
 */

void PointCloudCuda::slotReduce()
{
    if(!CudaHelper::isDeviceSupported)
    {
//        qDebug() << "PointCloudCuda::slotReduce(): device not supported, not reducing points.";
        return;
    }

    qDebug() << __PRETTY_FUNCTION__ << "cloud" << mName << "will reduce" << mParameters.elementQueueCount << "queued points";

    if(!mIsInitialized) initialize();

    if(mGridHasChanged) initializeGrid();

    // We want to remove all points that have been appended and have close neighbors in the pre-existing data.
    // When appended points have close neighbors in other appended points, we want to delete just one of both.

    const bool haveToMapPointPos = checkAndMapPointsToCuda();

    QTime time; // for profiling

    // 1) - reduce the queued points
    if(mParameters.elementQueueCount)
    {
        time.start();
        float *devicePointsQueued = mDevicePointPos + (mParameters.elementCount * 4);
        const quint32 numberOfQueuedPointsRemaining = reducePoints(devicePointsQueued, mParameters.elementQueueCount);
        qDebug() << __PRETTY_FUNCTION__ << mName << "1) - reducing" << mParameters.elementQueueCount << "to" << numberOfQueuedPointsRemaining << "points took" << time.elapsed() << "ms";

        // We have now reduced the queued points. Send them to other point clouds interested in the new points. First, we need to update
        // elementQueueCount, as the receiving pointcloud might want to inquire about our queue.
        mParameters.elementQueueCount = numberOfQueuedPointsRemaining;
        emit pointsInserted(this, mParameters.elementCount, mParameters.elementQueueCount);

        // Now append the remaining queued points
        mParameters.elementCount += numberOfQueuedPointsRemaining;
        mParameters.elementQueueCount = 0;
    }

    // 2) - reduce all points (including the already reduced, previously queued points)
    time.start();
    const quint32 numberOfPointsRemaining = reducePoints(mDevicePointPos, mParameters.elementCount);
    qDebug() << __PRETTY_FUNCTION__ << mName << "2) - reducing" << mParameters.elementCount << "to" << numberOfPointsRemaining << "points took" << time.elapsed() << "ms";
    mParameters.elementCount = numberOfPointsRemaining;

    // Unmap at end here to avoid unnecessary graphics/CUDA context switch.
    // Once unmapped, the resource may not be accessed by CUDA until they
    // are mapped again. This function provides the synchronization guarantee
    // that any CUDA work issued  before ::cudaGraphicsUnmapResources()
    // will complete before any subsequently issued graphics work begins.
    if(haveToMapPointPos) checkAndUnmapPointsFromCuda();

    mRenderInfoList[0]->size = getNumberOfPointsStored();

    emit parameters(&mParameters);
}

quint32 PointCloudCuda::reducePoints(float* devicePoints, quint32 numElements)
{
    if(numElements == 0) return 0;

    qDebug() << __PRETTY_FUNCTION__ << mName << "reducing" << numElements << "points, outlierTreatment" << static_cast<quint8>(mOutlierTreatment);

    cudaThreadSynchronize(); // dbg

    QTime t; t.start();

    if(mOutlierTreatment == OutlierTreatment::GrowBoundingBox)
    {
        // Determine bounding-box of given points
        float3 bBoxMin, bBoxMax;
        getBoundingBox(devicePoints, numElements, bBoxMin, bBoxMax);

        qDebug() << __PRETTY_FUNCTION__ << mName << "there are" << numElements << "points with a bbox from"
                 << bBoxMin.x << bBoxMin.y << bBoxMin.z << "to" << bBoxMax.x << bBoxMax.y << bBoxMax.z << "using grid with" << mParameters.grid.cells.x << mParameters.grid.cells.y << mParameters.grid.cells.z << "cells";

        // Define the new bounding box for all queued points
        mParameters.grid.worldMin = bBoxMin;
        mParameters.grid.worldMax = bBoxMax;
    }
    else if(mOutlierTreatment == OutlierTreatment::Remove)
    {
        numElements = removePointsOutsideBoundingBox(devicePoints, numElements, &mParameters.grid);
        qDebug() << __PRETTY_FUNCTION__ << mName << "removing points outside the bounding box"
                 << mParameters.grid.worldMin.x << mParameters.grid.worldMin.y << mParameters.grid.worldMin.z
                 << "to"
                 << mParameters.grid.worldMax.x << mParameters.grid.worldMax.y << mParameters.grid.worldMax.z
                 << "has left us with" << numElements << "points";
    }

    cudaThreadSynchronize();qDebug() << __PRETTY_FUNCTION__ << t.elapsed() << "ms after outlier treatment";

    copyParametersToGpu(&mParameters);

    ParametersPointCloud* paramsOnGpu;
    getDeviceAddressOfParametersPointCloud(&paramsOnGpu);

    // Build grid mapping grid cell to particle id
    computeMappingFromPointToGridCell(
            mDeviceMapGridCell,
            mDeviceMapPointIndex,
            devicePoints,
            &paramsOnGpu->grid,
            numElements);

    cudaThreadSynchronize();qDebug() << __PRETTY_FUNCTION__ << t.elapsed() << "ms after creating mapping from point to gridcell";

    // Sort this mapping according to grid cell
    sortMapAccordingToKeys(mDeviceMapGridCell, mDeviceMapPointIndex, numElements);

    cudaThreadSynchronize();qDebug() << __PRETTY_FUNCTION__ << t.elapsed() << "ms after sorting map according to keys";

    sortParticlePosAndVelAccordingToGridCellAndFillCellStartAndEndArrays(
                mDeviceCellStart,               // output: At which index in mDeviceMapParticleIndex does cell X start?
                mDeviceCellStopp,               // output: At which index in mDeviceMapParticleIndex does cell X end?
                mDevicePointSortedPos,          // output: The particle positions, sorted by gridcell
                0,                              // output: The particle velocities, sorted by gridcell
                mDeviceMapGridCell,             // input:  The key - part of the particle gridcell->index map, sorted
                mDeviceMapPointIndex,           // input:  The value-part of the particle gridcell->index map, sorted
                devicePoints,                   // input:  The particle-positions, unsorted
                0,                              // input:  The particle-velocities, unsorted
                numElements,                    // input:  The number of particles
                mParameters.grid.getCellCount() // input:  Number of grid cells
                );

    cudaThreadSynchronize();qDebug() << __PRETTY_FUNCTION__ << t.elapsed() << "ms after creating start and end arrays";

    // Mark redundant points by colliding with themselves using mParameters.minimumDistance
    markCollidingPoints(
                devicePoints,
                mDevicePointSortedPos,
                mDeviceMapPointIndex,
                mDeviceCellStart,
                mDeviceCellStopp,
                numElements);

    cudaThreadSynchronize();qDebug() << __PRETTY_FUNCTION__ << t.elapsed() << "ms after marking colliding points";

    // Remove all points with values 0/0/0/0
    const quint32 remainingElements = removeClearedPoints(devicePoints, numElements);

//    cudaThreadSynchronize();qDebug() << __PRETTY_FUNCTION__ << t.elapsed() << "ms after removing zero-points of" << numElements;

    size_t memTotal, memFree;
    cudaMemGetInfo(&memFree, &memTotal);
    qDebug() << __PRETTY_FUNCTION__ << mName << "device has" << memFree / 1048576 << "of" << memTotal / 1048576 << "mb free";

    return remainingElements;
}

void PointCloudCuda::setBoundingBox(const Box3D& box)
{
    qDebug() << __PRETTY_FUNCTION__ << mName << box;
    mParameters.grid.worldMin = make_float3(box.min.x(), box.min.y(), box.min.z());
    mParameters.grid.worldMax = make_float3(box.max.x(), box.max.y(), box.max.z());

    // remove points that now lie outside, if mOutlierTreatment indicates that
    if(mIsInitialized && getNumberOfPointsStored() && mOutlierTreatment == OutlierTreatment::Remove)
    {
        const bool haveToMapPointPos = checkAndMapPointsToCuda();
        mParameters.elementCount = removePointsOutsideBoundingBox(mDevicePointPos, getNumberOfPointsStored(), &mParameters.grid);
        qDebug() << __PRETTY_FUNCTION__ << mName << "removing points outside the bounding box" << box << "has left us with" << mParameters.elementCount << "points";
        mParameters.elementQueueCount = 0;
        if(haveToMapPointPos) checkAndUnmapPointsFromCuda();
    }
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
    qDebug() << __PRETTY_FUNCTION__ << mName;
    if(!mIsInitialized) initialize();

    mParameters.elementQueueCount = 0;
    mParameters.elementCount = 0;
    mRenderInfoList[0]->size = 0;
}

bool PointCloudCuda::importFromFile(const QString& fileName, QWidget* widget)
{
    if(fileName.toLower().contains(".ply"))
    {
        PointCloudReaderPly pcr(fileName);
        if(!pcr.open())
        {
            if(widget != nullptr) QMessageBox::critical(widget, "Error reading file", QString("Could not open file %1 for reading.").arg(fileName));
            return false;
        }

        quint32 numPointsToLoad = pcr.getNumberOfPoints();
        numPointsToLoad = qMin(mParameters.capacity - mParameters.elementQueueCount - mParameters.elementCount, numPointsToLoad);

        float* points = new float[4*numPointsToLoad];
        pcr.readPoints(points, numPointsToLoad, widget);
        slotInsertPoints4(points, numPointsToLoad);
        delete points;

        return true;
    }

    QMessageBox::critical(widget, "Error reading file", QString("I do not know the type of file %1.").arg(fileName));
    return false;
}

bool PointCloudCuda::exportToFile(const QString& fileName, QWidget *widget)
{
    if(fileName.toLower().contains(".ply"))
    {
        PointCloudWriterPly pcw(fileName);
        pcw.setDataFormat(DataFormatBinaryLittleEndian);
        if(!pcw.open())
        {
            if(widget != nullptr) QMessageBox::critical(widget, "Error writing file", QString("Could not open file %1 for writing.").arg(fileName));
            return false;
        }

        for(int i=0;i < mRenderInfoList.size();i++)
        {
            glBindBuffer(GL_ARRAY_BUFFER, mRenderInfoList[0]->vbo);

            float* points = (float*)glMapBuffer(GL_ARRAY_BUFFER, GL_READ_ONLY);

            pcw.writePoints(points, mRenderInfoList.at(i)->size, widget);

            glUnmapBuffer(GL_ARRAY_BUFFER);

            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }

        return true;
    }

    QMessageBox::critical(widget, "Error reading file", QString("I do not know the type of file %1.").arg(fileName));
    return false;
}

bool PointCloudCuda::checkAndMapPointsToCuda()
{
    // Simply make sure that the pointer used for the resource is mapped.
    // Return true if it had to be mapped, false otherwise.
    if(mDevicePointPos == nullptr)
    {
        //qDebug() << __PRETTY_FUNCTION__ << mName << "mapping into cuda space";
        mDevicePointPos = (float*) CudaHelper::mapGLBufferObject(getCudaGraphicsResource());
        return true;
    }
    else
    {
        return false;
    }
}

bool PointCloudCuda::checkAndUnmapPointsFromCuda()
{
    // Simply make sure that the pointer used for the resource is unmapped.
    // Return true if it had to be unmapped, false otherwise.
    if(mDevicePointPos == nullptr)
    {
        return false;
    }
    else
    {
        //qDebug() << __PRETTY_FUNCTION__ << mName << "unmapping out of cuda space";
        cudaGraphicsUnmapResources(1, getCudaGraphicsResource(), 0);
        mDevicePointPos = nullptr;
        return true;
    }

}
