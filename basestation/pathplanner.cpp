#include "pathplanner.h"

#include <cuda_runtime_api.h>
#include "cudahelper.h"
#include "cudahelper.cuh"

#include <QtConcurrent/QtConcurrentRun>

PathPlanner::PathPlanner(QObject *parent) :
    QObject(parent)
{
    mPointCloudColliders = 0;

    mPointCloudCollidersChanged = true;

    mParametersPathPlanner->grid.cells.x = mParametersPathPlanner->grid.cells.y = mParametersPathPlanner->grid.cells.z = 64;
}

PathPlanner::~PathPlanner()
{
    cudaSafeCall(cudaFree(mDeviceOccupancyGrid));

    cudaSafeCall(cudaGraphicsUnregisterResource(mCudaVboResourceColliderPositions));
}
void PathPlanner::slotInitialize()
{
    Q_ASSERT(mPointCloudColliders != 0);

    const quint32 numberOfCells = mParametersPathPlanner->grid.cells.x * mParametersPathPlanner->grid.cells.y * mParametersPathPlanner->grid.cells.z;

    cudaSafeCall(cudaMalloc((void**)&mDeviceOccupancyGrid, numberOfCells * sizeof(char)));

    cudaSafeCall(cudaStreamCreate(&mCudaStream));

    cudaSafeCall(cudaGraphicsGLRegisterBuffer(&mCudaVboResourceColliderPositions, mPointCloudColliders->getVboInfo()[0].vbo, cudaGraphicsMapFlagsNone));

}


void PathPlanner::slotSetPointCloudColliders(PointCloudCuda* const pcd)
{
    mPointCloudColliders = pcd;
    connect(mPointCloudColliders, SIGNAL(pointsInserted(PointCloud*const,quint32,quint32)), SLOT(slotColliderCloudInsertedPoints()));
}

void PathPlanner::slotColliderCloudInsertedPoints()
{
    mPointCloudCollidersChanged = true;
}

void PathPlanner::slotRequestPath(const QVector3D& start, const QVector3D& goal)
{
    mParametersPathPlanner->start = CudaHelper::cudaConvert(start);
    mParametersPathPlanner->goal = CudaHelper::cudaConvert(goal);

    if(mFuture.isRunning())
    {
        qDebug() << __PRETTY_FUNCTION__ << ": still computing previous request, cancelling...";
        mCancelComputation = true;
        mFuture.waitForFinished();
        qDebug() << __PRETTY_FUNCTION__ << ": done cancelling.";
        mCancelComputation = false;
    }

    mFuture = QtConcurrent::run(this, &PathPlanner::computePathOnGpu);
}

void PathPlanner::computePathOnGpu()
{
    // TODO: use pinned host memory for asynchronous transfers!

    // mDeviceOccupancyGrid points to device memory filled with grid-values of quint8.
    // After fillOccupancyGrid(), empty cells contain a 0, occupied cells contain a 255.
    // The cell containing the start is set to 1. Then, one thread is launched per cell,
    // looking into the neighboring cells. If a neighboring cell (26 3d-neighbors) contains
    // a value other than 0 or 255, the current cell is set to min(neighbor)+1.
    // This is executed often, so that all cells reachable from start get filled with the
    // distance TO start

    copyParametersToGpu(mParametersPathPlanner);

    // Get a pointer to the particle positions in the device by mapping GPU mem into CPU mem
    float *colliderPos = (float*)CudaHelper::mapGLBufferObject(&mCudaVboResourceColliderPositions);

    // Only re-create the occupancy grid if the collider cloud changed
    if(mPointCloudCollidersChanged)
    {
        fillOccupancyGrid(mDeviceOccupancyGrid, colliderPos, mPointCloudColliders->getNumberOfPoints(), mParametersPathPlanner->grid.getCellCount(), &mCudaStream);
        mPointCloudCollidersChanged = false;
    }

    // Now that the occupancy grid is filled, start path planning.


//    cudaMemcpyAsync( dev1, host1, size,H2D,mCudaStream);
//    kernel2 <<< grid, block, 0,mCudaStream>>> ( ..., dev2, ... ) ;
//    kernel3 <<< grid, block, 0,mCudaStream>>> ( ..., dev3, ... ) ;
//    cudaMemcpyAsync( host4, dev4, size,D2H,mCudaStream);


    // Unmap at end here to avoid unnecessary graphics/CUDA context switch.
    // Once unmapped, the resource may not be accessed by CUDA until they
    // are mapped again. This function provides the synchronization guarantee
    // that any CUDA work issued  before ::cudaGraphicsUnmapResources()
    // will complete before any subsequently issued graphics work begins.
    cudaGraphicsUnmapResources(1, &mCudaVboResourceColliderPositions, 0);
}
