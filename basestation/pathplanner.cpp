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

    mParametersPathPlanner.grid.cells.x = mParametersPathPlanner.grid.cells.y = mParametersPathPlanner.grid.cells.z = 64;

    mHostWaypoints = 0;
}

PathPlanner::~PathPlanner()
{
    cudaSafeCall(cudaFree(mDeviceOccupancyGrid));
    cudaSafeCall(cudaFree(mDeviceWaypoints));
    delete mHostWaypoints;
}

void PathPlanner::slotInitialize()
{
    Q_ASSERT(mPointCloudColliders != 0);

    const quint32 numberOfCells = mParametersPathPlanner.grid.cells.x * mParametersPathPlanner.grid.cells.y * mParametersPathPlanner.grid.cells.z;

    cudaSafeCall(cudaMalloc((void**)&mDeviceOccupancyGrid, numberOfCells * sizeof(char)));
    cudaSafeCall(cudaMalloc((void**)&mDeviceWaypoints, mMaxWaypoints * 4 * sizeof(float)));

    cudaSafeCall(cudaStreamCreate(&mCudaStream));

    mHostWaypoints = new float[mMaxWaypoints * 4];
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
    mParametersPathPlanner.start = CudaHelper::cudaConvert(start);
    mParametersPathPlanner.goal = CudaHelper::cudaConvert(goal);

    /* threading is hard, as we have no control over mapping the VBO for grid population. I tmight be used for rendering in another thread!
    if(mFuture.isRunning())
    {
        qDebug() << __PRETTY_FUNCTION__ << ": still computing previous request, cancelling...";
        mCancelComputation = true;
        mFuture.waitForFinished();
        qDebug() << __PRETTY_FUNCTION__ << ": done cancelling.";
        mCancelComputation = false;
    }

    mFuture = QtConcurrent::run(this, &PathPlanner::slotComputePathOnGpu);
    */

    QTimer::singleShot(0, this, SLOT(slotComputePathOnGpu()));
}

void PathPlanner::slotComputePathOnGpu()
{
    qDebug() << __PRETTY_FUNCTION__ << "now computing path";

    // TODO: use pinned host memory for asynchronous transfers!

    // mDeviceOccupancyGrid points to device memory filled with grid-values of quint8.
    // After fillOccupancyGrid(), empty cells contain a 0, occupied cells contain a 255.
    // The cell containing the start is set to 1. Then, one thread is launched per cell,
    // looking into the neighboring cells. If a neighboring cell (26 3d-neighbors) contains
    // a value other than 0 or 255, the current cell is set to min(neighbor)+1.
    // This is executed often, so that all cells reachable from start get filled with the
    // distance TO start

    copyParametersToGpu(&mParametersPathPlanner);

    // Get a pointer to the particle positions in the device by mapping GPU mem into CPU mem
    float *colliderPos = (float*)CudaHelper::mapGLBufferObject(mPointCloudColliders->getCudaGraphicsResource());

    // Only re-create the occupancy grid if the collider cloud changed
    if(mPointCloudCollidersChanged)
    {
        fillOccupancyGrid(mDeviceOccupancyGrid, colliderPos, mPointCloudColliders->getNumberOfPoints(), mParametersPathPlanner.grid.getCellCount(), &mCudaStream);
        mPointCloudCollidersChanged = false;
    }

    // Now that the occupancy grid is filled, start path planning.
    computePath(
                mDeviceOccupancyGrid,
                mParametersPathPlanner.grid.getCellCount(),
                mDeviceWaypoints,
                &mCudaStream);

    cudaMemcpy(
                (void*)mHostWaypoints,
                (void*)mDeviceWaypoints,
                4 * mMaxWaypoints * sizeof(float),
                cudaMemcpyDeviceToHost);

    mComputedPath.clear();

    if(fabs(mHostWaypoints[0]) < 0.001 && fabs(mHostWaypoints[1]) < 0.001 && fabs(mHostWaypoints[2]) < 0.001)
    {
        qDebug() << __PRETTY_FUNCTION__ << "found NO path!";
        emit pathFound(&mComputedPath, WayPointListSource::WayPointListSourceFlightPlanner);
    }
    else
    {
        // The first waypoint isn't one, it only contains the number of waypoints
        for(int i=1;i<=mHostWaypoints[0];i++)
        {
            QVector3D newWayPoint(
                        mHostWaypoints[4*i+0],
                        mHostWaypoints[4*i+1],
                        mHostWaypoints[4*i+2]);

            // If this newWayPoint is colinear to the previous two, we can remove the last point in mComputedPath:
            //
            //     1         2             3
            //     *---------* - - - - - - *
            //
            //     ^ mComputedPath         ^ newWayPoint

            if(mComputedPath.size() >= 2)
            {
                if(newWayPoint.distanceToLine(
                            mComputedPath.at(mComputedPath.size()-2),
                            mComputedPath.at(mComputedPath.size()-1)) < 0.01f)
                {
                    qDebug() << __PRETTY_FUNCTION__ << "removing colinear waypoint!";
                    mComputedPath.takeLast();
                }
            }

            mComputedPath.append(
                        WayPoint(
                            newWayPoint,
                            0,
                            i == mHostWaypoints[0] ? WayPoint::Purpose::SCAN : WayPoint::Purpose::DETOUR));
        }

        qDebug() << __PRETTY_FUNCTION__ << "found path:" << mComputedPath;

        emit pathFound(&mComputedPath, WayPointListSource::WayPointListSourceFlightPlanner);
    }

    // Unmap at end here to avoid unnecessary graphics/CUDA context switch.
    // Once unmapped, the resource may not be accessed by CUDA until they
    // are mapped again. This function provides the synchronization guarantee
    // that any CUDA work issued  before ::cudaGraphicsUnmapResources()
    // will complete before any subsequently issued graphics work begins.
    cudaGraphicsUnmapResources(1, mPointCloudColliders->getCudaGraphicsResource(), 0);
}
