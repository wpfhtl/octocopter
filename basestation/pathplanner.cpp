#include "pathplanner.h"

#include <cuda_runtime_api.h>
#include <openglutilities.h>
#include "cudahelper.h"
#include "cudahelper.cuh"

#include <iostream>

#include <QtConcurrent/QtConcurrentRun>

PathPlanner::PathPlanner(QObject *parent) :
    QObject(parent)
{
    mPointCloudColliders = 0;

    mRepopulateOccupanccyGrid = true;

    mParametersPathPlanner.grid.cells.x = 32;
    mParametersPathPlanner.grid.cells.y = 32;
    mParametersPathPlanner.grid.cells.z = 32;

    mHostWaypoints = 0;
}

PathPlanner::~PathPlanner()
{
    OpenGlUtilities::deleteVbo(mVboGridOccupancy);
    OpenGlUtilities::deleteVbo(mVboGridPathFinder);
    cudaSafeCall(cudaFree(mDeviceWaypoints));
    delete mHostWaypoints;
    delete mHostOccupancyGrid;
}

void PathPlanner::slotInitialize()
{
    Q_ASSERT(mPointCloudColliders != 0);

    const quint32 numberOfCells = mParametersPathPlanner.grid.cells.x * mParametersPathPlanner.grid.cells.y * mParametersPathPlanner.grid.cells.z;

    mHostOccupancyGrid = new quint8[numberOfCells];

    // The occupancy grid will be built and dilated in this memory
    mVboGridOccupancy = OpenGlUtilities::createVbo(numberOfCells * sizeof(quint8));
    cudaSafeCall(cudaGraphicsGLRegisterBuffer(&mCudaVboResourceGridOccupancy, mVboGridOccupancy, cudaGraphicsMapFlagsNone));

    // The (dilated) occupancy grid will be copied in here, then the pathplanner fill it. Separate memories
    // enable re-use of the pre-built occupancy grid.
    mVboGridPathFinder = OpenGlUtilities::createVbo(numberOfCells * sizeof(quint8));
    cudaSafeCall(cudaGraphicsGLRegisterBuffer(&mCudaVboResourceGridPathFinder, mVboGridPathFinder, cudaGraphicsMapFlagsNone));

    cudaSafeCall(cudaMalloc((void**)&mDeviceWaypoints, mMaxWaypoints * 4 * sizeof(float)));

    cudaSafeCall(cudaStreamCreate(&mCudaStream));

    mHostWaypoints = new float[mMaxWaypoints * 4];

    emit vboInfoGridOccupancy(
                mVboGridOccupancy,
                QVector3D(mParametersPathPlanner.grid.worldMin.x, mParametersPathPlanner.grid.worldMin.y, mParametersPathPlanner.grid.worldMin.z),
                QVector3D(mParametersPathPlanner.grid.worldMax.x, mParametersPathPlanner.grid.worldMax.y, mParametersPathPlanner.grid.worldMax.z),
                Vector3i(mParametersPathPlanner.grid.cells.x, mParametersPathPlanner.grid.cells.y, mParametersPathPlanner.grid.cells.z)
                );

    emit vboInfoGridPathFinder(
                mVboGridPathFinder,
                QVector3D(mParametersPathPlanner.grid.worldMin.x, mParametersPathPlanner.grid.worldMin.y, mParametersPathPlanner.grid.worldMin.z),
                QVector3D(mParametersPathPlanner.grid.worldMax.x, mParametersPathPlanner.grid.worldMax.y, mParametersPathPlanner.grid.worldMax.z),
                Vector3i(mParametersPathPlanner.grid.cells.x, mParametersPathPlanner.grid.cells.y, mParametersPathPlanner.grid.cells.z)
                );
}

void PathPlanner::slotSetPointCloudColliders(PointCloudCuda* const pcd)
{
    mPointCloudColliders = pcd;
    connect(mPointCloudColliders, SIGNAL(pointsInserted(PointCloud*const,quint32,quint32)), SLOT(slotColliderCloudInsertedPoints()));
}

void PathPlanner::slotColliderCloudInsertedPoints()
{
    mRepopulateOccupanccyGrid = true;
}

void PathPlanner::slotRequestPath(const QVector3D& start, const QVector3D& goal)
{
    mParametersPathPlanner.start = CudaHelper::cudaConvert(start);
    mParametersPathPlanner.goal = CudaHelper::cudaConvert(goal + QVector3D(0,3,0));

    qDebug() << __PRETTY_FUNCTION__ << "computing path from" << CudaHelper::cudaConvert(mParametersPathPlanner.start) << "to" << CudaHelper::cudaConvert(mParametersPathPlanner.goal);

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

    QTime t;t.start();

    // If the collider cloud's grid changed, copy the grid dimensions (not resolution) and repopulate
    if(! mParametersPathPlanner.grid.hasSameExtents(mPointCloudColliders->getGrid()))
    {
        qDebug() << __PRETTY_FUNCTION__ << "following collider cloud from current"
                 << CudaHelper::cudaConvert(mParametersPathPlanner.grid.worldMin)
                 << CudaHelper::cudaConvert(mParametersPathPlanner.grid.worldMin)
                 << "to"
                 << CudaHelper::cudaConvert(mPointCloudColliders->getGrid().worldMin)
                 << CudaHelper::cudaConvert(mPointCloudColliders->getGrid().worldMax);

        mParametersPathPlanner.grid.worldMin = mPointCloudColliders->getGrid().worldMin;
        mParametersPathPlanner.grid.worldMax = mPointCloudColliders->getGrid().worldMax;
        mRepopulateOccupanccyGrid = true;

        emit vboInfoGridOccupancy(
                    mVboGridOccupancy,
                    QVector3D(mParametersPathPlanner.grid.worldMin.x, mParametersPathPlanner.grid.worldMin.y, mParametersPathPlanner.grid.worldMin.z),
                    QVector3D(mParametersPathPlanner.grid.worldMax.x, mParametersPathPlanner.grid.worldMax.y, mParametersPathPlanner.grid.worldMax.z),
                    Vector3i(mParametersPathPlanner.grid.cells.x, mParametersPathPlanner.grid.cells.y, mParametersPathPlanner.grid.cells.z)
                    );

        emit vboInfoGridPathFinder(
                    mVboGridPathFinder,
                    QVector3D(mParametersPathPlanner.grid.worldMin.x, mParametersPathPlanner.grid.worldMin.y, mParametersPathPlanner.grid.worldMin.z),
                    QVector3D(mParametersPathPlanner.grid.worldMax.x, mParametersPathPlanner.grid.worldMax.y, mParametersPathPlanner.grid.worldMax.z),
                    Vector3i(mParametersPathPlanner.grid.cells.x, mParametersPathPlanner.grid.cells.y, mParametersPathPlanner.grid.cells.z)
                    );
    }

    copyParametersToGpu(&mParametersPathPlanner);

    quint8* gridOccupancy = (quint8*)CudaHelper::mapGLBufferObject(&mCudaVboResourceGridOccupancy);
    quint8* gridPathFinder = (quint8*)CudaHelper::mapGLBufferObject(&mCudaVboResourceGridPathFinder);
    // Only re-create the occupancy grid if the collider cloud's content changed
    if(mRepopulateOccupanccyGrid)
    {
        // Get a pointer to the particle positions in the device by mapping GPU mem into CPU mem
        float *colliderPos = (float*)CudaHelper::mapGLBufferObject(mPointCloudColliders->getCudaGraphicsResource());
        fillOccupancyGrid(gridOccupancy, colliderPos, mPointCloudColliders->getNumberOfPoints(), mParametersPathPlanner.grid.getCellCount(), &mCudaStream);
        cudaGraphicsUnmapResources(1, mPointCloudColliders->getCudaGraphicsResource(), 0);

        mRepopulateOccupanccyGrid = false;

        qDebug() << "grid after filling:";
        printHostOccupancyGrid(gridOccupancy);

        dilateOccupancyGrid(
                    gridOccupancy,
                    mParametersPathPlanner.grid.getCellCount(),
                    &mCudaStream);

        qDebug() << "grid after dilation:";
        printHostOccupancyGrid(gridOccupancy);
    }

//    qDebug() << "grid after init:";
//    printHostOccupancyGrid(grid);

    // Copy the populated and dilated occupancy grid into the PathFinder's domain
    cudaMemcpy(gridPathFinder, gridOccupancy, mParametersPathPlanner.grid.getCellCount(), cudaMemcpyDeviceToDevice);
    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridOccupancy, 0);

    // Now that the occupancy grid is filled, start path planning.
    computePath(
                gridPathFinder,
                mParametersPathPlanner.grid.getCellCount(),
                mDeviceWaypoints,
                &mCudaStream);

    qDebug() << "grid after computePath:";
    printHostOccupancyGrid(gridPathFinder);

    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridPathFinder, 0);

    qDebug() << mMaxWaypoints;
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
            //     1               2             3
            //     *---------------* - - - - - - *
            //
            //     ^ mComputedPath ^             ^ newWayPoint

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

    qDebug() << __PRETTY_FUNCTION__ << "took" << t.elapsed() << "ms.";
}

void PathPlanner::printHostOccupancyGrid(quint8* deviceGridOccupancy)
{
    return;
    cudaMemcpy(mHostOccupancyGrid, deviceGridOccupancy, mParametersPathPlanner.grid.getCellCount(), cudaMemcpyDeviceToHost);

//    qDebug() << __PRETTY_FUNCTION__;

    int3 c;
    QString separatorLine = QString().rightJustified(5 + 4*mParametersPathPlanner.grid.cells.x, '=');
    QString header = "      x:";
    for(int i=0;i<mParametersPathPlanner.grid.cells.x;i++)
    {
        QString part = QString::number(i).leftJustified(4, ' ');
        header.append(part);
    }

    for(int y = 0;y<mParametersPathPlanner.grid.cells.y;y++)
    {
        qDebug() << __PRETTY_FUNCTION__ << "layer y" << y <<":";
        printf("%s\n", qPrintable(header));
        printf("%s\n", qPrintable(separatorLine));
        for(int z = 0;z<mParametersPathPlanner.grid.cells.z;z++)
        {
            printf("z%3d| ", z);
            fflush(stdout);
            for(int x = 0;x<mParametersPathPlanner.grid.cells.x;x++)
            {
                c.x = x; c.y = y; c.z = z;
                quint8 value = mHostOccupancyGrid[mParametersPathPlanner.grid.getCellHash(c)];
                printf("%3d ", value);
                fflush(stdout);
            }
            printf("\n");
            fflush(stdout);
        }
    }
    printf("\n");
    fflush(stdout);
}
