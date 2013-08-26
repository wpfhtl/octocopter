#include "pathplanner.h"

//#include <cuda_runtime_api.h>
#include <openglutilities.h>
//#include "cudahelper.h"
//#include "cudahelper.cuh"

PathPlanner::PathPlanner(QObject *parent) :
    QObject(parent)
{
    mPointCloudColliders = nullptr;

    mRepopulateOccupanccyGrid = true;

    mParametersPathPlanner.grid.cells.x = 32;
    mParametersPathPlanner.grid.cells.y = 32;
    mParametersPathPlanner.grid.cells.z = 32;

}

PathPlanner::~PathPlanner()
{
    OpenGlUtilities::deleteVbo(mVboGridOccupancy);
    OpenGlUtilities::deleteVbo(mVboGridPathFinder);
    cudaSafeCall(cudaFree(mDeviceWaypoints));
}

void PathPlanner::slotInitialize()
{
    Q_ASSERT(mPointCloudColliders != nullptr);

    const quint32 numberOfCells = mParametersPathPlanner.grid.cells.x * mParametersPathPlanner.grid.cells.y * mParametersPathPlanner.grid.cells.z;

//    mHostOccupancyGrid = new quint8[numberOfCells];

    // The occupancy grid will be built and dilated in this memory
    mVboGridOccupancy = OpenGlUtilities::createVbo(numberOfCells * sizeof(quint8));
    cudaSafeCall(cudaGraphicsGLRegisterBuffer(&mCudaVboResourceGridOccupancy, mVboGridOccupancy, cudaGraphicsMapFlagsNone));

    // The (dilated) occupancy grid will be copied in here, then the pathplanner fill it. Separate memories
    // enable re-use of the pre-built occupancy grid.
    mVboGridPathFinder = OpenGlUtilities::createVbo(numberOfCells * sizeof(quint8));
    cudaSafeCall(cudaGraphicsGLRegisterBuffer(&mCudaVboResourceGridPathFinder, mVboGridPathFinder, cudaGraphicsMapFlagsNone));

    cudaSafeCall(cudaMalloc((void**)&mDeviceWaypoints, mMaxWaypoints * 4 * sizeof(float)));

    cudaSafeCall(cudaStreamCreate(&mCudaStream));


    emit vboInfoGridOccupancy(
                mVboGridOccupancy,
                Box3D(
                    QVector3D(mParametersPathPlanner.grid.worldMin.x, mParametersPathPlanner.grid.worldMin.y, mParametersPathPlanner.grid.worldMin.z),
                    QVector3D(mParametersPathPlanner.grid.worldMax.x, mParametersPathPlanner.grid.worldMax.y, mParametersPathPlanner.grid.worldMax.z)
                    ),
                Vector3i(mParametersPathPlanner.grid.cells.x, mParametersPathPlanner.grid.cells.y, mParametersPathPlanner.grid.cells.z)
                );

    emit vboInfoGridPathFinder(
                mVboGridPathFinder,
                Box3D(
                    QVector3D(mParametersPathPlanner.grid.worldMin.x, mParametersPathPlanner.grid.worldMin.y, mParametersPathPlanner.grid.worldMin.z),
                    QVector3D(mParametersPathPlanner.grid.worldMax.x, mParametersPathPlanner.grid.worldMax.y, mParametersPathPlanner.grid.worldMax.z)
                    ),
                Vector3i(mParametersPathPlanner.grid.cells.x, mParametersPathPlanner.grid.cells.y, mParametersPathPlanner.grid.cells.z)
                );
}

void PathPlanner::slotSetPointCloudColliders(PointCloudCuda* const pcd)
{
    mPointCloudColliders = pcd;
    connect(mPointCloudColliders, &PointCloudCuda::pointsInserted, this, &PathPlanner::slotColliderCloudInsertedPoints);
}

void PathPlanner::slotColliderCloudInsertedPoints()
{
    mRepopulateOccupanccyGrid = true;
}

void PathPlanner::populateOccupancyGrid(quint8* gridOccupancy)
{
    if(mRepopulateOccupanccyGrid == false)
    {
        qDebug() << __PRETTY_FUNCTION__ << "occupancy grid is still up to date, returning.";
        return;
    }

    // We can be given a pointer to the device's mapped VBO. If not, we map and unmap it ourselves.
    quint8* gridOccupancyLocal;

    if(gridOccupancy == nullptr)
        gridOccupancyLocal = (quint8*)CudaHelper::mapGLBufferObject(&mCudaVboResourceGridOccupancy);
    else
        gridOccupancyLocal = gridOccupancy;

    qDebug() << __PRETTY_FUNCTION__ << "updating occupancy grid...";

    // Get a pointer to the particle positions in the device by mapping GPU mem into CUDA address space
    float *colliderPos = (float*)CudaHelper::mapGLBufferObject(mPointCloudColliders->getCudaGraphicsResource());
    fillOccupancyGrid(gridOccupancyLocal, colliderPos, mPointCloudColliders->getNumberOfPoints(), mParametersPathPlanner.grid.getCellCount(), &mCudaStream);
    cudaGraphicsUnmapResources(1, mPointCloudColliders->getCudaGraphicsResource(), 0);

    dilateOccupancyGrid(
                gridOccupancyLocal,
                mParametersPathPlanner.grid.getCellCount(),
                &mCudaStream);

    if(gridOccupancy == nullptr)
        cudaGraphicsUnmapResources(1, &mCudaVboResourceGridOccupancy, 0);

    mRepopulateOccupanccyGrid = false;
}

void PathPlanner::checkWayPointSafety(const QVector3D& vehiclePosition, const WayPointList* const wayPointsAhead)
{
    // The collider cloud inserted some points. Let's check whether our waypoint-cells are now occupied. In that case, we'd have to re-plan!
    qDebug() << "PathPlanner::checkWayPointSafety(): checking safety of" << wayPointsAhead->size() << "waypoints...";
    if(wayPointsAhead->size())
    {
        copyParametersToGpu(&mParametersPathPlanner);

        // Make some room for waypoints in host memory. The first float4's x=y=z=w will store just the number of waypoints
        float* waypointsHost = new float[wayPointsAhead->size() * 4];

        // We want to check wayPointsAhead, so we need to copy it into GPU memory space.
        for(int i=0;i<wayPointsAhead->size();i++)
        {
            waypointsHost[4*i+0] = wayPointsAhead->at(i).x();
            waypointsHost[4*i+1] = wayPointsAhead->at(i).y();
            waypointsHost[4*i+2] = wayPointsAhead->at(i).z();
            waypointsHost[4*i+3] = 0.0f;
        }

        cudaMemcpy(
                    (void*)mDeviceWaypoints,
                    (void*)waypointsHost,
                    4 * wayPointsAhead->size() * sizeof(float),
                    cudaMemcpyHostToDevice);

        quint8* gridOccupancy = (quint8*)CudaHelper::mapGLBufferObject(&mCudaVboResourceGridOccupancy);
        populateOccupancyGrid(gridOccupancy);
        testWayPointCellOccupancy(gridOccupancy, mDeviceWaypoints, wayPointsAhead->size(), &mCudaStream);
        cudaGraphicsUnmapResources(1, &mCudaVboResourceGridOccupancy, 0);

        // Now copy the waypoints back from device into host memory and see if the w-components are non-zero.
        cudaMemcpy(
                    (void*)waypointsHost,
                    (void*)mDeviceWaypoints,
                    4 * wayPointsAhead->size() * sizeof(float),
                    cudaMemcpyDeviceToHost);

        // Check the next N waypoints for collisions with ALL of the collider cloud!
        const quint32 numberOfUpcomingWayPointsToCheck = 5;

        QVector<quint32> occupiedWayPoints;

        // Check all future-waypoint-w-components.
        for(int i=0; i<numberOfUpcomingWayPointsToCheck && i < wayPointsAhead->size()-1; i++)
        {
            if(waypointsHost[4*i+3] > 0.0f)
            {
                // Oh my, waypoint i is now unreachable! Extract the important (information gain) waypoints and
                // re-plan a path through all of them. If the occupied waypoint is a InformationGain-WayPoint,
                // then remove it.
                occupiedWayPoints.append(i);

                const QVector4D wpt(waypointsHost[4*i+0], waypointsHost[4*i+1], waypointsHost[4*i+2], waypointsHost[4*i+3]);
                qDebug() << __PRETTY_FUNCTION__ << "waypoint" << i << "at" << wpt.x() << wpt.y() << wpt.z() << "collides with point cloud!";
            }
        }

        delete waypointsHost;

        // If we found occupied waypoints, compute a new path using only the information-gain waypoints.
        if(occupiedWayPoints.size())
        {
            emit message(LogImportance::Warning, "PathPlanner", "waypoints are now colliding with geometry. Stopping rover and replanning path.");

            // wayPointsAhead is a pointer to flightplanner's real list. As soon as we emit an empty path,
            // it will also become empty! So, create a copy.
            WayPointList wayPointsWithHighInformationGain(*wayPointsAhead);

            // Create an empty list to emit an empty path, so we stop the rover!
            QList<WayPoint> wpl;
            emit path(&wpl, WayPointListSource::WayPointListSourceFlightPlanner);

            for(int i=wayPointsWithHighInformationGain.size()-1;i>=0;i--)
            {
                const WayPoint wpt = wayPointsWithHighInformationGain.at(i);

                // If this waypoint has information gain AND is not one of the occupied ones, re-use it!
                if(wpt.informationGain <= 0.0f || occupiedWayPoints.contains(i))
                    wayPointsWithHighInformationGain.remove(i);
            }

            // compute a new path through the same waypoints
            if(wayPointsWithHighInformationGain.size())
            {
                slotComputePath(vehiclePosition, wayPointsWithHighInformationGain);
            }
            else
            {
                // There are no waypoints left to compute a path. Generate new waypoints!
                qDebug() << "PathPlanner::checkWayPointSafety(): no waypoitns left to compute path!";
                emit generateNewWayPoints();
            }
        }
    }
}

// This is not part of slotComputePath() because we want to moveToSafety, travellingSalesMan, computePath in that order!
// So, these methods are called by FlightPlanner
void PathPlanner::moveWayPointsToSafety(WayPointList* wayPointList)
{
    int numberOfWayPoints = wayPointList->size();

    // The collider cloud inserted some points. Let's check whether our waypoint-cells are now occupied. In that case, we'd have to re-plan!
    qDebug() << __PRETTY_FUNCTION__ << "moving" << numberOfWayPoints << "waypoints to safe cells...";
    if(numberOfWayPoints)
    {
        alignPathPlannerGridToColliderCloud();
        copyParametersToGpu(&mParametersPathPlanner);

        // Make some room for waypoints in host memory. The first float4's x=y=z=w will store just the number of waypoints
        float* waypointsHost = new float[numberOfWayPoints * 4];

        // We want to check wayPointList, so we need to copy it into GPU memory space.
        for(int i=0;i<numberOfWayPoints;i++)
        {
            waypointsHost[4*i+0] = wayPointList->at(i).x();
            waypointsHost[4*i+1] = wayPointList->at(i).y();
            waypointsHost[4*i+2] = wayPointList->at(i).z();
            waypointsHost[4*i+3] = wayPointList->at(i).informationGain;
        }

        wayPointList->clear();

        cudaMemcpy(
                    (void*)mDeviceWaypoints,
                    (void*)waypointsHost,
                    4 * numberOfWayPoints * sizeof(float),
                    cudaMemcpyHostToDevice);

        quint8* gridOccupancy = (quint8*)CudaHelper::mapGLBufferObject(&mCudaVboResourceGridOccupancy);
        populateOccupancyGrid(gridOccupancy);
        moveWayPointsToSafetyGpu(gridOccupancy, mDeviceWaypoints, numberOfWayPoints, &mCudaStream);
        cudaGraphicsUnmapResources(1, &mCudaVboResourceGridOccupancy, 0);

        // Now copy the waypoints back from device into host memory and see if the w-components are non-zero.
        cudaMemcpy(
                    (void*)waypointsHost,
                    (void*)mDeviceWaypoints,
                    4 * numberOfWayPoints * sizeof(float),
                    cudaMemcpyDeviceToHost);

        // Check all future-waypoint-w-components.
        for(int i=0;i<numberOfWayPoints-1; i++)
        {
            if(waypointsHost[4*i+3] > 0.0f)
            {
                wayPointList->append(
                            WayPoint(
                                QVector3D(waypointsHost[4*i+0], waypointsHost[4*i+1], waypointsHost[4*i+2]),
                                waypointsHost[4*i+3]
                            ));
            }
        }

        delete waypointsHost;
    }
}

void PathPlanner::alignPathPlannerGridToColliderCloud()
{
    // If the collider cloud's grid changed, copy the grid dimensions (not resolution) and repopulate
    if(!mParametersPathPlanner.grid.hasSameExtents(mPointCloudColliders->getGrid()))
    {
        qDebug() << __PRETTY_FUNCTION__ << "following collider cloud from current"
                 << CudaHelper::convert(mParametersPathPlanner.grid.worldMin)
                 << CudaHelper::convert(mParametersPathPlanner.grid.worldMin)
                 << "to"
                 << CudaHelper::convert(mPointCloudColliders->getGrid().worldMin)
                 << CudaHelper::convert(mPointCloudColliders->getGrid().worldMax);

        mParametersPathPlanner.grid.worldMin = mPointCloudColliders->getGrid().worldMin;
        mParametersPathPlanner.grid.worldMax = mPointCloudColliders->getGrid().worldMax;
        mRepopulateOccupanccyGrid = true;

        emit vboInfoGridOccupancy(
                    mVboGridOccupancy,
                    Box3D(
                        QVector3D(mParametersPathPlanner.grid.worldMin.x, mParametersPathPlanner.grid.worldMin.y, mParametersPathPlanner.grid.worldMin.z),
                        QVector3D(mParametersPathPlanner.grid.worldMax.x, mParametersPathPlanner.grid.worldMax.y, mParametersPathPlanner.grid.worldMax.z)
                        ),
                    Vector3i(mParametersPathPlanner.grid.cells.x, mParametersPathPlanner.grid.cells.y, mParametersPathPlanner.grid.cells.z)
                    );

        emit vboInfoGridPathFinder(
                    mVboGridPathFinder,
                    Box3D(
                        QVector3D(mParametersPathPlanner.grid.worldMin.x, mParametersPathPlanner.grid.worldMin.y, mParametersPathPlanner.grid.worldMin.z),
                        QVector3D(mParametersPathPlanner.grid.worldMax.x, mParametersPathPlanner.grid.worldMax.y, mParametersPathPlanner.grid.worldMax.z)
                        ),
                    Vector3i(mParametersPathPlanner.grid.cells.x, mParametersPathPlanner.grid.cells.y, mParametersPathPlanner.grid.cells.z)
                    );
    }
}

void PathPlanner::slotComputePath(const QVector3D& vehiclePosition, const WayPointList& wayPointList)
{
    Q_ASSERT(!wayPointList.isEmpty());

    WayPointList wayPointsWithHighInformationGain = wayPointList;
    wayPointsWithHighInformationGain.prepend(WayPoint(vehiclePosition, 0));

    qDebug() << __PRETTY_FUNCTION__ << "computing path for waypointlist" << wayPointsWithHighInformationGain.toString();

    // mDeviceOccupancyGrid points to device memory filled with grid-values of quint8.
    // After fillOccupancyGrid(), empty cells contain a 0, occupied cells contain a 255.
    // The cell containing the start is set to 1. Then, one thread is launched per cell,
    // looking into the neighboring cells. If a neighboring cell (26 3d-neighbors) contains
    // a value other than 0 or 255, the current cell is set to min(neighbor)+1.
    // This is executed often, so that all cells reachable from start get filled with the
    // distance TO start

    alignPathPlannerGridToColliderCloud();

    QTime t;t.start();

    WayPointList computedPath;

    copyParametersToGpu(&mParametersPathPlanner);

    quint8* gridOccupancy = (quint8*)CudaHelper::mapGLBufferObject(&mCudaVboResourceGridOccupancy);
    quint8* gridPathFinder = (quint8*)CudaHelper::mapGLBufferObject(&mCudaVboResourceGridPathFinder);

    // Only re-creates the occupancy grid if the collider cloud's content changed
    populateOccupancyGrid(gridOccupancy);

    // The first path leads from vehicle position to first waypoint. Clear the occupancy grid above the vehicle!
    // This is currently necessary, because we also scan bernd and the fishing rod, occupying our own cells.
    clearOccupancyGridAboveVehiclePosition(
                gridPathFinder,
                vehiclePosition.x(),
                vehiclePosition.y(),
                vehiclePosition.z(),
                &mCudaStream);

    // Make some room for waypoints in host memory. The first float4's x=y=z=w will store just the number of waypoints
    float* waypointsHost = new float[mMaxWaypoints * 4];

    // Find a path between every pair of waypoints
    quint32 indexWayPointStart = 0;
    quint32 indexWayPointGoal = 1;
    quint32 pathNumber = 0;
    do
    {
        mParametersPathPlanner.start = CudaHelper::convert(wayPointsWithHighInformationGain.at(indexWayPointStart));
        mParametersPathPlanner.goal = CudaHelper::convert(wayPointsWithHighInformationGain.at(indexWayPointGoal));

        qDebug() << __PRETTY_FUNCTION__ << "now computing path from" << indexWayPointStart << ":" << wayPointsWithHighInformationGain.at(indexWayPointStart).toString() << "to" << indexWayPointGoal << ":" << wayPointsWithHighInformationGain.at(indexWayPointGoal).toString();

        copyParametersToGpu(&mParametersPathPlanner);

        // Copy the populated and dilated occupancy grid into the PathFinder's domain
        cudaMemcpy(gridPathFinder, gridOccupancy, mParametersPathPlanner.grid.getCellCount(), cudaMemcpyDeviceToDevice);

        // Now start path planning.
        markStartCell(gridPathFinder, &mCudaStream);

        growGrid(
                    gridPathFinder,
                    &mParametersPathPlanner,
                    &mCudaStream);

        retrievePath(
                    gridPathFinder,
                    mDeviceWaypoints,
                    &mCudaStream);

        cudaMemcpy(
                    (void*)waypointsHost,
                    (void*)mDeviceWaypoints,
                    4 * mMaxWaypoints * sizeof(float),
                    cudaMemcpyDeviceToHost);

        if(fabs(waypointsHost[0]) < 0.001 && fabs(waypointsHost[1]) < 0.001 && fabs(waypointsHost[2]) < 0.001)
        {
            qDebug() << __PRETTY_FUNCTION__ << "found NO path from" << indexWayPointStart << ":" << wayPointsWithHighInformationGain.at(indexWayPointStart).toString() << "to" << indexWayPointGoal << ":" << wayPointsWithHighInformationGain.at(indexWayPointGoal).toString();
            // When no path was found, we try to find a path to the next waypoint, skipping the problematic one.
            indexWayPointGoal++;
        }
        else
        {
            qDebug() << __PRETTY_FUNCTION__ << "found path with" << waypointsHost[0] << "waypoints";
            // The first waypoint isn't one, it only contains the number of waypoints
            for(int i=1;i<=waypointsHost[0];i++)
            {
                WayPoint newWayPoint;

                if(i == 1)
                {
                    newWayPoint = wayPointsWithHighInformationGain.at(indexWayPointStart);
                }
                else if(i == (int)waypointsHost[0])
                {
                    newWayPoint = wayPointsWithHighInformationGain.at(indexWayPointGoal);
                }
                else
                {
                    newWayPoint = WayPoint(QVector3D(waypointsHost[4*i+0], waypointsHost[4*i+1], waypointsHost[4*i+2]), 0);
                }

                // Append all points of the first path, and then only starting at the second point of the following paths.
                // Otherwise, we have the end of path A and the beginning of path B in the list, although they're the same.
                if(pathNumber == 0 || i > 1)
                {
                    computedPath.append(newWayPoint);
                }
            }

            qDebug() << __PRETTY_FUNCTION__ << "found path between" << wayPointsWithHighInformationGain.at(indexWayPointStart).toString() << "and" << wayPointsWithHighInformationGain.at(indexWayPointGoal).toString() << ":" << computedPath.toString();

            pathNumber++;
            indexWayPointStart = indexWayPointGoal;
            indexWayPointGoal++;
        }
    } while(indexWayPointGoal < wayPointsWithHighInformationGain.size());

    delete waypointsHost;

    emit path(computedPath.list(), WayPointListSource::WayPointListSourceFlightPlanner);

    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridPathFinder, 0);
    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridOccupancy, 0);

    qDebug() << __PRETTY_FUNCTION__ << "took" << t.elapsed() << "ms.";
}
