#include "pathplanner.h"
#include <openglutilities.h>

PathPlanner::PathPlanner(PointCloudCuda* const pointCloudColliders, QObject *parent) : QObject(parent)
{
    mIsInitialized = false;

    mRepopulateOccupanccyGrid = true;

    mGridOccupancyTemplate = nullptr;
    mGridOccupancyPathPanner = nullptr;

    mDeviceWaypoints = nullptr;

    mParametersPathPlanner.grid.cells.x = mParametersPathPlanner.grid.cells.y = mParametersPathPlanner.grid.cells.z = 32;

    mPointCloudColliders = pointCloudColliders;
    connect(mPointCloudColliders, &PointCloudCuda::pointsInserted, this, &PathPlanner::slotColliderCloudInsertedPoints);
}

PathPlanner::~PathPlanner()
{
    OpenGlUtilities::deleteVbo(mVboGridOccupancy);
    OpenGlUtilities::deleteVbo(mVboGridPathPlanner);
    cudaSafeCall(cudaFree(mDeviceWaypoints));
}

void PathPlanner::initialize()
{
    qDebug() << __PRETTY_FUNCTION__;

    Q_ASSERT(!mIsInitialized);

    Q_ASSERT(mPointCloudColliders != nullptr);

    const quint32 numberOfCells = mParametersPathPlanner.grid.getCellCount();

    // The occupancy grid will be built and dilated in this memory
    mVboGridOccupancy = OpenGlUtilities::createVbo(numberOfCells * sizeof(quint8));
    cudaSafeCall(cudaGraphicsGLRegisterBuffer(&mCudaVboResourceGridOccupancyTemplate, mVboGridOccupancy, cudaGraphicsMapFlagsNone));

    checkAndMapGridOccupancy(mCudaVboResourceGridOccupancyTemplate);
    cudaMemset(mGridOccupancyTemplate, 0, numberOfCells);
    checkAndUnmapGridOccupancy(mCudaVboResourceGridOccupancyTemplate);

    // The (dilated) occupancy grid will be copied in here, then the pathplanner fills it. Separate memories
    // enable re-use of the pre-built occupancy grid.
    mVboGridPathPlanner = OpenGlUtilities::createVbo(numberOfCells * sizeof(quint8));
    cudaSafeCall(cudaGraphicsGLRegisterBuffer(&mCudaVboResourceGridPathPlanner, mVboGridPathPlanner, cudaGraphicsMapFlagsNone));

    cudaSafeCall(cudaMalloc((void**)&mDeviceWaypoints, mMaxWaypoints * 4 * sizeof(float)));

    cudaSafeCall(cudaStreamCreate(&mCudaStream));

    alignPathPlannerGridToColliderCloud();

    mIsInitialized = true;
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
                    Vector3<quint16>(mParametersPathPlanner.grid.cells.x, mParametersPathPlanner.grid.cells.y, mParametersPathPlanner.grid.cells.z)
                    );

        emit vboInfoGridPathPlanner(
                    mVboGridPathPlanner,
                    Box3D(
                        QVector3D(mParametersPathPlanner.grid.worldMin.x, mParametersPathPlanner.grid.worldMin.y, mParametersPathPlanner.grid.worldMin.z),
                        QVector3D(mParametersPathPlanner.grid.worldMax.x, mParametersPathPlanner.grid.worldMax.y, mParametersPathPlanner.grid.worldMax.z)
                        ),
                    Vector3<quint16>(mParametersPathPlanner.grid.cells.x, mParametersPathPlanner.grid.cells.y, mParametersPathPlanner.grid.cells.z)
                    );
    }
}

void PathPlanner::slotColliderCloudInsertedPoints()
{
    mRepopulateOccupanccyGrid = true;
}

void PathPlanner::populateOccupancyGrid()
{
    if(mRepopulateOccupanccyGrid == false)
    {
        qDebug() << __PRETTY_FUNCTION__ << "occupancy grid is still up to date, returning.";
        return;
    }

    copyParametersToGpu(&mParametersPathPlanner);

    const bool haveToMapGridOccupancyTemplate = checkAndMapGridOccupancy(mCudaVboResourceGridOccupancyTemplate);

    qDebug() << __PRETTY_FUNCTION__ << "filling occupancy grid...";

    // Get a pointer to the particle positions in the device by mapping GPU mem into CUDA address space

    const bool haveToMapPointPos = mPointCloudColliders->checkAndMapPointsToCuda();
    const float *colliderPos = mPointCloudColliders->getPointsInCudaSpace();
    fillOccupancyGrid(mGridOccupancyTemplate, colliderPos, mPointCloudColliders->getNumberOfPointsStored(), mParametersPathPlanner.grid.getCellCount(), &mCudaStream);
    if(haveToMapPointPos) mPointCloudColliders->checkAndUnmapPointsFromCuda();

    qDebug() << __PRETTY_FUNCTION__ << "dilating occupancy grid...";
    dilateOccupancyGrid(
                mGridOccupancyTemplate,
                mParametersPathPlanner.grid.getCellCount(),
                &mCudaStream);

    if(haveToMapGridOccupancyTemplate) checkAndUnmapGridOccupancy(mCudaVboResourceGridOccupancyTemplate);

    mRepopulateOccupanccyGrid = false;

    qDebug() << __PRETTY_FUNCTION__ << "done.";
}

qint32 PathPlanner::checkWayPointSafety(const WayPointList* const wayPointsAhead)
{
    if(!mIsInitialized) initialize();

    // The collider cloud inserted some points. Let's check whether our waypoint-cells are now occupied. In that case, we'd have to re-plan!
    qDebug() << "PathPlanner::checkWayPointSafety(): checking safety of" << wayPointsAhead->size() << "waypoints...";

    if(wayPointsAhead->size() == 0) return -1;

    copyParametersToGpu(&mParametersPathPlanner);

    // Make some room for waypoints in host memory.
    float* waypointsHost = new float[wayPointsAhead->size() * 4];

    // We want to check wayPointsAhead, so we need to copy it into GPU memory space.
    for(int i=0;i<wayPointsAhead->size();i++)
    {
        waypointsHost[4*i+0] = wayPointsAhead->at(i).x();
        waypointsHost[4*i+1] = wayPointsAhead->at(i).y();
        waypointsHost[4*i+2] = wayPointsAhead->at(i).z();
        waypointsHost[4*i+3] = wayPointsAhead->at(i).informationGain;
    }

    cudaSafeCall(cudaMemcpy(
                     (void*)mDeviceWaypoints,
                     (void*)waypointsHost,
                     4 * wayPointsAhead->size() * sizeof(float),
                     cudaMemcpyHostToDevice));

    const bool haveToMapGridOccupancyTemplate = checkAndMapGridOccupancy(mCudaVboResourceGridOccupancyTemplate);
    populateOccupancyGrid();
    testWayPointCellOccupancy(mGridOccupancyTemplate, mDeviceWaypoints, wayPointsAhead->size(), &mCudaStream);
    if(haveToMapGridOccupancyTemplate) checkAndUnmapGridOccupancy(mCudaVboResourceGridOccupancyTemplate);

    // Now copy the waypoints back from device into host memory and see if the w-components are non-zero.
    cudaSafeCall(cudaMemcpy(
                     (void*)waypointsHost,
                     (void*)mDeviceWaypoints,
                     4 * wayPointsAhead->size() * sizeof(float),
                     cudaMemcpyDeviceToHost));

    // Check future-waypoint-w-components. They are now assigned the value from the grid,
    // meaning that 0 is free, 254 occupied-dilated and 255 occupied.
    for(int i=0; i < wayPointsAhead->size()-1; i++)
    {
        if(waypointsHost[4*i+3] != 0.0f)
        {
            qDebug() << __PRETTY_FUNCTION__ << "next-up-waypoint" << i << "at" << waypointsHost[4*i+0] << waypointsHost[4*i+1] << waypointsHost[4*i+2] << "collides with point cloud, grid-value is" << waypointsHost[4*i+3] << "returning false!";
            delete waypointsHost;
            return i;
        }
    }

    qDebug() << __PRETTY_FUNCTION__ << "all" << wayPointsAhead->size() << "waypoints are safe!";
    delete waypointsHost;
    return -1;
}

// This is not part of slotComputePath() because we want to moveToSafety, travellingSalesMan, computePath in that order!
// So, these methods are called by FlightPlanner. Pass 0 as last parameter to start searching in the waypoint's cells or
// a higher number to start searching above. When first moving waypoints fresh from the information gain grid, we want to
// search 1 or 2 cells above for sufficient ground clearance. Later-on, when just moving those waypoints that collide, we
// do not really want to raise them again. That's what the last parameter is made for.
void PathPlanner::moveWayPointsToSafety(WayPointList* wayPointList, bool raiseAllWaypointsForGroundClearance)
{
    // The collider cloud inserted some points. Let's check whether our waypoint-cells are now occupied. In that case, we'd have to re-plan!
    if(!mIsInitialized) initialize();

    const int numberOfWayPointsToCheck = wayPointList->size();

    // If desired, raise waypoints by 4m
    int startSearchNumberOfCellsAbove = raiseAllWaypointsForGroundClearance ? (4.0 / mParametersPathPlanner.grid.getCellSize().y) : 0;

    qDebug() << __PRETTY_FUNCTION__ << "raising" << numberOfWayPointsToCheck << "waypoints by" << startSearchNumberOfCellsAbove << "cells, then moving to safe cells if required.";

    if(numberOfWayPointsToCheck)
    {
        alignPathPlannerGridToColliderCloud();
        copyParametersToGpu(&mParametersPathPlanner);

        // Make some room for waypoints in host memory. The first float4's x=y=z=w will store just the number of waypoints
        float* waypointsHost = new float[numberOfWayPointsToCheck * 4];

        // We want to check wayPointList, so we need to copy it into GPU memory space.
        for(int i=0;i<numberOfWayPointsToCheck;i++)
        {
            waypointsHost[4*i+0] = wayPointList->at(i).x();
            waypointsHost[4*i+1] = wayPointList->at(i).y();
            waypointsHost[4*i+2] = wayPointList->at(i).z();
            waypointsHost[4*i+3] = wayPointList->at(i).informationGain;
        }

        wayPointList->clear();

        cudaSafeCall(cudaMemcpy(
                         (void*)mDeviceWaypoints,
                         (void*)waypointsHost,
                         4 * numberOfWayPointsToCheck * sizeof(float),
                         cudaMemcpyHostToDevice));


        const bool haveToMapGridOccupancyTemplate = checkAndMapGridOccupancy(mCudaVboResourceGridOccupancyTemplate);
        populateOccupancyGrid();
        moveWayPointsToSafetyGpu(mGridOccupancyTemplate, mDeviceWaypoints, numberOfWayPointsToCheck, startSearchNumberOfCellsAbove, &mCudaStream);
        if(haveToMapGridOccupancyTemplate) checkAndUnmapGridOccupancy(mCudaVboResourceGridOccupancyTemplate);

        // Now copy the waypoints back from device into host memory and see if the w-components are non-zero.
        cudaSafeCall(cudaMemcpy(
                         (void*)waypointsHost,
                         (void*)mDeviceWaypoints,
                         4 * numberOfWayPointsToCheck * sizeof(float),
                         cudaMemcpyDeviceToHost));

        // Check all future-waypoint-w-components.
        for(int i=0;i<numberOfWayPointsToCheck; i++)
        {
            const WayPoint wpt(QVector3D(waypointsHost[4*i+0], waypointsHost[4*i+1], waypointsHost[4*i+2]), waypointsHost[4*i+3]);

            if(wpt.informationGain > 0.0f)
            {
                wayPointList->append(wpt);
            }
            else
            {
                qDebug() << __PRETTY_FUNCTION__ << "waypoint" << i << ":" << wpt << "couldn't be saved, removing it.";
            }
        }

        delete waypointsHost;
    }
    qDebug() << __PRETTY_FUNCTION__ << "returning list with" << wayPointList->size() << "of" << numberOfWayPointsToCheck << "waypoints left.";
}

void PathPlanner::slotComputePath(const QVector3D& vehiclePosition, const WayPointList& wayPointList)
{
    Q_ASSERT(!wayPointList.isEmpty());

    if(!mIsInitialized) initialize();

    WayPointList wayPointsWithHighInformationGain = wayPointList;
    wayPointsWithHighInformationGain.prepend(WayPoint(vehiclePosition, 0));

    qDebug() << __PRETTY_FUNCTION__ << "computing path for waypointlist" << wayPointsWithHighInformationGain.toString();

    // mDeviceOccupancyGrid points to device memory filled with grid-values of quint8.
    // After fillOccupancyGrid(), empty cells contain a 0, occupied cells contain a 254/255.
    // The cell containing the start is set to 1. Then, one thread is launched per cell,
    // looking into the neighboring cells. If a neighboring cell (26 3d-neighbors) contains
    // a value other than 0 or 254/255, the current cell is set to min(neighbor)+1.
    // This is executed often, so that all cells reachable from start get filled with the
    // distance TO start

    alignPathPlannerGridToColliderCloud();

    QTime t;t.start();

    WayPointList computedPath;

    copyParametersToGpu(&mParametersPathPlanner);

    const bool haveToMapGridOccupancyTemplate = checkAndMapGridOccupancy(mCudaVboResourceGridOccupancyTemplate);
    const bool haveToMapGridOccupancyPathPlanner = checkAndMapGridOccupancy(mCudaVboResourceGridPathPlanner);

    // Only re-creates the occupancy grid if the collider cloud's content changed
    populateOccupancyGrid();

    // The first path leads from vehicle position to first waypoint. Clear the occupancy grid above the vehicle!
    // This is currently necessary, because we also scan bernd and the fishing rod, occupying our own cells.
    clearOccupancyGridAboveVehiclePosition(
                mGridOccupancyPathPanner,
                vehiclePosition.x(),
                vehiclePosition.y(),
                vehiclePosition.z(),
                &mCudaStream);

    // We have freed the occupancy grid a little to make path planning easier/possible. Restore it to the real grid asap.
    mRepopulateOccupanccyGrid = true;

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
        cudaSafeCall(cudaMemcpy(
                         mGridOccupancyPathPanner,
                         mGridOccupancyTemplate,
                         mParametersPathPlanner.grid.getCellCount(),
                         cudaMemcpyDeviceToDevice));

        // Now start path planning.
        markStartCell(mGridOccupancyPathPanner, &mCudaStream);

        growGrid(
                    mGridOccupancyPathPanner,
                    &mParametersPathPlanner,
                    &mCudaStream);

        // We must set the waypoints-array on the device to a special value because
        // a bug can lead to some waypoints not being written. This is ok
        // as long as we can detect this. When memsetting with 0, we can,
        // otherwise we'd find a waypoint from a previous run and couldn't
        // detect this incidence.
        cudaSafeCall(cudaMemset(
                         (void*)mDeviceWaypoints,
                         255, // interpreted as float, should be NaN!
                         4 * mMaxWaypoints * sizeof(float)));

        retrievePath(
                    mGridOccupancyPathPanner,
                    mDeviceWaypoints,
                    &mCudaStream);

        cudaSafeCall(cudaMemcpy(
                         (void*)waypointsHost,
                         (void*)mDeviceWaypoints,
                         4 * mMaxWaypoints * sizeof(float),
                         cudaMemcpyDeviceToHost));

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
                // workaround for the no-waypoint-bug, which will show values of NaN/NaN/NaN/NaN due to the 255-memset above
                if(isnan(waypointsHost[4*i+3]))
                {
                    qDebug() << __PRETTY_FUNCTION__ << "ignoring waypoint that was skpped in retrievePath due to bug in growGrid.";
                    continue;
                }

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

    if(haveToMapGridOccupancyTemplate) checkAndUnmapGridOccupancy(mCudaVboResourceGridOccupancyTemplate);
    //    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridPathFinder, 0);

    if(haveToMapGridOccupancyPathPlanner) checkAndUnmapGridOccupancy(mCudaVboResourceGridPathPlanner);
    //    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridOccupancy, 0);

    emit path(computedPath.list(), WayPointListSource::WayPointListSourceFlightPlanner);

    qDebug() << __PRETTY_FUNCTION__ << "took" << t.elapsed() << "ms.";
}

bool PathPlanner::checkAndMapGridOccupancy(cudaGraphicsResource *resource)
{
    // Simply make sure that the pointer used for this resource is mapped.
    // Return true if it had to be mapped, false otherwise.
    if(resource == mCudaVboResourceGridOccupancyTemplate)
    {
        if(mGridOccupancyTemplate == nullptr)
        {
            qDebug() << __PRETTY_FUNCTION__ << "mapping grid occupancy template into cuda space";
            mGridOccupancyTemplate = (quint8*)CudaHelper::mapGLBufferObject(&resource);
            return true;
        }
        else
        {
            return false;
        }
    }

    if(resource == mCudaVboResourceGridPathPlanner)
    {
        if(mGridOccupancyPathPanner == nullptr)
        {
            qDebug() << __PRETTY_FUNCTION__ << "mapping grid occupancy pathplanner into cuda space";
            mGridOccupancyPathPanner = (quint8*)CudaHelper::mapGLBufferObject(&resource);
            return true;
        }
        else
        {
            return false;
        }
    }
}

bool PathPlanner::checkAndUnmapGridOccupancy(cudaGraphicsResource *resource)
{
    // Simply make sure that the pointer used for this resource is unmapped.
    // Return true if it had to be unmapped, false otherwise.
    if(resource == mCudaVboResourceGridOccupancyTemplate)
    {
        if(mGridOccupancyTemplate == nullptr)
        {
            return false;
        }
        else
        {
            //qDebug() << __PRETTY_FUNCTION__ << "mapping grid occupancy template out of cuda space";
            cudaGraphicsUnmapResources(1, &resource, 0);
            mGridOccupancyTemplate = nullptr;
            return true;
        }
    }

    if(resource == mCudaVboResourceGridPathPlanner)
    {
        if(mGridOccupancyPathPanner == nullptr)
        {
            return false;
        }
        else
        {
            //qDebug() << __PRETTY_FUNCTION__ << "mapping grid occupancy pathplanner out of cuda space";
            cudaGraphicsUnmapResources(1, &resource, 0);
            mGridOccupancyPathPanner = nullptr;
            return true;
        }
    }
}
