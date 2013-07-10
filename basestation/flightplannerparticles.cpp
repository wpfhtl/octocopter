#include "flightplannerparticles.h"
#include "flightplannerparticlesdialog.h"
#include "particlesystem.cuh"
#include "pointcloudcuda.h"
#include "cudahelper.cuh"
#include "basestation.h"

FlightPlannerParticles::FlightPlannerParticles(BaseStation* baseStation, GlWindow *glWidget, PointCloud *pointcloud) : FlightPlannerInterface(baseStation, glWidget, pointcloud)
{
    mParticleSystem = 0;
    mPathPlanner = 0;
    mParticleRenderer = 0;
    mVboGridMapOfWayPointPressure = 0;
    mDeviceGridMapInformationGainCellWorldPositions = 0;

    mPointCloudColliders = new PointCloudCuda(
                QVector3D(-56.0f, -8.0f, -50.0f),
                QVector3D(8.0f, 26.0f, 14.0f),
                64 * 1024);

    mPointCloudColliders->setMinimumPointDistance(0.3f);

    mPointCloudColliders->setColor(QColor(0,0,255,120));

    mGlWindow->slotPointCloudRegister(mPointCloudColliders);

    mSimulationParameters.initialize();
    mDialog = new FlightPlannerParticlesDialog(&mSimulationParameters, (QWidget*)baseStation);

    mPathPlanner = new PathPlanner;
    connect(mPathPlanner, SIGNAL(pathFound(QList<WayPoint>*const,WayPointListSource)), SLOT(slotSetWayPoints(QList<WayPoint>*const,WayPointListSource)));

    connect(&mTimerProcessInformationGain, SIGNAL(timeout()), SLOT(slotProcessInformationGain()));
}

void FlightPlannerParticles::slotInitialize()
{
    qDebug() << "FlightPlannerParticles::slotInitialize()";

    mSimulationParameters.initialize();
    mSimulationParameters.gridInformationGain.cells = make_uint3(256, 32, 256);
    slotSetScanVolume(QVector3D(-56, -8, -50), QVector3D(8, 26, 14));

    const quint32 numberOfCellsInScanVolume = mSimulationParameters.gridInformationGain.getCellCount();

    // Store the gridcell-waypoint-pressure-values in a VBO as 8bit-unsigned-ints
    mVboGridMapOfWayPointPressure = OpenGlUtilities::createVbo(sizeof(quint8) * numberOfCellsInScanVolume);
    cudaSafeCall(cudaGraphicsGLRegisterBuffer(&mCudaVboResourceGridMapOfWayPointPressure, mVboGridMapOfWayPointPressure, cudaGraphicsMapFlagsNone));
    // Set vbo values to zero
    cudaSafeCall(cudaMemset(CudaHelper::mapGLBufferObject(&mCudaVboResourceGridMapOfWayPointPressure), 0, sizeof(quint8) * numberOfCellsInScanVolume));
    cudaSafeCall(cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfWayPointPressure, 0));

    // Allocate the same vector again for the same values, just sorted and ranked.
    cudaSafeCall(cudaMalloc((void**)&mDeviceGridMapWayPointPressureToBeSorted, sizeof(float) * numberOfCellsInScanVolume));

    // Allocate a vector of float4 for the world-positions of the gridcell positions. These will be the values when sorting
    cudaSafeCall(cudaMalloc((void**)&mDeviceGridMapInformationGainCellWorldPositions, sizeof(float4) * numberOfCellsInScanVolume));

    // Will contain QVector4Ds of worldpos of every gridcell, sorted by waypoint pressure
//    cudaSafeCall(cudaMalloc((void**)&mDeviceGridMapCellWorldPositions, sizeof(float4) * numberOfCellsInScanVolume));

    mPointCloudColliders->slotInitialize();

//    connect(mPointCloudDense, SIGNAL(pointsInserted(PointCloud*const,quint32,quint32)), mPointCloudColliders, SLOT(slotInsertPoints(PointCloud*const,quint32,quint32)));
    connect(mPointCloudDense, SIGNAL(pointsInserted(PointCloud*const,quint32,quint32)), SLOT(slotDenseCloudInsertedPoints(PointCloud*const,quint32,quint32)));

    mShaderProgramGridLines = new ShaderProgram(this, "shader-default-vertex.c", "", "shader-default-fragment.c");

    mParticleSystem = new ParticleSystem((PointCloudCuda*)mPointCloudDense, (PointCloudCuda*)mPointCloudColliders, &mSimulationParameters); // ParticleSystem will draw its points as colliders from the pointcloud passed here
    mParticleRenderer = new ParticleRenderer;

    connect(mParticleSystem, SIGNAL(particleRadiusChanged(float)), mParticleRenderer, SLOT(slotSetParticleRadius(float)));
    connect(mParticleSystem, SIGNAL(vboInfoParticles(quint32,quint32,quint32,QVector3D,QVector3D)), mParticleRenderer, SLOT(slotSetVboInfoParticles(quint32,quint32,quint32,QVector3D,QVector3D)));

    connect(
                mPathPlanner,
                SIGNAL(vboInfoGridOccupancy(quint32,QVector3D,QVector3D,Vector3i)),
                mParticleRenderer,
                SLOT(slotSetVboInfoGridOccupancy(quint32,QVector3D,QVector3D,Vector3i))
                );

    connect(
                mPathPlanner,
                SIGNAL(vboInfoGridPathFinder(quint32,QVector3D,QVector3D,Vector3i)),
                mParticleRenderer,
                SLOT(slotSetVboInfoGridPathFinder(quint32,QVector3D,QVector3D,Vector3i))
                );

    connect(
                this,
                SIGNAL(vboInfoGridInformationGain(quint32,QVector3D,QVector3D,Vector3i)),
                mParticleRenderer,
                SLOT(slotSetVboInfoGridInformationGain(quint32,QVector3D,QVector3D,Vector3i))
                );

    emit vboInfoGridInformationGain(
                mVboGridMapOfWayPointPressure,
                QVector3D(mSimulationParameters.gridInformationGain.worldMin.x, mSimulationParameters.gridInformationGain.worldMin.y, mSimulationParameters.gridInformationGain.worldMin.z),
                QVector3D(mSimulationParameters.gridInformationGain.worldMax.x, mSimulationParameters.gridInformationGain.worldMax.y, mSimulationParameters.gridInformationGain.worldMax.z),
                Vector3i(mSimulationParameters.gridInformationGain.cells.x, mSimulationParameters.gridInformationGain.cells.y, mSimulationParameters.gridInformationGain.cells.z)
                );

    mParticleSystem->slotSetParticleCount(32768);
    mParticleSystem->slotSetParticleRadius(1.0f/2.0f); // balance against mOctreeCollisionObjects.setMinimumPointDistance() above
    mParticleSystem->slotSetDefaultParticlePlacement(ParticleSystem::ParticlePlacement::PlacementFillSky);

    connect(mDialog, &FlightPlannerParticlesDialog::deleteWayPoints, [=](){
        QList<WayPoint> wl;
        slotSetWayPoints(&wl, WayPointListSource::WayPointListSourceFlightPlanner);
        });


    connect(mDialog, SIGNAL(generateWayPoints()), SLOT(slotGenerateWaypoints()));
    connect(mDialog, SIGNAL(resetParticles()), mParticleSystem, SLOT(slotResetParticles()));
    connect(mDialog, SIGNAL(resetInformationGain()), SLOT(slotClearGridWayPointPressure()));
    connect(mDialog, SIGNAL(simulationParameters(const ParametersParticleSystem*)), mParticleSystem, SLOT(slotSetSimulationParametersFromUi(const ParametersParticleSystem*)));

//    connect(mDialog, SIGNAL(processPhysicsChanged(bool)), SLOT(slotProcessPhysics(bool)));
    connect(mDialog, SIGNAL(showParticlesChanged(bool)), mParticleRenderer, SLOT(slotSetRenderParticles(bool)));
    connect(mDialog, SIGNAL(showInformationGainChanged(bool)), mParticleRenderer, SLOT(slotSetRenderInformationGain(bool)));
    connect(mDialog, SIGNAL(showOccupancyGridChanged(bool)), mParticleRenderer, SLOT(slotSetRenderOccupancyGrid(bool)));
    connect(mDialog, SIGNAL(showPathFinderGridChanged(bool)), mParticleRenderer, SLOT(slotSetRenderPathFinderGrid(bool)));
    connect(mDialog, SIGNAL(reduceColliderCloud()), mPointCloudColliders, SLOT(slotReduce()));

    connect(mPointCloudDense, SIGNAL(numberOfPoints(quint32)), mDialog, SLOT(slotSetPointCloudSizeDense(quint32)));
    connect(mPointCloudColliders, SIGNAL(numberOfPoints(quint32)), mDialog, SLOT(slotSetPointCloudSizeSparse(quint32)));

    // PathPlanner needs ColliderCloud to initialize!
    mPathPlanner->slotSetPointCloudColliders(mPointCloudColliders);
    mPathPlanner->slotInitialize();
}

void FlightPlannerParticles::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_F)
    {
        qDebug() << "FlightPlannerParticles::keyPressEvent(): f, toggling collider cloud visualization";
        if(mGlWindow->isPointCloudRegistered(mPointCloudColliders))
            mGlWindow->slotPointCloudUnregister(mPointCloudColliders);
        else
            mGlWindow->slotPointCloudRegister(mPointCloudColliders);
    }

    emit suggestVisualization();
}

void FlightPlannerParticles::slotShowUserInterface()
{
    mDialog->show();
}

void FlightPlannerParticles::slotGenerateWaypoints(quint32 numberOfWaypointsToGenerate)
{
    if(mVboGridMapOfWayPointPressure == 0)
    {
        qDebug() << "FlightPlannerParticles::getRankedWaypoints(): not initialized yet, returning.";
        return;
    }

    const quint32 numberOfCells = mSimulationParameters.gridInformationGain.getCellCount();
    QVector<QVector4D> waypoints(std::min((quint32)200, numberOfCells));

    // Copy waypoint pressure from VBO into mDeviceGridMapWayPointPressureSorted
    quint8* gridMapOfWayPointPressure = (quint8*)CudaHelper::mapGLBufferObject(&mCudaVboResourceGridMapOfWayPointPressure);
    // We should divide a cell's waypointpressure by its distance to the vehicle - this way, we get either close or valuable waypoints!
    QVector3D vehiclePosition;
    if(mVehiclePoses.size()) vehiclePosition = mVehiclePoses.last().getPosition();
    computeWaypointBenefit(mDeviceGridMapWayPointPressureToBeSorted, gridMapOfWayPointPressure, (float*)&vehiclePosition, numberOfCells);
    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfWayPointPressure, 0);

    // Fill mDeviceGridMapCellWorldPositions - this might be done only once and then copied lateron (just like the waypoint pressure above)
    fillGridMapCellWorldPositions(mDeviceGridMapInformationGainCellWorldPositions, numberOfCells);

    // Sort mDeviceGridMapWayPointPressureSorted => mDeviceGridMapCellWorldPositions according to the keys DESC
    sortGridMapWayPointPressure(mDeviceGridMapWayPointPressureToBeSorted, mDeviceGridMapInformationGainCellWorldPositions, numberOfCells, waypoints.size());

    // Copy the @count cells with the highest waypoint pressure into @waypoints
    cudaMemcpy(waypoints.data(), mDeviceGridMapInformationGainCellWorldPositions, sizeof(QVector4D) * waypoints.size(), cudaMemcpyDeviceToHost);

    WayPointList* const wayPointStructure = mWaypointListMap.value("ahead");
    QList<WayPoint>* waypointList = wayPointStructure->list();

    // Do we want to clear the old (but still pending) waypoints?
    waypointList->clear();

    for(int i=0;i<waypoints.size();i++)
    {
        QVector4D wp = waypoints[i];

        // If we ask for more waypoints than cells with wp-pressure > 0.0, we'll get useless candidates!
        if(wp.w() > 0.1f)
            waypointList->append(WayPoint(wp.toVector3D() + QVector3D(0.0f, 4.0f, 0.0f), wp.w(), WayPoint::Purpose::SCAN));
    }

    qDebug() << "FlightPlannerParticles::slotGenerateWaypoints(): generated" << waypointList->size() << "waypoints";

    // Merge waypoints if they're closer than X meters
    wayPointStructure->mergeCloseWaypoints(5.0f);

    // Reduce them to the desired number
    while(waypointList->size() > numberOfWaypointsToGenerate)
        waypointList->removeLast();

    // Lets generate just one for now. Makes pathfinding easier!
    // Sort them to the shortest path, honoring the vehicle's current location.
    //wayPointStructure->sortToShortestPath(getLastKnownVehiclePose().getPosition());

    qDebug() << "FlightPlannerParticles::slotGenerateWaypoints(): after merging, there are" << wayPointStructure->size() << "waypoints left. Now stopping physics processing.";

    // Now that we've generated waypoints, we can deactivate the physics processing (and restart when only few waypoints are left)
    mDialog->setProcessPhysics(false);

    // Now use the PathPlanner to find a path to this next waypoint. It will emit a signal with the list. Hopefully.
    mPathPlanner->slotRequestPath(vehiclePosition, waypointList->first());
}

// If the maximum waypointpressure is higher than threshold, it will generate new waypoints and decrease the pressure.
void FlightPlannerParticles::slotProcessInformationGain(const quint8 threshold)
{
    if(mVboGridMapOfWayPointPressure == 0)
    {
        qDebug() << "FlightPlannerParticles::slotProcessInformationGain(): not initialized yet, returning.";
        return;
    }

    const quint32 numberOfCells = mSimulationParameters.gridInformationGain.getCellCount();

    // Copy waypoint pressure from VBO into mDeviceGridMapWayPointPressureSorted
    quint8* gridMapOfWayPointPressure = (quint8*)CudaHelper::mapGLBufferObject(&mCudaVboResourceGridMapOfWayPointPressure);
    const quint8 maxPressure = getMaximumInformationGain(gridMapOfWayPointPressure, numberOfCells);
    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfWayPointPressure, 0);

    if(maxPressure >= threshold)
    {
        qDebug() << "FlightPlannerParticles::slotProcessInformationGain(): threshold is" << threshold << "- max pressure is" << maxPressure << ": generating waypoints.";
        mTimerProcessInformationGain.stop();
        slotGenerateWaypoints();
    }
}

void FlightPlannerParticles::slotCreateSafePathToNextWayPoint()
{

}

FlightPlannerParticles::~FlightPlannerParticles()
{
    delete mParticleRenderer;
    delete mParticleSystem;

    if(CudaHelper::isDeviceSupported)
    {
        cudaSafeCall(cudaFree(mDeviceGridMapInformationGainCellWorldPositions));
        cudaSafeCall(cudaFree(mDeviceGridMapWayPointPressureToBeSorted));
        cudaSafeCall(cudaGraphicsUnregisterResource(mCudaVboResourceGridMapOfWayPointPressure));
    }

    glDeleteBuffers(1, (const GLuint*)&mVboGridMapOfWayPointPressure);
}


void FlightPlannerParticles::slotNewScanFused(const float* const points, const quint32& count, const QVector3D* const scannerPosition)
{
    // Insert all points into mPointCloudDense
    mPointCloudDense->slotInsertPoints4(points, count);

    emit suggestVisualization();
}

void FlightPlannerParticles::slotVisualize()
{
    FlightPlannerInterface::slotVisualize();

    if(!mParticleSystem && CudaHelper::isDeviceSupported)
        slotInitialize();

    if(mParticleSystem)
    {
        if(mDialog->processPhysics())
        {
            // Provide the particle system a pointer to our waypoint-pressure-gridmap (in device address space)
            quint8 *deviceGridMapOfWayPointPressure = (quint8*)CudaHelper::mapGLBufferObject(&mCudaVboResourceGridMapOfWayPointPressure);

            for(float timeStepped = 0.0f; timeStepped < mSimulationParameters.timeStepOuter; timeStepped += mSimulationParameters.timeStepInner)
            {
                mParticleSystem->update(deviceGridMapOfWayPointPressure);
            }

            cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfWayPointPressure, 0);
        }

        mParticleRenderer->slotSetRenderBoundingBox(mRenderLocalBoundingBox);
        mParticleRenderer->render();
    }

}

void FlightPlannerParticles::slotSetScanVolume(const QVector3D min, const QVector3D max)
{
    qDebug() << __PRETTY_FUNCTION__ << "min" << min << "max" << max;
    // We're being told to change the particle system volume - this is NOT the particle system' world's volume!
    slotClearGridWayPointPressure();

    FlightPlannerInterface::slotSetScanVolume(min, max);

    mSimulationParameters.gridInformationGain.worldMin = CudaHelper::cudaConvert(mScanVolumeMin);
    mSimulationParameters.gridInformationGain.worldMax = CudaHelper::cudaConvert(mScanVolumeMax);

    if(CudaHelper::isDeviceSupported)
    {
        copyParametersToGpu(&mSimulationParameters);

        emit vboInfoGridInformationGain(
                    mVboGridMapOfWayPointPressure,
                    QVector3D(mSimulationParameters.gridInformationGain.cells.x, mSimulationParameters.gridInformationGain.cells.y, mSimulationParameters.gridInformationGain.cells.z),
                    QVector3D(mSimulationParameters.gridInformationGain.cells.x, mSimulationParameters.gridInformationGain.cells.y, mSimulationParameters.gridInformationGain.cells.z),
                    Vector3i(mSimulationParameters.gridInformationGain.cells.x, mSimulationParameters.gridInformationGain.cells.y, mSimulationParameters.gridInformationGain.cells.z)
                    );
    }
}

void FlightPlannerParticles::slotWayPointReached(const WayPoint wpt)
{
    FlightPlannerInterface::slotWayPointReached(wpt);

    // When no waypoints are left, start testing for water-leaks!
    if(mWaypointListMap.value("ahead")->size() == 0)
    {
        qDebug() << "FlightPlannerParticles::slotWayPointReached(): no waypoints left, restarting generation!";

        // This will first copy points to collider (which will reduce), then reduce the dense cloud.
        ((PointCloudCuda*)mPointCloudDense)->slotReduce();

        slotClearGridWayPointPressure();

        mDialog->setProcessPhysics(true);

        const Pose p = mVehiclePoses.last();
        slotVehiclePoseChanged(&p);

        mParticleSystem->slotResetParticles();

        mTimerProcessInformationGain.start(5000);
    }
}

void FlightPlannerParticles::slotVehiclePoseChanged(const Pose* const pose)
{
    FlightPlannerInterface::slotVehiclePoseChanged(pose);

    if(!mDialog->processPhysics()) return;

    // Move the particlesystem with the vehicle if so desired and the vehicle moved X meters
    const QVector3D pos = pose->getPosition();

    if(mParticleSystem && mDialog->followVehicle() && pos.distanceToLine(mLastParticleSystemPositionToFollowVehicle, QVector3D()) > 20.0)
    {
        qDebug() << "FlightPlannerParticles::slotVehiclePoseChanged(): vehicle moved far enough, moving particle system...";
        mLastParticleSystemPositionToFollowVehicle = pos;

        const Box3D scanVolume(mScanVolumeMin, mScanVolumeMax);
        const Box3D desiredParticleSystemExtents(QVector3D(
                                               pos.x()-32.0f,
                                               pos.y()-10.0f,
                                               pos.z()-32.0f),
                                           QVector3D(
                                               pos.x()+32.0f,
                                               pos.y()+22.0f,
                                               pos.z()+32.0f));

        const Box3D particleSystemInScanVolume = desiredParticleSystemExtents.tryToKeepWithin(scanVolume);

        mParticleSystem->slotSetVolume(particleSystemInScanVolume.min, particleSystemInScanVolume.max);
        //mParticleSystem->slotResetParticles();
    }

    // check for collisions?!
}

void FlightPlannerParticles::slotClearGridWayPointPressure()
{
    // check if we're initialized!
    if(mVboGridMapOfWayPointPressure)
    {
        const quint32 numberOfCells = mSimulationParameters.gridInformationGain.getCellCount();
        qDebug() << "ParticleSystem::slotClearGridWayPointPressure(): clearing pressure in" << numberOfCells << "cells in VBO" << mVboGridMapOfWayPointPressure;
        quint8* gridMapOfWayPointPressure = (quint8*)CudaHelper::mapGLBufferObject(&mCudaVboResourceGridMapOfWayPointPressure);
        cudaSafeCall(cudaMemset(gridMapOfWayPointPressure, 0, sizeof(quint8) * numberOfCells));
        cudaSafeCall(cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfWayPointPressure, 0));
    }
}

void FlightPlannerParticles::showInformationGain()
{
    qDebug() << "ParticleSystem::showInformationGain():";

    // This pointer is NOT accessible on the host (its in device address space), we need to cudaMemcpy() it from the device first.
    quint8* waypointPressureMapDevice = (quint8*)CudaHelper::mapGLBufferObject(&mCudaVboResourceGridMapOfWayPointPressure);

    const quint32 numberOfGridCells = mSimulationParameters.gridInformationGain.getCellCount();

    quint8* waypointPressureMapHost = new quint8[numberOfGridCells];

    cudaSafeCall(cudaMemcpy(waypointPressureMapHost, waypointPressureMapDevice, sizeof(quint8) * numberOfGridCells, cudaMemcpyDeviceToHost));

    for(int i=0;i<numberOfGridCells;i++)
    {
        const int3 cell = mSimulationParameters.gridInformationGain.getCellCoordinate(i);
        Q_ASSERT(i == mSimulationParameters.gridInformationGain.getCellHash(mSimulationParameters.gridInformationGain.getCellCoordinate(i)) && "grid computation error!");
        if(waypointPressureMapHost[i] > 0)
        {
            qDebug() << "grid cell hash" << i << "at grid-coord" << cell.x << cell.y << cell.z << "and pos" << CudaHelper::cudaConvert(mSimulationParameters.gridInformationGain.getCellCenter(cell)) << "has waypoint pressure" << waypointPressureMapHost[i];
        }
    }

    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfWayPointPressure, 0);
    delete waypointPressureMapHost;
}




void FlightPlannerParticles::slotDenseCloudInsertedPoints(PointCloud*const pointCloudSource, const quint32& firstPointToReadFromSrc, quint32 numberOfPointsToCopy)
{
    mPointCloudColliders->slotInsertPoints(pointCloudSource, firstPointToReadFromSrc, numberOfPointsToCopy);

    if(firstPointToReadFromSrc == 0)
    {
        // This is the first time that points were inserted - start the physics processing!
        mDialog->setProcessPhysics(true);
        mParticleSystem->slotResetParticles();
        mTimerProcessInformationGain.start(5000);
    }

    // In the future, we will update the collidercloud manually when generating new waypoints
//    disconnect(mPointCloudDense, SIGNAL(pointsInserted(PointCloud*const,quint32,quint32)), this, SLOT(slotDenseCloudInsertedPoints(PointCloud*const,quint32,quint32)));
}
