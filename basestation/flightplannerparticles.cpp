#include "flightplannerparticles.h"
#include "flightplannerparticlesdialog.h"
#include "particlesystem.cuh"
#include "pointcloudcuda.h"
#include "cudahelper.cuh"
#include "basestation.h"

FlightPlannerParticles::FlightPlannerParticles(BaseStation* baseStation, GlWindow *glWidget, PointCloud *pointcloud) :
    QObject(baseStation),
    mWayPointsAhead(QColor(255,200,0,200)),
    mWayPointsPassed(QColor(128,128,128,50))
{
    mGlWindow = glWidget;
    mBaseStation = baseStation;
    mPointCloudDense = pointcloud;

    mActiveWayPointVisualizationIndex = -1;
    mParticleSystem = 0;
    mPathPlanner = 0;
    mParticleRenderer = 0;
    mVboGridMapOfInformationGainBytes = 0;
    mVboGridMapOfInformationGainFloats = 0;
    mDeviceGridInformationGainCellWorldPositions = 0;
    mShaderProgramDefault = mShaderProgramWaypoint = 0;
    mVboBoundingBoxScanVolume = 0;
    mVboWayPointConnections = 0;

    mRenderWayPointsAhead = mRenderWayPointsPassed = true;
    mRenderScanVolume = true;
    mRenderDetectionVolume = true;

    mVehiclePoses.reserve(25 * 60 * 20); // enough poses for 20 minutes with 25Hz

    mPointCloudColliders = new PointCloudCuda(
                Box3D(
                    QVector3D(-32.0f, -6.0f, -32.0f),
                    QVector3D(32.0f, 26.0f, 32.0f)
                    ),
                64 * 1024);


    mPointCloudColliders->setMinimumPointDistance(0.2f);
    //mPointCloudColliders->setColor(QColor(0,0,255,255));

    mGlWindow->slotPointCloudRegister(mPointCloudColliders);

    mSimulationParameters.initialize();
    mDialog = new FlightPlannerParticlesDialog(&mSimulationParameters, (QWidget*)baseStation);

    mPathPlanner = new PathPlanner;
    connect(mPathPlanner, &PathPlanner::path, this, &FlightPlannerParticles::slotSetWayPoints);
    connect(mPathPlanner, &PathPlanner::generateNewWayPoints, this, &FlightPlannerParticles::slotStartWayPointGeneration);
    connect(mPathPlanner, SIGNAL(message(LogImportance,QString,QString)), this, SIGNAL(message(LogImportance,QString,QString)));

    connect(&mTimerProcessInformationGain, SIGNAL(timeout()), this, SLOT(slotProcessInformationGain()));

    // When the dialog dis/enables physics processing, start/stop the timer
    mTimerStepSimulation.setInterval(1);
    connect(&mTimerStepSimulation, &QTimer::timeout, this, &FlightPlannerParticles::slotStepSimulation);
    connect(mDialog, &FlightPlannerParticlesDialog::processPhysicsChanged, [=](const bool enable) {if(enable) mTimerStepSimulation.start(); else mTimerStepSimulation.stop();});
}

void FlightPlannerParticles::slotInitialize()
{
    qDebug() << "FlightPlannerParticles::slotInitialize()";

    mSimulationParameters.initialize();
    mSimulationParameters.gridInformationGain.cells = make_uint3(256, 32, 256);
    slotSetScanVolume(Box3D(QVector3D(-32, -6, -32), QVector3D(32, 26, 32)));

    const quint32 numberOfCellsInGridInformationGain = mSimulationParameters.gridInformationGain.getCellCount();

    // Store the gridcell-waypoint-pressure-values in a VBO as 8bit-unsigned-ints
    mVboGridMapOfInformationGainBytes = OpenGlUtilities::createVbo(sizeof(quint8) * numberOfCellsInGridInformationGain);
    cudaSafeCall(cudaGraphicsGLRegisterBuffer(&mCudaVboResourceGridMapOfInformationGainBytes, mVboGridMapOfInformationGainBytes, cudaGraphicsMapFlagsNone));
    // Set vbo values to zero
    cudaSafeCall(cudaMemset(CudaHelper::mapGLBufferObject(&mCudaVboResourceGridMapOfInformationGainBytes), 0, sizeof(quint8) * numberOfCellsInGridInformationGain));
    cudaSafeCall(cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfInformationGainBytes, 0));

    // Allocate the same vector again for the same values, just weighed (so we use float) and then sorted.
    mVboGridMapOfInformationGainFloats = OpenGlUtilities::createVbo(sizeof(float) * numberOfCellsInGridInformationGain);
    cudaSafeCall(cudaGraphicsGLRegisterBuffer(&mCudaVboResourceGridMapOfInformationGainFloats, mVboGridMapOfInformationGainFloats, cudaGraphicsMapFlagsNone));

    //cudaSafeCall(cudaMalloc((void**)&mVboGridMapOfInformationGainFloat, sizeof(float) * numberOfCellsInScanVolume));

    // Allocate a vector of float4 for the world-positions of the gridcell positions. These will be the values when sorting
    cudaSafeCall(cudaMalloc((void**)&mDeviceGridInformationGainCellWorldPositions, sizeof(float4) * numberOfCellsInGridInformationGain));

    // Will contain QVector4Ds of worldpos of every gridcell, sorted by waypoint pressure
    //    cudaSafeCall(cudaMalloc((void**)&mDeviceGridMapCellWorldPositions, sizeof(float4) * numberOfCellsInScanVolume));

    mPointCloudColliders->slotInitialize();

    connect(mPointCloudDense, SIGNAL(pointsInserted(PointCloud*const,quint32,quint32)), SLOT(slotDenseCloudInsertedPoints(PointCloud*const,quint32,quint32)));

    mShaderProgramGridLines = new ShaderProgram(this, "shader-default-vertex.c", "", "shader-default-fragment.c");

    mParticleSystem = new ParticleSystem((PointCloudCuda*)mPointCloudDense, (PointCloudCuda*)mPointCloudColliders, &mSimulationParameters); // ParticleSystem will draw its points as colliders from the pointcloud passed here
    mParticleRenderer = new ParticleRenderer;

    connect(mParticleSystem, SIGNAL(particleRadiusChanged(float)), mParticleRenderer, SLOT(slotSetParticleRadius(float)));
    //connect(mParticleSystem, SIGNAL(vboInfoParticles(quint32,quint32,quint32,QVector3D,QVector3D)), mParticleRenderer, SLOT(slotSetVboInfoParticles(quint32,quint32,quint32,QVector3D,QVector3D)));
    connect(mParticleSystem, &ParticleSystem::vboInfoParticles, mParticleRenderer, &ParticleRenderer::slotSetVboInfoParticles);

    connect(mPathPlanner, &PathPlanner::vboInfoGridOccupancy, mParticleRenderer, &ParticleRenderer::slotSetVboInfoGridOccupancy);
    connect(mPathPlanner, &PathPlanner::vboInfoGridPathFinder, mParticleRenderer, &ParticleRenderer::slotSetVboInfoGridPathFinder);
    connect(this, &FlightPlannerParticles::vboInfoGridInformationGain, mParticleRenderer, &ParticleRenderer::slotSetVboInfoGridInformationGain);

    emit vboInfoGridInformationGain(
                mVboGridMapOfInformationGainBytes,
                Box3D(
                    CudaHelper::convert(mSimulationParameters.gridInformationGain.worldMin),
                    CudaHelper::convert(mSimulationParameters.gridInformationGain.worldMax)
                    ),
                Vector3i(mSimulationParameters.gridInformationGain.cells.x, mSimulationParameters.gridInformationGain.cells.y, mSimulationParameters.gridInformationGain.cells.z)
                );

    mParticleSystem->slotSetParticleCount(32768);
    mParticleSystem->slotSetParticleRadius(1.0f/2.0f); // balance against mOctreeCollisionObjects.setMinimumPointDistance() above
    mParticleSystem->slotSetDefaultParticlePlacement(ParticleSystem::ParticlePlacement::PlacementFillSky);

    connect(mDialog, SIGNAL(generateWayPoints()), SLOT(slotGenerateWaypoints()));
    connect(mDialog, SIGNAL(resetParticles()), mParticleSystem, SLOT(slotResetParticles()));
    connect(mDialog, SIGNAL(resetInformationGain()), SLOT(slotClearGridOfInformationGain()));
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

void FlightPlannerParticles::slotShowUserInterface()
{
    mDialog->show();
}

// If the maximum informationGain is higher than threshold, it will generate new waypoints and decrease the pressure.
void FlightPlannerParticles::slotProcessInformationGain(const quint8 threshold)
{
    if(mVboGridMapOfInformationGainBytes == 0 || mVboGridMapOfInformationGainFloats == 0)
    {
        qDebug() << "FlightPlannerParticles::slotProcessInformationGain(): not initialized yet, returning.";
        return;
    }

    const quint32 numberOfCells = mSimulationParameters.gridInformationGain.getCellCount();

    // Copy waypoint pressure from VBO into mDeviceGridMapinformationGainSorted
    quint8* gridMapOfInformationGain = (quint8*)CudaHelper::mapGLBufferObject(&mCudaVboResourceGridMapOfInformationGainBytes);
    const quint8 maxPressure = getMaximumInformationGain(gridMapOfInformationGain, numberOfCells);
    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfInformationGainBytes, 0);

    if(maxPressure >= threshold)
    {
        qDebug() << "FlightPlannerParticles::slotProcessInformationGain(): max pressure" << maxPressure << ">" << "threshold" << threshold << "- generating waypoints!";
        // Now that we're generating waypoints, we can deactivate the physics processing (and restart when only few waypoints are left)
        mDialog->setProcessPhysics(false);
        mTimerProcessInformationGain.stop();
        slotGenerateWaypoints();
    }
}

void FlightPlannerParticles::slotGenerateWaypoints(quint32 numberOfWaypointsToGenerate)
{
    if(mVboGridMapOfInformationGainBytes == 0 || mVboGridMapOfInformationGainFloats == 0)
    {
        qDebug() << "FlightPlannerParticles::getRankedWaypoints(): not initialized yet, returning.";
        return;
    }

    copyParametersToGpu(&mSimulationParameters);

    const quint32 numberOfCells = mSimulationParameters.gridInformationGain.getCellCount();
    QVector<QVector4D> waypoints(std::min((quint32)200, numberOfCells));

    QVector3D vehiclePosition;
    if(mVehiclePoses.size()) vehiclePosition = mVehiclePoses.last().getPosition();

    // Copy raw information gain from byte-VBO into weighed (to distance) information gain in float-VBO
    float* gridMapOfinformationGainFloat = (float*)CudaHelper::mapGLBufferObject(&mCudaVboResourceGridMapOfInformationGainFloats);
    quint8* gridMapOfinformationGainBytes = (quint8*)CudaHelper::mapGLBufferObject(&mCudaVboResourceGridMapOfInformationGainBytes);
    computeWaypointBenefit(gridMapOfinformationGainFloat, gridMapOfinformationGainBytes, (float*)&vehiclePosition, numberOfCells);
    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfInformationGainBytes, 0);

    // Fill mDeviceGridMapCellWorldPositions - this might be done only once and then copied lateron (just like the waypoint pressure above)
    // The w-component is set to 0.0 here
    fillGridMapCellWorldPositions(mDeviceGridInformationGainCellWorldPositions, numberOfCells);

    // Sort mDeviceGridInformationGainCellWorldPositions according to gridMapOfinformationGainFloat and fill the w-component
    sortGridMapInformationGainAndCopyInformationGain(gridMapOfinformationGainFloat, mDeviceGridInformationGainCellWorldPositions, numberOfCells, waypoints.size());

    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfInformationGainFloats, 0);

    // Copy the @count cells with the highest waypoint pressure into @waypoints
    cudaMemcpy(waypoints.data(), mDeviceGridInformationGainCellWorldPositions, sizeof(QVector4D) * waypoints.size(), cudaMemcpyDeviceToHost);

    // Do we want to clear the old (but still pending) waypoints?
    mWayPointsAhead.clear();

    WayPointList wayPointsWithHighInformationGain;

    for(int i=0;i<waypoints.size();i++)
    {
        QVector4D wp = waypoints[i];

        // If we ask for more waypoints than cells with wp-pressure > 0.0, we'll get useless candidates!
        if(wp.w() > 0.1f)
        {
            // First, create waypoints exactly where the information-gain-cell is.
            // This is directly on the ground/walls/geometry, which is unsafe to reach.
            wayPointsWithHighInformationGain.append(WayPoint(wp.toVector3D(), wp.w()));
        }
    }

    // Now use the PathPlanner to move waypoints into safe areas and find a path through them. It will emit a signal with the list.
    if(wayPointsWithHighInformationGain.size())
    {
        // Merge waypoints if they're closer than X meters
        wayPointsWithHighInformationGain.mergeCloseWaypoints(7.5f);

        // Reduce them to the desired number
        while(wayPointsWithHighInformationGain.size() > numberOfWaypointsToGenerate)
            wayPointsWithHighInformationGain.removeLast();

        qDebug() << "FlightPlannerParticles::slotGenerateWaypoints(): after merging and reducing:" << wayPointsWithHighInformationGain.toString();

        // Now use the occupancy grid to move the waypoints into close-by, safe areas
        mPathPlanner->moveWayPointsToSafety(&wayPointsWithHighInformationGain);

        // Sort them to the shortest path, honoring the vehicle's current location.
        wayPointsWithHighInformationGain.sortToShortestPath(getLastKnownVehiclePose().getPosition());

        qDebug() << "FlightPlannerParticles::slotGenerateWaypoints(): after sort to shortest path:" << wayPointsWithHighInformationGain.toString();

        mPathPlanner->slotComputePath(vehiclePosition, wayPointsWithHighInformationGain);
    }
    else
    {
        qDebug() << __PRETTY_FUNCTION__ << "waypointlist is empty, informationGain is probably also empty, NOT stopping physics!";
        emit message(LogImportance::Warning, "flightplanner", "cannot generate waypoints, is there information gain?");
    }
}

FlightPlannerParticles::~FlightPlannerParticles()
{
    delete mParticleRenderer;
    delete mParticleSystem;

    if(CudaHelper::isDeviceSupported)
    {
        cudaSafeCall(cudaFree(mDeviceGridInformationGainCellWorldPositions));
        //cudaSafeCall(cudaFree(mVboGridMapOfInformationGainFloat));
        cudaSafeCall(cudaGraphicsUnregisterResource(mCudaVboResourceGridMapOfInformationGainBytes));
        cudaSafeCall(cudaGraphicsUnregisterResource(mCudaVboResourceGridMapOfInformationGainFloats));
    }

    glDeleteBuffers(1, (const GLuint*)&mVboGridMapOfInformationGainBytes);
    glDeleteBuffers(1, (const GLuint*)&mVboGridMapOfInformationGainFloats);
}



void FlightPlannerParticles::slotClearVehicleTrajectory()
{
    mVehiclePoses.clear();
    //    if(mGlWindow) mGlWindow->slotClearVehicleTrajectory();
    //    emit suggestVisualization();
}

const Pose FlightPlannerParticles::getLastKnownVehiclePose(void) const
{
    if(mVehiclePoses.size())
        return mVehiclePoses.last();
    else
        return Pose();
}

void FlightPlannerParticles::slotNewScanFused(const float* const points, const quint32& count, const QVector3D* const scannerPosition)
{
    // Insert all points into mPointCloudDense
    mPointCloudDense->slotInsertPoints4(points, count);

    emit suggestVisualization();
}

void FlightPlannerParticles::slotSetScanVolume(const Box3D scanVolume)
{
    qDebug() << __PRETTY_FUNCTION__ << scanVolume;
    // We're being told to change the particle system volume - this is NOT the particle system' world's volume!
    slotClearGridOfInformationGain();

    mScanVolume = scanVolume;

    OpenGlUtilities::setVboToBoundingBox(mVboBoundingBoxScanVolume, mScanVolume);

    mSimulationParameters.gridInformationGain.worldMin = CudaHelper::convert(mScanVolume.min);
    mSimulationParameters.gridInformationGain.worldMax = CudaHelper::convert(mScanVolume.max);

    if(CudaHelper::isDeviceSupported)
    {
        copyParametersToGpu(&mSimulationParameters);

        emit vboInfoGridInformationGain(
                    mVboGridMapOfInformationGainBytes,
                    Box3D(
                        CudaHelper::convert(mSimulationParameters.gridInformationGain.worldMin),
                        CudaHelper::convert(mSimulationParameters.gridInformationGain.worldMax)
                        ),
                    Vector3i(mSimulationParameters.gridInformationGain.cells.x, mSimulationParameters.gridInformationGain.cells.y, mSimulationParameters.gridInformationGain.cells.z)
                    );
    }
}

void FlightPlannerParticles::slotSetWayPoints(const QList<WayPoint>* const wayPointList, const WayPointListSource source)
{
    mWayPointsAhead.setList(wayPointList);

    // Tell others about our new waypoints!
    emit wayPoints(mWayPointsAhead.list(), source);
    emit suggestVisualization();
}

const QList<WayPoint>* const FlightPlannerParticles::getWayPoints()
{
    return mWayPointsAhead.list();
}

void FlightPlannerParticles::slotWayPointReached(const WayPoint& wpt)
{
    qDebug() << "FlightPlannerParticles::slotWayPointReached(): rover->baseconnection->flightplanner waypoint reached, so appending first element of mWayPointsAhead to mWayPointsPassed";

    if(mWayPointsAhead.size())
    {
        mWayPointsPassed.append(mWayPointsAhead.takeAt(0));
    }
    else
    {
        qDebug() << __PRETTY_FUNCTION__ << "mWayPointsAhead is empty, but rover reached wpt" << wpt.toString() << "- appending anyways.";
        mWayPointsPassed.append(wpt);
    }

    emit message(
                Information,
                QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                QString("Reached waypoint %1 %2 %3").arg(wpt.x()).arg(wpt.y()).arg(wpt.z()));

    emit wayPoints(mWayPointsAhead.list(), WayPointListSource::WayPointListSourceFlightPlanner);
    qDebug() << "FlightPlannerParticles::slotWayPointReached(): rover->baseconnection->flightplanner waypoint reached, emitted wayPoints())";
    emit suggestVisualization();

    // When no waypoints are left, start testing for water-leaks!
    if(mWayPointsAhead.isEmpty())
    {
        qDebug() << __PRETTY_FUNCTION__ << "mWayPointsAhead is empty, starting waypoint generation";
        emit message(
                    Information,
                    QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                    QString("No more waypoints, starting generation"));
        slotStartWayPointGeneration();
    }
}

void FlightPlannerParticles::slotStartWayPointGeneration()
{
    qDebug() << __PRETTY_FUNCTION__ << "starting generation!";

    // This will first copy points to collider (which will reduce), then reduce the dense cloud.
    ((PointCloudCuda*)mPointCloudDense)->slotReduce();

    slotClearGridOfInformationGain();

    mDialog->setProcessPhysics(true);

    const Pose p = mVehiclePoses.last();
    slotVehiclePoseChanged(&p);

    mParticleSystem->slotResetParticles();

    mTimerProcessInformationGain.start(5000);
}

void FlightPlannerParticles::slotVehiclePoseChanged(const Pose* const pose)
{
    mVehiclePoses.append(*pose);

    if(!mDialog->processPhysics()) return;

    // Move the particlesystem with the vehicle if so desired and the vehicle moved X meters
    const QVector3D pos = pose->getPosition();

    if(mParticleSystem && mDialog->followVehicle() && pos.distanceToLine(mLastParticleSystemPositionToFollowVehicle, QVector3D()) > 20.0)
    {
        qDebug() << "FlightPlannerParticles::slotVehiclePoseChanged(): vehicle moved far enough, moving particle system...";
        mLastParticleSystemPositionToFollowVehicle = pos;

        const Box3D desiredParticleSystemExtents(QVector3D(
                                                     pos.x()-32.0f,
                                                     pos.y()-10.0f,
                                                     pos.z()-32.0f),
                                                 QVector3D(
                                                     pos.x()+32.0f,
                                                     pos.y()+22.0f,
                                                     pos.z()+32.0f));

        const Box3D particleSystemInScanVolume = desiredParticleSystemExtents.tryToKeepWithin(mScanVolume);

        mParticleSystem->slotSetVolume(particleSystemInScanVolume);
        //mParticleSystem->slotResetParticles();
    }

    // check for collisions?!
}

void FlightPlannerParticles::slotClearGridOfInformationGain()
{
    // check if we're initialized!
    if(mVboGridMapOfInformationGainBytes)
    {
        const quint32 numberOfCells = mSimulationParameters.gridInformationGain.getCellCount();
        qDebug() << "ParticleSystem::slotClearGridinformationGain(): clearing information gain in" << numberOfCells << "cells in VBO" << mVboGridMapOfInformationGainBytes;
        quint8* gridMapOfInformationGainBytes = (quint8*)CudaHelper::mapGLBufferObject(&mCudaVboResourceGridMapOfInformationGainBytes);
        cudaSafeCall(cudaMemset(gridMapOfInformationGainBytes, 0, sizeof(quint8) * numberOfCells));
        cudaSafeCall(cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfInformationGainBytes, 0));
    }
}

void FlightPlannerParticles::showInformationGain()
{
    qDebug() << "ParticleSystem::showInformationGain():";

    // This pointer is NOT accessible on the host (its in device address space), we need to cudaMemcpy() it from the device first.
    quint8* informationGainMapDevice = (quint8*)CudaHelper::mapGLBufferObject(&mCudaVboResourceGridMapOfInformationGainBytes);

    const quint32 numberOfGridCells = mSimulationParameters.gridInformationGain.getCellCount();

    quint8* informationGainMapHost = new quint8[numberOfGridCells];

    cudaSafeCall(cudaMemcpy(informationGainMapHost, informationGainMapDevice, sizeof(quint8) * numberOfGridCells, cudaMemcpyDeviceToHost));

    for(int i=0;i<numberOfGridCells;i++)
    {
        const int3 cell = mSimulationParameters.gridInformationGain.getCellCoordinate(i);
        Q_ASSERT(i == mSimulationParameters.gridInformationGain.getCellHash(mSimulationParameters.gridInformationGain.getCellCoordinate(i)) && "grid computation error!");
        if(informationGainMapHost[i] > 0)
        {
            qDebug() << "grid cell hash" << i << "at grid-coord" << cell.x << cell.y << cell.z << "and pos" << CudaHelper::convert(mSimulationParameters.gridInformationGain.getCellCenter(cell)) << "has waypoint pressure" << informationGainMapHost[i];
        }
    }

    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfInformationGainBytes, 0);
    delete informationGainMapHost;
}

void FlightPlannerParticles::slotDenseCloudInsertedPoints(PointCloud*const pointCloudSource, const quint32& firstPointToReadFromSrc, quint32 numberOfPointsToCopy)
{
    // Son, we shouldn't mess with CUDA while rendering is in progress
    Q_ASSERT(!mGlWindow->mIsCurrentlyRendering);

    mPointCloudColliders->slotInsertPoints(pointCloudSource, firstPointToReadFromSrc, numberOfPointsToCopy);

    if(mBaseStation->getOperatingMode() == OperatingMode::OperatingOnline && mWayPointsAhead.size() == 0 && !mTimerStepSimulation.isActive() && !mTimerProcessInformationGain.isActive())
    {
        // This is the first time that points were inserted (and we have no waypoints) - start the physics processing!
        slotStartWayPointGeneration();
    }
    else if(mWayPointsAhead.size())
    {
        // Points were added to the pointcloud and there's waypoints currently being passed
        // Check to make sure that none of the new points are in cells that contain waypoints!
        mPathPlanner->checkWayPointSafety(getLastKnownVehiclePose().getPosition(), &mWayPointsAhead);
    }
}

void FlightPlannerParticles::slotVisualize()
{
    // Initialize shaders and VBO if necessary
    if((mShaderProgramWaypoint == 0 || mShaderProgramDefault == 0) && mGlWindow != 0)
    {
        qDebug() << __PRETTY_FUNCTION__ << "initializing opengl...";
        initializeOpenGLFunctions();
        mShaderProgramDefault = new ShaderProgram(this, "shader-default-vertex.c", "", "shader-default-fragment.c");
        mShaderProgramWaypoint = new ShaderProgram(this, "shader-waypoint-vertex.c", "", "shader-waypoint-fragment.c");
    }

    if(mVboBoundingBoxScanVolume == 0)
    {
        glGenBuffers(1, &mVboBoundingBoxScanVolume);
        OpenGlUtilities::setVboToBoundingBox(mVboBoundingBoxScanVolume, mScanVolume);
    }

    if(mRenderScanVolume && mShaderProgramDefault != 0)
    {
        mShaderProgramDefault->bind();
        mShaderProgramDefault->setUniformValue("useFixedColor", true);

        glEnable(GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Beau.Ti.Ful!
        {
            glBindBuffer(GL_ARRAY_BUFFER, mVboBoundingBoxScanVolume);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0); // position

            // draw the lines around the box
            mShaderProgramDefault->setUniformValue("fixedColor", QVector4D(0.2f, 0.2f, 1.0f, 0.8f));
            glDrawArrays(GL_LINE_LOOP, 0, 4);
            glDrawArrays(GL_LINE_LOOP, 4, 4);
            glDrawArrays(GL_LINE_LOOP, 8, 4);
            glDrawArrays(GL_LINE_LOOP, 12, 4);
            glDrawArrays(GL_LINE_LOOP, 16, 4);
            glDrawArrays(GL_LINE_LOOP, 20, 4);

            // draw a half-transparent box
            //            mShaderProgramDefault->setUniformValue("fixedColor", QVector4D(1.0f, 1.0f, 1.0f, 0.015f));
            //            glDrawArrays(GL_QUADS, 0, 24);

            glDisableVertexAttribArray(0);
        }
        glDisable(GL_BLEND);
        mShaderProgramDefault->release();
    }

    if(mShaderProgramDefault != 0 && mShaderProgramWaypoint != 0)
    {
        glEnable(GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Beau.Ti.Ful!
        {
                if(mRenderWayPointsAhead && !mWayPointsAhead.isEmpty())
                {
                    const QColor c = mWayPointsAhead.color();
                    mShaderProgramDefault->bind();
                    mShaderProgramDefault->setUniformValue("useFixedColor", true);
                    mShaderProgramDefault->setUniformValue("fixedColor", QVector4D(c.redF(),c.greenF(),c.blueF(),c.alphaF()));
                    glBindBuffer(GL_ARRAY_BUFFER, mWayPointsAhead.vbo());
                    // Make the contents of this array available at layout position vertexShaderVertexIndex in the vertex shader
                    Q_ASSERT(glGetAttribLocation(mShaderProgramDefault->programId(), "in_position") != -1);
                    glEnableVertexAttribArray(glGetAttribLocation(mShaderProgramDefault->programId(), "in_position"));
                    glVertexAttribPointer(glGetAttribLocation(mShaderProgramDefault->programId(), "in_position"), 4, GL_FLOAT, GL_FALSE, 20, 0);
                    glLineWidth(2.0f);
                    glDrawArrays(GL_LINE_STRIP, 0, mWayPointsAhead.size());
                    mShaderProgramDefault->release();

                    mShaderProgramWaypoint->bind();
                    mShaderProgramWaypoint->setUniformValue("activeWayPointIndex", mActiveWayPointVisualizationIndex);
                    mShaderProgramWaypoint->setUniformValue("fixedColor", QVector4D(c.redF(),c.greenF(),c.blueF(),c.alphaF()));

                    mShaderProgramWaypoint->enableAttributeArray("in_informationgain");
                    glVertexAttribPointer(mShaderProgramWaypoint->attributeLocation("in_informationgain"), 1, GL_FLOAT, GL_FALSE, 20, (void*)16);
                    glDrawArrays(GL_POINTS, 0, mWayPointsAhead.size());
                    glDisableVertexAttribArray(glGetAttribLocation(mShaderProgramDefault->programId(), "in_position"));
                    glDisableVertexAttribArray(glGetAttribLocation(mShaderProgramDefault->programId(), "in_informationgain"));
                    mShaderProgramWaypoint->release();
                    glBindBuffer(GL_ARRAY_BUFFER, 0);
                }

                if(mRenderWayPointsPassed && !mWayPointsPassed.isEmpty())
                {
                    const QColor c = mWayPointsPassed.color();
                    mShaderProgramDefault->bind();
                    mShaderProgramDefault->setUniformValue("useFixedColor", true);
                    mShaderProgramDefault->setUniformValue("fixedColor", QVector4D(c.redF(),c.greenF(),c.blueF(),c.alphaF()));
                    glBindBuffer(GL_ARRAY_BUFFER, mWayPointsPassed.vbo());
                    // Make the contents of this array available at layout position vertexShaderVertexIndex in the vertex shader
                    Q_ASSERT(glGetAttribLocation(mShaderProgramDefault->programId(), "in_position") != -1);
                    glEnableVertexAttribArray(glGetAttribLocation(mShaderProgramDefault->programId(), "in_position"));
                    glVertexAttribPointer(glGetAttribLocation(mShaderProgramDefault->programId(), "in_position"), 4, GL_FLOAT, GL_FALSE, 20, 0);
                    glLineWidth(1.0f);
                    glDrawArrays(GL_LINE_STRIP, 0, mWayPointsPassed.size());
                    mShaderProgramDefault->release();

                    mShaderProgramWaypoint->bind();
                    mShaderProgramWaypoint->setUniformValue("activeWayPointIndex", -1);
                    mShaderProgramWaypoint->setUniformValue("fixedColor", QVector4D(c.redF(),c.greenF(),c.blueF(),c.alphaF()));

                    mShaderProgramWaypoint->enableAttributeArray("in_informationgain");
                    glVertexAttribPointer(mShaderProgramWaypoint->attributeLocation("in_informationgain"), 1, GL_FLOAT, GL_FALSE, 20, (void*)16);
                    glDrawArrays(GL_POINTS, 0, mWayPointsPassed.size());
                    glDisableVertexAttribArray(glGetAttribLocation(mShaderProgramDefault->programId(), "in_position"));
                    glDisableVertexAttribArray(glGetAttribLocation(mShaderProgramDefault->programId(), "in_informationgain"));
                    mShaderProgramWaypoint->release();
                    glBindBuffer(GL_ARRAY_BUFFER, 0);
                }
        }
        glDisable(GL_BLEND);
    }

    if(!mParticleSystem && CudaHelper::isDeviceSupported)
        slotInitialize();

    if(mParticleSystem)
    {
        mParticleRenderer->slotSetRenderBoundingBox(mRenderDetectionVolume);
        mParticleRenderer->render();
    }

}

void FlightPlannerParticles::slotStepSimulation()
{
    if(!mParticleSystem && CudaHelper::isDeviceSupported)
    {
        mParticleSystem->slotResetParticles();
        slotInitialize();
    }

    if(mPointCloudColliders->getNumberOfPoints() < 1)
    {
        qDebug() << __PRETTY_FUNCTION__ << "will not collide particles against 0 colliders, disablig particle simulation";
        mDialog->setProcessPhysics(false);
        return;
    }

    // Provide the particle system a pointer to our waypoint-pressure-gridmap (in device address space)
    quint8 *deviceGridMapOfinformationGain = (quint8*)CudaHelper::mapGLBufferObject(&mCudaVboResourceGridMapOfInformationGainBytes);

    for(float timeStepped = 0.0f; timeStepped < mSimulationParameters.timeStepOuter; timeStepped += mSimulationParameters.timeStepInner)
    {
        mParticleSystem->update(deviceGridMapOfinformationGain);
    }

    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfInformationGainBytes, 0);

    static int iteration = 0;
    if(iteration++ % 100 == 0)
        emit suggestVisualization();
}
