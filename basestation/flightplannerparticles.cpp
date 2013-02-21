#include <GL/glew.h>
#include "flightplannerparticles.h"
#include "flightplannerparticlesdialog.h"
#include "particlesystem_cuda.cuh"
#include "pointcloudcuda.h"

FlightPlannerParticles::FlightPlannerParticles(QWidget* parentWidget, GlWidget *glWidget, PointCloud *pointcloud) : FlightPlannerInterface(parentWidget, glWidget, pointcloud)
{
    mParticleSystem = 0;
    mParticleRenderer = 0;
    mVboGridMapOfWayPointPressure = 0;
    mDeviceGridMapWaypointPressureCellWorldPositions = 0;

    // register dense pointcloud for rendering. Might be moved to base class c'tor
    mGlWidget->slotPointCloudRegister(mPointCloudDense);

    mPointCloudColliders = new PointCloudCuda(
                QVector3D(-32.0f, -4.0f, -32.0f),
                QVector3D(32.0f, 60.0f, 32.0f),
                256 * 1024);

    mPointCloudColliders->setMinimumPointDistance(0.5f);

    mPointCloudColliders->setColor(QColor(255,255,255,64));

    mGlWidget->slotPointCloudRegister(mPointCloudColliders);

    mDialog = new FlightPlannerParticlesDialog(parentWidget);
}

void FlightPlannerParticles::slotInitializeWaypointPressureGrid()
{
    const quint32 numberOfCellsInScanVolume = mSimulationParameters.gridWaypointPressure.cells.x * mSimulationParameters.gridWaypointPressure.cells.y * mSimulationParameters.gridWaypointPressure.cells.z;

    // Store the gridcell-waypoint-pressure-values in a VBO as 8bit-unsigned-ints
    mVboGridMapOfWayPointPressure = OpenGlUtilities::createVbo(sizeof(quint8) * numberOfCellsInScanVolume);
    cudaSafeCall(cudaGraphicsGLRegisterBuffer(&mCudaVboResourceGridMapOfWayPointPressure, mVboGridMapOfWayPointPressure, cudaGraphicsMapFlagsNone));
    // Set vbo values to zero
    cudaSafeCall(cudaMemset(mapGLBufferObject(&mCudaVboResourceGridMapOfWayPointPressure), 0, sizeof(quint8) * numberOfCellsInScanVolume));
    cudaSafeCall(cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfWayPointPressure, 0));

    // Allocate the same size again for the same values. Here, they can be sorted in-place. Unfortunately, there is no thrust::sort_by_key()
    // which would leave the keys (=mVboGridMapOfWayPointPressure) alone and just rearrange the values (=mDeviceGridMapCellWorldPositions).
    cudaSafeCall(cudaMalloc((void**)&mDeviceGridMapWayPointPressureSorted, sizeof(quint8) * numberOfCellsInScanVolume));

    // Allocate a vector of float4 for the world-positions of the gridcell positions. These will be the values when sorting
    cudaSafeCall(cudaMalloc((void**)&mDeviceGridMapWaypointPressureCellWorldPositions, sizeof(float4) * numberOfCellsInScanVolume));

    // Will contain QVector4Ds of worldpos of every gridcell, sorted by waypoint pressure
//    cudaSafeCall(cudaMalloc((void**)&mDeviceGridMapCellWorldPositions, sizeof(float4) * numberOfCellsInScanVolume));
}

void FlightPlannerParticles::slotInitialize()
{
    qDebug() << "FlightPlannerParticles::slotInitialize()";

    mSimulationParameters.gridWaypointPressure.cells = make_uint3(256, 32, 256);
    slotSetScanVolume(QVector3D(-64, -5, -64), QVector3D(64, 27, 64));
    slotInitializeWaypointPressureGrid();

    mPointCloudColliders->slotInitialize();

//    connect(mPointCloudDense, SIGNAL(pointsInserted(PointCloud*const,quint32,quint32)), mPointCloudColliders, SLOT(slotInsertPoints(PointCloud*const,quint32,quint32)));
    connect(mPointCloudDense, SIGNAL(pointsInserted(PointCloud*const,quint32,quint32)), SLOT(slotDenseCloudInsertedPoints(PointCloud*const,quint32,quint32)));

    mShaderProgramGridLines = new ShaderProgram(this, "shader-default-vertex.c", "", "shader-default-fragment.c");

    mParticleSystem = new ParticleSystem((PointCloudCuda*)mPointCloudDense, (PointCloudCuda*)mPointCloudColliders, &mSimulationParameters); // ParticleSystem will draw its points as colliders from the pointcloud passed here
    mParticleRenderer = new ParticleRenderer;

    connect(mParticleSystem, SIGNAL(particleRadiusChanged(float)), mParticleRenderer, SLOT(slotSetParticleRadius(float)));
    connect(mParticleSystem, SIGNAL(vboInfoParticles(quint32,quint32,quint32,QVector3D,QVector3D)), mParticleRenderer, SLOT(slotSetVboInfoParticles(quint32,quint32,quint32,QVector3D,QVector3D)));

    connect(
                this,
                SIGNAL(vboInfoGridWaypointPressure(quint32,QVector3D,QVector3D,Vector3i)),
                mParticleRenderer,
                SLOT(slotSetVboInfoGridWaypointPressure(quint32,QVector3D,QVector3D,Vector3i))
                );

    emit vboInfoGridWaypointPressure(
                mVboGridMapOfWayPointPressure,
                QVector3D(mSimulationParameters.gridWaypointPressure.worldMin.x, mSimulationParameters.gridWaypointPressure.worldMin.y, mSimulationParameters.gridWaypointPressure.worldMin.z),
                QVector3D(mSimulationParameters.gridWaypointPressure.worldMax.x, mSimulationParameters.gridWaypointPressure.worldMax.y, mSimulationParameters.gridWaypointPressure.worldMax.z),
                Vector3i(mSimulationParameters.gridWaypointPressure.cells.x, mSimulationParameters.gridWaypointPressure.cells.y, mSimulationParameters.gridWaypointPressure.cells.z)
                );

    mParticleSystem->slotSetParticleCount(16384);
    mParticleSystem->slotSetParticleRadius(0.5f); // balance against mOctreeCollisionObjects.setMinimumPointDistance() above
    mParticleSystem->slotSetDefaultParticlePlacement(ParticleSystem::ParticlePlacement::PlacementFillSky);

    connect(mDialog, SIGNAL(deleteWayPoints()), SLOT(slotWayPointsClear()));
    connect(mDialog, SIGNAL(generateWayPoints()), SLOT(slotGenerateWaypoints()));
    connect(mDialog, SIGNAL(resetParticles()), mParticleSystem, SLOT(slotResetParticles()));
    connect(mDialog, SIGNAL(resetWaypointPressure()), SLOT(slotClearGridWayPointPressure()));
    connect(mDialog, SIGNAL(simulationParameters(const SimulationParameters*)), mParticleSystem, SLOT(slotSetSimulationParametersFromUi(const SimulationParameters*)));

//    connect(mDialog, SIGNAL(processPhysicsChanged(bool)), SLOT(slotProcessPhysics(bool)));
    connect(mDialog, SIGNAL(showParticlesChanged(bool)), mParticleRenderer, SLOT(slotSetRenderParticles(bool)));
    connect(mDialog, SIGNAL(showWaypointPressureChanged(bool)), mParticleRenderer, SLOT(slotSetRenderWaypointPressure(bool)));
}

void FlightPlannerParticles::keyPressEvent(QKeyEvent *event)
{
}

void FlightPlannerParticles::slotShowUserInterface()
{
    // Just show the dialog for generating-options
    mDialog->show();
}

void FlightPlannerParticles::slotGenerateWaypoints()
{
    if(mVboGridMapOfWayPointPressure == 0)
    {
        qDebug() << "FlightPlannerParticles::getRankedWaypoints(): not initialized yet, returning.";
        return;
    }

    const quint32 numberOfCells = mSimulationParameters.gridWaypointPressure.cellCount();
    QVector<QVector4D> waypoints(std::min((quint32)200, numberOfCells));

    // Copy waypoint pressure from VBO into mDeviceGridMapWayPointPressureSorted
    quint8* gridMapOfWayPointPressure = (quint8*)mapGLBufferObject(&mCudaVboResourceGridMapOfWayPointPressure);
    cudaMemcpy(mDeviceGridMapWayPointPressureSorted, gridMapOfWayPointPressure, sizeof(quint8) * numberOfCells, cudaMemcpyDeviceToDevice);
    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfWayPointPressure, 0);

    // Fill mDeviceGridMapCellWorldPositions - this might be done only once and then copied lateron (just like the waypoint pressure above)
    fillGridMapCellWorldPositions(mDeviceGridMapWaypointPressureCellWorldPositions, numberOfCells);

    // Sort mDeviceGridMapWayPointPressureSorted => mDeviceGridMapCellWorldPositions according to the keys DESC
    sortGridMapWayPointPressure(mDeviceGridMapWayPointPressureSorted, mDeviceGridMapWaypointPressureCellWorldPositions, numberOfCells, waypoints.size());

    // Copy the @count cells with the highest waypoint pressure into @waypoints
    cudaMemcpy(waypoints.data(), mDeviceGridMapWaypointPressureCellWorldPositions, sizeof(QVector4D) * waypoints.size(), cudaMemcpyDeviceToHost);

    WayPointList* const wpl = mWaypointListMap.value("ahead");

    // Do we want to clear the old (but still pending) waypoints?
    wpl->clear();

    for(int i=0;i<waypoints.size();i++)
    {
        QVector4D wp = waypoints[i];

        // If we ask for more waypoints than cells with wp-pressure > 0.0, we'll get useless candidates!
        if(wp.w() > 0.1f)
            wpl->append(WayPoint(wp.toVector3D() + QVector3D(0.0f, 5.0f, 0.0f), wp.w(), WayPoint::Purpose::SCAN));
    }

    qDebug() << "FlightPlannerParticles::slotGenerateWaypoints(): generated" << wpl->size() << "waypoints";

    // Merge waypoints if they're closer than X meters
    wpl->mergeCloseWaypoints(10.0f);

    // Sort them to the shortest path, honoring the vehicle's current location.
    wpl->sortToShortestPath(getLastKnownVehiclePose().getPosition());

    qDebug() << "FlightPlannerParticles::slotGenerateWaypoints(): after merging, there are" << wpl->size() << "waypoints left";

    // Now that we've generated waypoints, we can deactivate the physics processing (and restart when only few waypoints are left)
    mDialog->setProcessPhysics(false);

    emit wayPoints(wpl->list());
    emit wayPointsSetOnRover(wpl->list());
}

void FlightPlannerParticles::slotCreateSafePathToNextWayPoint()
{

}

FlightPlannerParticles::~FlightPlannerParticles()
{
    delete mParticleRenderer;
    delete mParticleSystem;

    cudaSafeCall(cudaFree(mDeviceGridMapWaypointPressureCellWorldPositions));
    cudaSafeCall(cudaFree(mDeviceGridMapWayPointPressureSorted));
    cudaSafeCall(cudaGraphicsUnregisterResource(mCudaVboResourceGridMapOfWayPointPressure));
    glDeleteBuffers(1, (const GLuint*)&mVboGridMapOfWayPointPressure);
}


void FlightPlannerParticles::slotNewScanData(const float* const points, const quint32& count, const QVector3D* const scannerPosition)
{
    // Insert all points into mPointCloudDense
    mPointCloudDense->slotInsertPoints3(points, count);

    emit suggestVisualization();
}

void FlightPlannerParticles::slotVisualize()
{
    FlightPlannerInterface::slotVisualize();

    if(!mParticleSystem)
        slotInitialize();

    if(mParticleSystem)
    {
        if(mDialog->processPhysics())
        {
            // Provide the particle system a pointer to our waypoint-pressure-gridmap (in device address space)
            quint8 *deviceGridMapOfWayPointPressure = (quint8*)mapGLBufferObject(&mCudaVboResourceGridMapOfWayPointPressure);

            mParticleSystem->update(0.1f, deviceGridMapOfWayPointPressure);

            cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfWayPointPressure, 0);
        }

        mParticleRenderer->render();
    }

}

void FlightPlannerParticles::slotSetScanVolume(const QVector3D min, const QVector3D max)
{
    // We're being told to change the scan-volume - this is NOT the particle system' world's volume!
    slotClearGridWayPointPressure();

    FlightPlannerInterface::slotSetScanVolume(min, max);

    mSimulationParameters.gridWaypointPressure.worldMin.x = mScanVolumeMin.x();
    mSimulationParameters.gridWaypointPressure.worldMin.y = mScanVolumeMin.y();
    mSimulationParameters.gridWaypointPressure.worldMin.z = mScanVolumeMin.z();
    mSimulationParameters.gridWaypointPressure.worldMax.x = mScanVolumeMax.x();
    mSimulationParameters.gridWaypointPressure.worldMax.y = mScanVolumeMax.y();
    mSimulationParameters.gridWaypointPressure.worldMax.z = mScanVolumeMax.z();

    copyParametersToGpu(&mSimulationParameters);

    emit vboInfoGridWaypointPressure(
                mVboGridMapOfWayPointPressure,
                QVector3D(mSimulationParameters.gridWaypointPressure.cells.x, mSimulationParameters.gridWaypointPressure.cells.y, mSimulationParameters.gridWaypointPressure.cells.z),
                QVector3D(mSimulationParameters.gridWaypointPressure.cells.x, mSimulationParameters.gridWaypointPressure.cells.y, mSimulationParameters.gridWaypointPressure.cells.z),
                Vector3i(mSimulationParameters.gridWaypointPressure.cells.x, mSimulationParameters.gridWaypointPressure.cells.y, mSimulationParameters.gridWaypointPressure.cells.z)
                );
}

void FlightPlannerParticles::slotWayPointReached(const WayPoint wpt)
{
    FlightPlannerInterface::slotWayPointReached(wpt);

    // When only two waypoints are left, we want to clear the ParticleSystem's grid of wayppint pressure.
    // This way, the particlesystem has the duration of the two remaining waypoints to create new waypoints
    if(mWaypointListMap.value("ahead")->size() == 3)
    {
        slotClearGridWayPointPressure();
        mDialog->setProcessPhysics(true);
        const Pose p = mVehiclePoses.last();
        slotVehiclePoseChanged(&p);
    }
    else if(mWaypointListMap.value("ahead")->size() == 1)
    {
        slotGenerateWaypoints();
    }
}

void FlightPlannerParticles::slotVehiclePoseChanged(const Pose* const pose)
{
    FlightPlannerInterface::slotVehiclePoseChanged(pose);

    if(!mDialog->processPhysics()) return;

    // Move the particlesystem with the vehicle if so desired and the vehicle moved 5m
    const QVector3D pos = pose->getPosition();

    if(mDialog->processPhysics() && mParticleSystem && mDialog->followVehicle() && pos.distanceToLine(mLastParticleSystemPositionToFollowVehicle, QVector3D()) > 20.0)
    {
        qDebug() << "FlightPlannerParticles::slotVehiclePoseChanged(): vehicle moved far enough, moving particle system...";
        mLastParticleSystemPositionToFollowVehicle = pos;

        const Box3D scanVolume(mScanVolumeMin, mScanVolumeMax);
        const Box3D desiredParticleSystemExtents(QVector3D(
                                               pos.x()-32.0f,
                                               pos.y()-32.0f,
                                               pos.z()-32.0f),
                                           QVector3D(
                                               pos.x()+32.0f,
                                               pos.y()+32.0f,
                                               pos.z()+32.0f));

        const Box3D particleSystemInScanVolume = desiredParticleSystemExtents.tryToKeepWithin(scanVolume);

        mParticleSystem->slotSetVolume(particleSystemInScanVolume.min, particleSystemInScanVolume.max);

        mParticleSystem->slotResetParticles();
    }

    // check for collisions?!
}

void FlightPlannerParticles::slotClearGridWayPointPressure()
{
    // check if we're initialized!
    if(mVboGridMapOfWayPointPressure)
    {
        const quint32 numberOfCells = mSimulationParameters.gridWaypointPressure.cellCount();
        qDebug() << "ParticleSystem::slotClearGridWayPointPressure(): clearing pressure in" << numberOfCells << "cells in VBO" << mVboGridMapOfWayPointPressure;
        quint8* gridMapOfWayPointPressure = (quint8*)mapGLBufferObject(&mCudaVboResourceGridMapOfWayPointPressure);
        cudaSafeCall(cudaMemset(gridMapOfWayPointPressure, 0, sizeof(quint8) * numberOfCells));
        cudaSafeCall(cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfWayPointPressure, 0));
    }
}

void FlightPlannerParticles::showWaypointPressure()
{
    qDebug() << "ParticleSystem::showWaypointPressure():";

    // This pointer is NOT accessible on the host (its in device address space), we need to cudaMemcpy() it from the device first.
    quint8* waypointPressureMapDevice = (quint8*)mapGLBufferObject(&mCudaVboResourceGridMapOfWayPointPressure);

    const quint32 numberOfGridCells = mSimulationParameters.gridWaypointPressure.cellCount();

    quint8* waypointPressureMapHost = new quint8[numberOfGridCells];

    cudaSafeCall(cudaMemcpy(waypointPressureMapHost, waypointPressureMapDevice, sizeof(quint8) * numberOfGridCells, cudaMemcpyDeviceToHost));

    for(int i=0;i<numberOfGridCells;i++)
    {
        const int3 cell = mSimulationParameters.gridWaypointPressure.getCellCoordinate(i);
        Q_ASSERT(i == mSimulationParameters.gridWaypointPressure.getCellHash(mSimulationParameters.gridWaypointPressure.getCellCoordinate(i)) && "grid computation error!");
        if(waypointPressureMapHost[i] > 0)
        {
            qDebug() << "grid cell hash" << i << "at grid-coord" << cell.x << cell.y << cell.z << "and pos" << mSimulationParameters.gridWaypointPressure.getCellCenterQt(cell) << "has waypoint pressure" << waypointPressureMapHost[i];
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

        // After ten seconds, we want to extract waypoints from the pressure that builds up now!
        QTimer::singleShot(10000, this, SLOT(slotGenerateWaypoints()));
    }
}
