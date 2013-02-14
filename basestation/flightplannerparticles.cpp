#include <GL/glew.h>
#include "flightplannerparticles.h"
#include "flightplannerparticlesdialog.h"
#include "particlesystem_cuda.cuh"
#include "pointcloudcuda.h"

FlightPlannerParticles::FlightPlannerParticles(QWidget* parentWidget, GlWidget *glWidget, PointCloud *pointcloud) : FlightPlannerInterface(parentWidget, glWidget, pointcloud)
{
    mParticleSystem = 0;
    mParticleRenderer = 0;
    mProcessPhysics = true;

    mVboGridMapOfWayPointPressure = 0;
    mDeviceGridMapCellWorldPositions = 0;

    // register dense pointcloud for rendering. Might be moved to base class c'tor
    mGlWidget->slotPointCloudRegister(mPointCloudDense);

    // For every point in this cloud, particlesystem will have to create
    // 2 32bit ints to map grid cell index to a particle index.
    mPointCloudColliders = new PointCloudCuda(
                mPointCloudDense->getBoundingBoxMin(),
                mPointCloudDense->getBoundingBoxMax(),
                256 * 1024);

    mPointCloudColliders->setMinimumPointDistance(0.5f);

    mPointCloudColliders->setColor(QColor(255,255,255,64));

    mGlWidget->slotPointCloudRegister(mPointCloudColliders);

    mDialog = new FlightPlannerParticlesDialog(parentWidget);
}

void FlightPlannerParticles::slotInitializeWaypointPressureGrid()
{
    const quint32 numberOfCellsInScanVolume = mSimulationParameters.scanVolumeGridSize.x * mSimulationParameters.scanVolumeGridSize.y * mSimulationParameters.scanVolumeGridSize.z;

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
    cudaSafeCall(cudaMalloc((void**)&mDeviceGridMapCellWorldPositions, sizeof(float4) * numberOfCellsInScanVolume));

    // Will contain QVector4Ds of worldpos of every gridcell, sorted by waypoint pressure
    cudaSafeCall(cudaMalloc((void**)&mDeviceGridMapCellWorldPositions, sizeof(float) * 4 * numberOfCellsInScanVolume));
}

void FlightPlannerParticles::slotInitialize()
{
    qDebug() << "FlightPlannerParticles::slotInitialize()";

    mSimulationParameters.scanVolumeGridSize = make_uint3(200, 20, 200);
    slotSetScanVolume(QVector3D(-128, -2, -128), QVector3D(128, 30, 128));
    slotInitializeWaypointPressureGrid();

    mPointCloudColliders->slotInitialize();

    connect(mPointCloudDense, SIGNAL(pointsInserted(VboInfo*const,quint32,quint32)), mPointCloudColliders, SLOT(slotInsertPoints(VboInfo*const,quint32,quint32)));

    mShaderProgramGridLines = new ShaderProgram(this, "shader-default-vertex.c", "", "shader-default-fragment.c");

    mParticleSystem = new ParticleSystem(mPointCloudColliders, &mSimulationParameters); // ParticleSystem will draw its points as colliders from the pointcloud passed here
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
                QVector3D(mSimulationParameters.scanVolumeWorldMin.x, mSimulationParameters.scanVolumeWorldMin.y, mSimulationParameters.scanVolumeWorldMin.z),
                QVector3D(mSimulationParameters.scanVolumeWorldMax.x, mSimulationParameters.scanVolumeWorldMax.y, mSimulationParameters.scanVolumeWorldMax.z),
                Vector3i(mSimulationParameters.scanVolumeGridSize.x, mSimulationParameters.scanVolumeGridSize.y, mSimulationParameters.scanVolumeGridSize.z)
                );

    mParticleSystem->slotSetParticleCount(16384);
    mParticleSystem->slotSetParticleRadius(0.5f); // balance against mOctreeCollisionObjects.setMinimumPointDistance() above
    mParticleSystem->slotSetDefaultParticlePlacement(ParticleSystem::ParticlePlacement::PlacementFillSky);

    connect(mDialog, SIGNAL(deleteWayPoints()), SLOT(slotWayPointsClear()));
    connect(mDialog, SIGNAL(generateWayPoints()), SLOT(slotGenerateWaypoints()));
    connect(mDialog, SIGNAL(resetParticles()), mParticleSystem, SLOT(slotResetParticles()));
    connect(mDialog, SIGNAL(resetWaypointPressure()), SLOT(slotClearGridWayPointPressure()));
    connect(mDialog, SIGNAL(simulationParameters(const SimulationParameters*)), mParticleSystem, SLOT(slotSetSimulationParametersFromUi(const SimulationParameters*)));

    connect(mDialog, SIGNAL(processPhysicsChanged(bool)), SLOT(slotProcessPhysics(bool)));
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

    QVector<QVector4D> waypoints(200);
    if(getRankedWaypoints(waypoints.data(), waypoints.size()))
    {
        WayPointList* const wpl = mWaypointListMap.value("ahead");
        wpl->clear();

        for(int i=0;i<waypoints.size();i++)
        {
            QVector4D wp = waypoints[i];

            // If we ask for more waypoints than cells with wp-pressure > 0.0, we'll get useless candidates!
            if(wp.w() > 0.1f)
                wpl->append(WayPoint(wp.toVector3D() + QVector3D(0.0f, 5.0f, 0.0f), wp.w(), WayPoint::Purpose::SCAN));
        }

        qDebug() << "FlightPlannerParticles::slotGenerateWaypoints(): generated" << wpl->size() << "waypoints";

        // Merge waypoints if they're closer than 2.0 meters
        wpl->mergeCloseWaypoints(5.0f);

        // Sort them to the shortest path, honoring the vehicle's current location.
        wpl->sortToShortestPath(getLastKnownVehiclePose().getPosition());

        qDebug() << "FlightPlannerParticles::slotGenerateWaypoints(): after merging, there are" << wpl->size() << "waypoints left";

        emit wayPoints(wpl->list());
        emit wayPointsSetOnRover(wpl->list());
    }
}

void FlightPlannerParticles::slotCreateSafePathToNextWayPoint()
{

}

FlightPlannerParticles::~FlightPlannerParticles()
{
    delete mParticleRenderer;
    delete mParticleSystem;


    cudaSafeCall(cudaFree(mDeviceGridMapCellWorldPositions));
    cudaSafeCall(cudaFree(mDeviceGridMapWayPointPressureSorted));
    cudaSafeCall(cudaGraphicsUnregisterResource(mCudaVboResourceGridMapOfWayPointPressure));
    glDeleteBuffers(1, (const GLuint*)&mVboGridMapOfWayPointPressure);
}


void FlightPlannerParticles::slotNewScanData(const QVector<QVector3D>* const pointList, const QVector3D* const scannerPosition)
{
    // Insert all points into mPointCloudDense
    mPointCloudDense->slotInsertPoints(pointList);

    emit suggestVisualization();
}

void FlightPlannerParticles::slotVisualize()
{
    FlightPlannerInterface::slotVisualize();

    if(!mParticleSystem)
        slotInitialize();

    if(mParticleSystem)
    {
        if(mProcessPhysics)
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

    mSimulationParameters.scanVolumeWorldMin.x = mScanVolumeMin.x();
    mSimulationParameters.scanVolumeWorldMin.y = mScanVolumeMin.y();
    mSimulationParameters.scanVolumeWorldMin.z = mScanVolumeMin.z();
    mSimulationParameters.scanVolumeWorldMax.x = mScanVolumeMax.x();
    mSimulationParameters.scanVolumeWorldMax.y = mScanVolumeMax.y();
    mSimulationParameters.scanVolumeWorldMax.z = mScanVolumeMax.z();

    copyParametersToGpu(&mSimulationParameters);

    emit vboInfoGridWaypointPressure(
                mVboGridMapOfWayPointPressure,
                QVector3D(mSimulationParameters.scanVolumeGridSize.x, mSimulationParameters.scanVolumeGridSize.y, mSimulationParameters.scanVolumeGridSize.z),
                QVector3D(mSimulationParameters.scanVolumeGridSize.x, mSimulationParameters.scanVolumeGridSize.y, mSimulationParameters.scanVolumeGridSize.z),
                Vector3i(mSimulationParameters.scanVolumeGridSize.x, mSimulationParameters.scanVolumeGridSize.y, mSimulationParameters.scanVolumeGridSize.z)
                );
}

void FlightPlannerParticles::slotWayPointReached(const WayPoint wpt)
{
    FlightPlannerInterface::slotWayPointReached(wpt);

    // When only two waypoints are left, we want to clear the ParticleSystem's grid of wayppint pressure.
    // This way, the particlesystem has the duration of the two remaining waypoints to create new waypoints
    if(mWaypointListMap.value("ahead")->size() == 2)
        slotClearGridWayPointPressure();
}

void FlightPlannerParticles::slotVehiclePoseChanged(const Pose* const pose)
{
    FlightPlannerInterface::slotVehiclePoseChanged(pose);

    // Move the particlesystem with the vehicle if so desired and the vehicle moved 5m
    const QVector3D pos = pose->getPosition();
    if(mDialog->followVehicle() && pos.distanceToLine(mParticleSystem->getWorldCenter(), QVector3D()) > 5.0f)
    {
        qDebug() << "FlightPlannerParticles::slotVehiclePoseChanged(): vehicle moved 5m, moving particle system...";
        if(mParticleSystem)
        {
            mParticleSystem->slotSetVolume(
                    QVector3D(
                        pos.x()-32.0f,
                        pos.y()-32.0f,
                        pos.z()-32.0f),
                    QVector3D(
                        pos.x()+32.0f,
                        pos.y()+32.0f,
                        pos.z()+32.0f)
                        );
        }
    }

    // check for collisions?!
}

void FlightPlannerParticles::slotClearGridWayPointPressure()
{
    // check if we're initialized!
    if(mVboGridMapOfWayPointPressure)
    {
        const quint32 numberOfCells = mSimulationParameters.particleSystemGridSize.x * mSimulationParameters.particleSystemGridSize.y * mSimulationParameters.particleSystemGridSize.z;

        quint8* gridMapOfWayPointPressure = (quint8*)mapGLBufferObject(&mCudaVboResourceGridMapOfWayPointPressure);
        cudaSafeCall(cudaMemset(gridMapOfWayPointPressure, 0, sizeof(quint8) * numberOfCells));
        cudaSafeCall(cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfWayPointPressure, 0));
    }
}


void FlightPlannerParticles::showWaypointPressure()
{
    qDebug() << "ParticleSystem::showWaypointPressure():";
/*
    // This pointer is NOT accessible on the host (its in device address space), we need to cudaMemcpy() it from the device first.
    quint8* waypointPressureMapDevice = (quint8*)mapGLBufferObject(&mCudaVboResourceGridMapOfWayPointPressure);

    const quint32 numberOfGridCells = mSimulationParameters.particleSystemGridSize.x * mSimulationParameters.particleSystemGridSize.y * mSimulationParameters.particleSystemGridSize.z;

    quint8* waypointPressureMapHost = new quint8[numberOfGridCells];

    cudaSafeCall(cudaMemcpy(waypointPressureMapHost, waypointPressureMapDevice, sizeof(quint8) * numberOfGridCells, cudaMemcpyDeviceToHost));

    for(int i=0;i<numberOfGridCells;i++)
    {
        Q_ASSERT(i == getGridCellHash(getGridCellCoordinate(i)));
        const Vector3i cell = getGridCellCoordinate(i);
        if(waypointPressureMapHost[i] > 0)
        {
            qDebug() << "grid cell hash" << i << "at grid-coord" << cell.x << cell.y << cell.z << "and pos" << getGridCellCenter(cell) << "has waypoint pressure" << waypointPressureMapHost[i];
        }
    }

    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfWayPointPressure, 0);
    delete waypointPressureMapHost;
    */
}


bool FlightPlannerParticles::getRankedWaypoints(QVector4D* const waypoints, const quint16 numberOfWaypointsRequested)
{
    if(mVboGridMapOfWayPointPressure)
    {
        qDebug() << "FlightPlannerParticles::getRankedWaypoints(): not initialized yet, returning.";
        return false;
    }

    qDebug() << "FlightPlannerParticles::getRankedWaypoints():";

    Q_ASSERT(numberOfWaypointsRequested <= mSimulationParameters.particleSystemGridSize.x * mSimulationParameters.particleSystemGridSize.y * mSimulationParameters.particleSystemGridSize.z);

    const quint32 numberOfCells = mSimulationParameters.particleSystemGridSize.x * mSimulationParameters.particleSystemGridSize.y * mSimulationParameters.particleSystemGridSize.z;

    // Copy waypoint pressure from VBO into mDeviceGridMapWayPointPressureSorted
    quint8* gridMapOfWayPointPressure = (quint8*)mapGLBufferObject(&mCudaVboResourceGridMapOfWayPointPressure);
    cudaMemcpy(mDeviceGridMapWayPointPressureSorted, gridMapOfWayPointPressure, sizeof(quint8) * numberOfCells, cudaMemcpyDeviceToDevice);
    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfWayPointPressure, 0);

    // Update constants
    copyParametersToGpu(&mSimulationParameters);

    // Fill mDeviceGridMapCellWorldPositions - this might be done only once and then copied lateron (just like the waypoint pressure above)
    fillGridMapCellWorldPositions(mDeviceGridMapCellWorldPositions, numberOfCells);

    // Sort mDeviceGridMapWayPointPressureSorted => mDeviceGridMapCellWorldPositions according to the keys DESC
    sortGridMapWayPointPressure(mDeviceGridMapWayPointPressureSorted, mDeviceGridMapCellWorldPositions, numberOfCells, numberOfWaypointsRequested);

    // Copy the @count cells with the highest waypoint pressure into @waypoints
    cudaMemcpy(waypoints, mDeviceGridMapCellWorldPositions, sizeof(QVector4D) * numberOfWaypointsRequested, cudaMemcpyDeviceToHost);

    return true;
}
