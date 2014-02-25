#include "flightplannerparticles.h"
#include "flightplannerparticlesdialog.h"
#include "particlesystem.cuh"
#include "pointcloudcuda.h"
#include "cudahelper.cuh"
#include "basestation.h"

FlightPlannerParticles::FlightPlannerParticles(BaseStation* baseStation, GlWindow *glWidget, PointCloudCuda *pointCloudDense) :
    QObject(baseStation),
    mWayPointsAhead(QColor(255,200,0,200)),
    mWayPointsPassed(QColor(128,128,128,50))
{
    mGlWindow = glWidget;
    mBaseStation = baseStation;
    mPointCloudDense = pointCloudDense;

    mParticleSystem = 0;
    mPathPlanner = 0;
    mVboGridMapOfInformationGainBytes = 0;
    mCudaVboResourceGridMapOfInformationGainBytes = 0;
    mVboGridMapOfInformationGainFloats = 0;
    mCudaVboResourceGridMapOfInformationGainFloats = 0;
    mDeviceGridInformationGainCellWorldPositions = 0;

    mVehiclePoses.reserve(25 * 60 * 20); // enough poses for 20 minutes with 25Hz

    mTimeOfLastDenseCloudReduction = QTime::currentTime();

    mPointCloudColliders = new PointCloudCuda(Box3D(), 512 * 1024, "ColliderCloud");
    mPointCloudColliders->setMinimumPointDistance(0.10f);

    mBaseStation->getGlScene()->slotPointCloudRegister(mPointCloudColliders);

    mSimulationParameters.initialize();

    mDialog = new FlightPlannerParticlesDialog((QWidget*)baseStation);
    connect(mDialog, SIGNAL(renderParticles(bool)), SIGNAL(renderParticles(bool)));
    connect(mDialog, SIGNAL(renderInformationGain(bool)), SIGNAL(renderInformationGain(bool)));
    connect(mDialog, SIGNAL(renderOccupancyGrid(bool)), SIGNAL(renderOccupancyGrid(bool)));
    connect(mDialog, SIGNAL(renderPathPlannerGrid(bool)), SIGNAL(renderPathPlannerGrid(bool)));
    connect(mDialog, &FlightPlannerParticlesDialog::reduceColliderCloud, [=] {mPointCloudColliders->slotReduceEnd();});
    connect(mDialog, &FlightPlannerParticlesDialog::generateWayPoints, this, &FlightPlannerParticles::slotGenerateWaypoints);
    connect(mDialog, &FlightPlannerParticlesDialog::resetInformationGain, this, &FlightPlannerParticles::slotClearGridOfInformationGain);

    // Fill dense cloud => sparse cloud in online mode only.
    if(mBaseStation->getOperatingMode() == OperatingMode::OperatingOnline)
    {
        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Will feeding CloudCollider from CloudDense");
        connect(mPointCloudDense, &PointCloudCuda::pointsInserted, this, &FlightPlannerParticles::slotDenseCloudInsertedPoints);
    }

    // To show pcd filling and capacity in UI
    connect(mPointCloudDense, &PointCloudCuda::parameters, mDialog, &FlightPlannerParticlesDialog::slotSetPointCloudParametersDense);
    connect(mPointCloudColliders, &PointCloudCuda::parameters, mDialog, &FlightPlannerParticlesDialog::slotSetPointCloudParametersSparse);

    mTimerProcessInformationGain.setInterval(5000);
    connect(&mTimerProcessInformationGain, &QTimer::timeout, this, &FlightPlannerParticles::slotProcessInformationGain);

    // When the dialog dis/enables physics processing, start/stop the timer
    mTimerStepSimulation.setInterval(1);
    connect(&mTimerStepSimulation, &QTimer::timeout, this, &FlightPlannerParticles::slotStepSimulation);
    connect(mDialog, &FlightPlannerParticlesDialog::processPhysics, [=](const bool enable)
    {
        if(enable)
        {
            qDebug() << __PRETTY_FUNCTION__ << "starting simulation timer!";
            mTimerStepSimulation.start();
        }
        else
        {
            qDebug() << __PRETTY_FUNCTION__ << "stopping simulation timer!";
            mTimerStepSimulation.stop();
        }
    });

}

void FlightPlannerParticles::slotInitialize()
{
    if(!CudaHelper::isDeviceSupported)
    {
        qDebug() << __PRETTY_FUNCTION__ << "device not supported, not initializing...";
        return;
    }

    qDebug() << __PRETTY_FUNCTION__;

    mPointCloudDense->initialize();
    mPointCloudColliders->initialize();

    mSimulationParameters.initialize();
    mSimulationParameters.gridInformationGain.cells = make_uint3(128, 32, 128);

    mParticleSystem = new ParticleSystem((PointCloudCuda*)mPointCloudDense, (PointCloudCuda*)mPointCloudColliders, &mSimulationParameters); // ParticleSystem will draw its points as colliders from the pointcloud passed here
    mParticleSystem->slotSetParticleRadius(FlightPlannerParticles::initialParticleRadius);

    connect(mDialog, &FlightPlannerParticlesDialog::resetParticles, mParticleSystem, &ParticleSystem::slotResetParticles);
    connect(mDialog, &FlightPlannerParticlesDialog::simulationParameters, mParticleSystem, &ParticleSystem::slotSetSimulationParameters);
    connect(mParticleSystem, &ParticleSystem::parametersChanged, mDialog, &FlightPlannerParticlesDialog::slotSetValuesFromStruct);

    mParticleSystem->slotEmitVboInfoAndParameters(); // so that mDialog gets the parameters set

    mPathPlanner = new PathPlanner(mPointCloudColliders);
    connect(mPathPlanner, &PathPlanner::path, this, &FlightPlannerParticles::slotSetWayPoints);
    connect(mPathPlanner, SIGNAL(message(LogImportance,QString,QString)), this, SIGNAL(message(LogImportance,QString,QString)));
    connect(mPathPlanner, SIGNAL(vboInfoGridOccupancy(quint32,Box3D,Vector3<quint16>)), SIGNAL(vboInfoGridOccupancy(quint32,Box3D,Vector3<quint16>)));
    connect(mPathPlanner, SIGNAL(vboInfoGridPathPlanner(quint32,Box3D,Vector3<quint16>)), SIGNAL(vboInfoGridPathPlanner(quint32,Box3D,Vector3<quint16>)));
    mPathPlanner->initialize();

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

    connect(mParticleSystem, SIGNAL(vboInfoParticles(quint32,quint32,float,Box3D)), SIGNAL(vboInfoParticles(quint32,quint32,float,Box3D)));

    emit vboInfoGridInformationGain(
                mVboGridMapOfInformationGainBytes,
                Box3D(
                    CudaHelper::convert(mSimulationParameters.gridInformationGain.worldMin),
                    CudaHelper::convert(mSimulationParameters.gridInformationGain.worldMax)
                    ),
                Vector3<quint16>(mSimulationParameters.gridInformationGain.cells.x, mSimulationParameters.gridInformationGain.cells.y, mSimulationParameters.gridInformationGain.cells.z)
                );

    // Just to have some defaults...
    slotSetVolumeLocal(Box3D(QVector3D(-32, -6, -32), QVector3D(32, 26, 32)));
    slotSetVolumeGlobal(Box3D(QVector3D(-128, -6, -128), QVector3D(128, 26, 128)));

    emit wayPointListAhead(&mWayPointsAhead);
    emit wayPointListPassed(&mWayPointsPassed);
}

void FlightPlannerParticles::slotShowUserInterface()
{
    mDialog->show();
}

// If the maximum informationGain is higher than threshold, it will generate new waypoints and decrease the pressure.
void FlightPlannerParticles::slotProcessInformationGain()
{
    if(mVboGridMapOfInformationGainBytes == 0 || mVboGridMapOfInformationGainFloats == 0)
    {
        qDebug() << "FlightPlannerParticles::slotProcessInformationGain(): not initialized yet, returning.";
        return;
    }

    if(!mDialog->createWayPoints())
    {
        qDebug() << "FlightPlannerParticles::slotProcessInformationGain(): automatic waypoint generation not enabled, returning.";
        return;
    }

    const quint32 numberOfCells = mSimulationParameters.gridInformationGain.getCellCount();

    // Copy waypoint pressure from VBO into mDeviceGridMapinformationGainSorted
    quint8* gridMapOfInformationGain = (quint8*)CudaHelper::mapGLBufferObject(&mCudaVboResourceGridMapOfInformationGainBytes);
    const quint8 maxPressure = getMaximumInformationGain(gridMapOfInformationGain, numberOfCells);
    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfInformationGainBytes, 0);

    // When using small particles, many will slip through and cause high pressure values
    // When using large particles, few  will slip through and cause low  pressure values
    const int threshold = qBound(1.0, 3.0f * (1.0f/sqrt(mSimulationParameters.particleRadius)), 255.0);
    qDebug() << __PRETTY_FUNCTION__ << "information gain threshold for particle radius" << mSimulationParameters.particleRadius << "is" << threshold;

    if(maxPressure >= threshold)
    {
        qDebug() << __PRETTY_FUNCTION__ << "max pressure" << maxPressure << ">=" << "threshold" << threshold << "- generating waypoints!";
        // Now that we're generating waypoints, we can deactivate the physics processing (and restart when only few waypoints are left)
        mDialog->setProcessPhysicsActive(false);
        mTimerProcessInformationGain.stop();
        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Informationgain found, generating waypoints...");
        slotGenerateWaypoints();
    }
    else
    {
        // Information gain hasn't built up. This indicates that the pointcloud is sufficiently
        // complete. This might be a good time to either move the volume or decrease particle size.

        qDebug() << __PRETTY_FUNCTION__ << "maxPressure is" << maxPressure << "- threshold is" << threshold << ": decreasing particle size...";

        // This will completely reset the particle system. No need to slotResetParticles() etc.
        mParticleSystem->slotSetParticleRadius(mSimulationParameters.particleRadius * 0.9f);

        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), QString("Informationgain not found, shrinking particles to %1m radius").arg(mSimulationParameters.particleRadius, 0, 'f', 2));

        // Using larger particles means that there will be less particles per information gain grid cell, so
        // we will have to use a lower threshold and possibly longer simulation time
        //mTimerProcessInformationGain.setInterval(10000);
    }
}

void FlightPlannerParticles::slotGenerateWaypoints()
{
    if(mVboGridMapOfInformationGainBytes == 0 || mVboGridMapOfInformationGainFloats == 0)
    {
        qDebug() << "FlightPlannerParticles::getRankedWaypoints(): not initialized yet, returning.";
        return;
    }

    emit particleOpacity(1.0f);

    emit processingState(GlScene::FlightPlannerProcessingState::WayPointComputation);

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

        const int numberOfWaypointsToGenerate = 15;

        // Reduce them to the desired number
        while(wayPointsWithHighInformationGain.size() > numberOfWaypointsToGenerate)
            wayPointsWithHighInformationGain.removeLast();

        qDebug() << "FlightPlannerParticles::slotGenerateWaypoints(): after merging and reducing:" << wayPointsWithHighInformationGain.toString();

        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), QString("Generated %1 waypoints").arg(wayPointsWithHighInformationGain.size()));

        // Now use the occupancy grid to move the waypoints into close-by, safe areas
        mPathPlanner->moveWayPointsToSafety(&wayPointsWithHighInformationGain, true);

        // Sort them to the shortest path, honoring the vehicle's current location.
        wayPointsWithHighInformationGain.sortToShortestPath(getLastKnownVehiclePose().getPosition());

        qDebug() << __PRETTY_FUNCTION__ << "after sort to shortest path:" << wayPointsWithHighInformationGain.toString();

        mPathPlanner->slotComputePath(vehiclePosition, wayPointsWithHighInformationGain);

        qDebug() << __PRETTY_FUNCTION__ << "setting processing state to idle";
        emit processingState(GlScene::FlightPlannerProcessingState::Idle);
    }
    else
    {
        qDebug() << __PRETTY_FUNCTION__ << "waypointlist is empty, informationGain is probably also empty, NOT stopping physics!";
        emit message(LogImportance::Warning, "flightplanner", "cannot generate waypoints, is there information gain?");
    }
}

FlightPlannerParticles::~FlightPlannerParticles()
{
    delete mParticleSystem;

    if(CudaHelper::isDeviceSupported)
    {
        cudaSafeCall(cudaFree(mDeviceGridInformationGainCellWorldPositions));
        //cudaSafeCall(cudaFree(mVboGridMapOfInformationGainFloat));
        if(mCudaVboResourceGridMapOfInformationGainBytes) cudaSafeCall(cudaGraphicsUnregisterResource(mCudaVboResourceGridMapOfInformationGainBytes));
        if(mCudaVboResourceGridMapOfInformationGainFloats) cudaSafeCall(cudaGraphicsUnregisterResource(mCudaVboResourceGridMapOfInformationGainFloats));
    }

    if(mVboGridMapOfInformationGainBytes) OpenGlUtilities::deleteVbo(mVboGridMapOfInformationGainBytes);
    if(mVboGridMapOfInformationGainFloats) OpenGlUtilities::deleteVbo(mVboGridMapOfInformationGainFloats);
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
    if(mTimerStepSimulation.isActive() || mTimerProcessInformationGain.isActive())
    {
        //qDebug() << "FlightPlannerParticles::slotNewScanFused(): not accepting points while processing.";
        return;
    }

    // Insert all points into mPointCloudDense
    mPointCloudDense->slotInsertPoints4(points, count);

    if(!CudaHelper::isDeviceSupported) return;

    // Reduce the dense cloud only in online mode, when we want to pass the resulting sparser points to pointcloudsparse.
    // If working offline, we just want to display the scanned data using the dense cloud and its ringbuffer
    if(mTimeOfLastDenseCloudReduction.msecsTo(QTime::currentTime()) > 2000 && mBaseStation->getOperatingMode() == OperatingMode::OperatingOnline)
    {
        qDebug() << "FlightPlannerParticles::slotNewScanFused(): reducing points (queued and end) in dense cloud.";
        mPointCloudDense->slotReduceQueue();
        mPointCloudDense->slotReduceEnd(200 * 1000); // reduce the last 200k points (limited for interactive framerates)
        mTimeOfLastDenseCloudReduction = QTime::currentTime();
    }

    if(
               mBaseStation->getOperatingMode() == OperatingMode::OperatingOnline
            && mWayPointsAhead.size() == 0
            && mPointCloudDense->getNumberOfPointsStored() > 30000
            && mDialog->createWayPoints())
    {
        // This is the first time that points were inserted (and we have no waypoints) - start the physics processing!
        qDebug() << "FlightPlannerParticles::slotNewScanFused(): 50k points are present, reducing and starting wpt generation!";
        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), QString("No waypoints present, starting particle simulation..."));
        mTimeOfLastDenseCloudReduction = QTime::currentTime();
        mPointCloudDense->slotReduceQueue();
        slotStartWayPointGeneration();
    }

    emit suggestVisualization();
}

void FlightPlannerParticles::slotSetVolumeGlobal(const Box3D volume)
{
    qDebug() << __PRETTY_FUNCTION__ << volume;
    // We're being told to change the global volume

    mVolumeGlobal = volume;

    // If the local volume is not enclosed anymore, try to make it fit!
    if(!mVolumeGlobal.containsAllOf(&mVolumeLocal))
    {
        slotSetVolumeLocal(mVolumeLocal);
    }

    emit volumeGlobal(&mVolumeGlobal);
}

void FlightPlannerParticles::slotSetVolumeLocal(const Box3D volume)
{
    qDebug() << __PRETTY_FUNCTION__ << volume;

    // We're being told to change the local volume that is used to bound waypoint generation.
    // This means we need to move the particle system, the pathplanner structures etc.
    slotClearGridOfInformationGain();

    mVolumeLocal = volume.tryToKeepWithin(mVolumeGlobal);
    emit volumeLocal(&mVolumeLocal);

    mSimulationParameters.gridInformationGain.worldMin = CudaHelper::convert(mVolumeLocal.min);
    mSimulationParameters.gridInformationGain.worldMax = CudaHelper::convert(mVolumeLocal.max);

    // We set this cloud's BBOX, the pathplanner will follow automatically
    mPointCloudColliders->setBoundingBox(mVolumeLocal);

    // Will completely reset the particleSystem
    mParticleSystem->slotSetVolume(mVolumeLocal);
    emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), QString("VolumeLocal changed, resetting particle-size to %1m.").arg(initialParticleRadius, 0, 'f', 2));
    mParticleSystem->slotSetParticleRadius(FlightPlannerParticles::initialParticleRadius);

    if(CudaHelper::isDeviceSupported)
    {
        copyParametersToGpu(&mSimulationParameters);

        emit vboInfoGridInformationGain(
                    mVboGridMapOfInformationGainBytes,
                    Box3D(
                        CudaHelper::convert(mSimulationParameters.gridInformationGain.worldMin),
                        CudaHelper::convert(mSimulationParameters.gridInformationGain.worldMax)
                        ),
                    Vector3<quint16>(mSimulationParameters.gridInformationGain.cells.x, mSimulationParameters.gridInformationGain.cells.y, mSimulationParameters.gridInformationGain.cells.z)
                    );
    }


    qDebug() << __PRETTY_FUNCTION__ << volume << "- done.";
}

void FlightPlannerParticles::slotVehiclePoseChanged(const Pose* const pose)
{
    mVehiclePoses.append(*pose);

    //why this?? if(!mDialog->isProcessPhysicsActive()) return;

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

        slotSetVolumeLocal(desiredParticleSystemExtents);
    }

    // check for collisions?!
}

void FlightPlannerParticles::slotSetWayPoints(const QList<WayPoint>* const wayPointList, const WayPointListSource source)
{
    qDebug() << __PRETTY_FUNCTION__ << "saving"
             << wayPointList->size() << "waypoints from"
             << static_cast<quint8>(source)
             << "- first one:" << (wayPointList->size() > 0 ? wayPointList->at(0).toString() : "[null]");

    mWayPointsAhead.setList(wayPointList);

    // Tell others about our new waypoints!
    emit wayPoints(mWayPointsAhead.list(), source);

    // Pointcloud calling this method might still have its buffer mapped,
    // but suggestVisualization() does render, not immediately.
    emit suggestVisualization();
}

const QList<WayPoint>* const FlightPlannerParticles::getWayPoints()
{
    return mWayPointsAhead.list();
}

void FlightPlannerParticles::slotWayPointReached(const WayPoint& wpt)
{
    qDebug() << __PRETTY_FUNCTION__ << "rover->baseconnection->flightplanner waypoint" << wpt << "reached, so appending first element of mWayPointsAhead"
             << (mWayPointsAhead.size() ? mWayPointsAhead.at(0).toString() : "[empty list]")
             << "to mWayPointsPassed";

    if(mWayPointsAhead.size())
    {
        if(mWayPointsAhead.at(0) != wpt)
            qDebug() << __PRETTY_FUNCTION__ << "error, the reached waypoint was not the first in mWayPointsAhead!";

        mWayPointsPassed.append(mWayPointsAhead.takeAt(0));
    }
    else
    {
        qDebug() << __PRETTY_FUNCTION__ << "error, mWayPointsAhead is empty, but rover reached wpt" << wpt.toString() << "- appending anyways.";
        mWayPointsPassed.append(wpt);
    }

    emit message(
                Information,
                QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                QString("Reached waypoint %1 %2 %3").arg(wpt.x()).arg(wpt.y()).arg(wpt.z()));

    emit wayPoints(mWayPointsAhead.list(), WayPointListSource::WayPointListSourceFlightPlanner);
    qDebug() << __PRETTY_FUNCTION__ << "rover->baseconnection->flightplanner waypoint reached, emitted wayPoints())";
    emit suggestVisualization();

    // When no waypoints are left, generate some!
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
    qDebug() << __PRETTY_FUNCTION__ << "starting generation, setting processing state to ParticleSimulation";
    mTimerProcessInformationGain.start();

    slotClearGridOfInformationGain();

    // Make sure we visualize the process
    emit processingState(GlScene::FlightPlannerProcessingState::ParticleSimulation);

    // This will make the dialog emit processPhysics() signal, which is connected to a lambda above, starting the stepSimulation-timer.
    mDialog->setProcessPhysicsActive(true);

    // This will make the local bbox follow the vehicle!
    const Pose p = mVehiclePoses.last();
    slotVehiclePoseChanged(&p);

    mParticleSystem->slotResetParticles();
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
    else
    {
        qDebug() << __PRETTY_FUNCTION__ << "couldn't clear information gain, VBO is not initialized yet!";
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

void FlightPlannerParticles::slotDenseCloudInsertedPoints(PointCloudCuda*const pointCloudSource, const quint32& firstPointToReadFromSrc, quint32 numberOfPointsToCopy)
{
    // This is called when the dense cloud inserted points. When points come in, we make sure this happens every 2 seconds,
    // so that waypoint safety can be checked as often.
    qDebug() << __PRETTY_FUNCTION__ << "cloud" << pointCloudSource << "inserted points. First point:" << firstPointToReadFromSrc << "number:" << numberOfPointsToCopy;

    mPointCloudColliders->slotInsertPoints(pointCloudSource, firstPointToReadFromSrc, numberOfPointsToCopy);

    // Check to make sure that all waypoints are safe (i.e. their gridcells are not occupied)
    if(mDialog->checkWayPointSafety() && mWayPointsAhead.size())
    {
        emit processingState(GlScene::FlightPlannerProcessingState::WayPointChecking);
        const qint32 offendingWayPointIndex = mPathPlanner->checkWayPointSafety(&mWayPointsAhead);

        if(offendingWayPointIndex < 0)
        {
            // Todo: wait a couple of frames before emitting this. There's no QTimer::singleShot(lambda), though :|
            qDebug() << __PRETTY_FUNCTION__ << "offendingWayPointIndex is" << offendingWayPointIndex << "- all waypoints are safe, continuing...";
            QTimer::singleShot(250, this, SLOT(slotSetProcessingStateToIdle()));
        }
        else if(offendingWayPointIndex > 2)
        {
            qDebug() << __PRETTY_FUNCTION__ << "offendingWayPointIndex is" << offendingWayPointIndex << "- problems ahead, but still some headroom...";
            QTimer::singleShot(250, this, SLOT(slotSetProcessingStateToIdle()));
        }
        else
        {
            qDebug() << __PRETTY_FUNCTION__ << "some of the following waypoints are now in occupied territory, moving them from:" << mWayPointsAhead.toString();
            // Retrieve all waypoints with informatio ngain, re-move them to safe positions and re-plan the path.
            emit message(LogImportance::Warning, "FlightPlanner", "waypoints are now colliding with geometry. Stopping rover and replanning path.");

            // Re-generate waypoints from old information gain. This is problematic,
            // as we will re-approach waypoints that we've already visited.
            //slotGenerateWaypoints();

            // Completey restart waypoint generation. Also problematic, because particle simulation takes time and it
            // sometimes seems like fly back where we came from. Reversing flight-direction might be smart, but seems
            // clumsy
            //QList<WayPoint> empty;
            //slotSetWayPoints(&empty, WayPointListSource::WayPointListSourceFlightPlanner);
            //slotStartWayPointGeneration();

            // wayPointsAhead is a pointer to flightplanner's real list. As soon as we emit an empty path,
            // it will also become empty! So, create a copy.
            WayPointList newWayPointsWithHighInformationGain;

            for(int i=0;i<mWayPointsAhead.size();i++)
            {
                const WayPoint wpt = mWayPointsAhead.at(i);

                if(std::abs(wpt.informationGain) > 0.0f)
                    newWayPointsWithHighInformationGain.append(wpt);
            }

            // Set an empty list, so we stop the rover!
            QList<WayPoint> wpl;
            slotSetWayPoints(&wpl, WayPointListSource::WayPointListSourceFlightPlanner);

            if(!newWayPointsWithHighInformationGain.size())
            {
                qDebug() << __PRETTY_FUNCTION__ << "no waypoints left after checking for safety, cannot compute path!";
                QTimer::singleShot(100, this, SLOT(slotSetProcessingStateToIdle()));
                return;
            }

            mPathPlanner->moveWayPointsToSafety(&newWayPointsWithHighInformationGain, false);

            if(!newWayPointsWithHighInformationGain.size())
            {
                qDebug() << __PRETTY_FUNCTION__ << "no waypoints left after moving remaining waypoints to safety, cannot compute path!";
                QTimer::singleShot(100, this, SLOT(slotSetProcessingStateToIdle()));
                return;
            }

            mPathPlanner->slotComputePath(getLastKnownVehiclePose().getPosition(), newWayPointsWithHighInformationGain);

            qDebug() << __PRETTY_FUNCTION__ << "waypoitns changed to:" << mWayPointsAhead.toString();
        }
    }
}

void FlightPlannerParticles::slotStepSimulation()
{
    if(!mParticleSystem && CudaHelper::isDeviceSupported)
    {
        mParticleSystem->slotResetParticles();
        slotInitialize();
    }

    if(mPointCloudColliders->getNumberOfPointsStored() < 1)
    {
        qDebug() << __PRETTY_FUNCTION__ << "will not collide particles against 0 colliders, disablig particle simulation";
        mDialog->setProcessPhysicsActive(false);
        return;
    }

    // Provide the particle system a pointer to our waypoint-pressure-gridmap (in device address space)
    quint8 *deviceGridMapOfinformationGain = (quint8*)CudaHelper::mapGLBufferObject(&mCudaVboResourceGridMapOfInformationGainBytes);

    for(float timeStepped = 0.0f; timeStepped < mSimulationParameters.timeStepOuter; timeStepped += mSimulationParameters.timeStepInner)
    {
        mParticleSystem->update(deviceGridMapOfinformationGain);
    }

    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfInformationGainBytes, 0);

    // Set the particle's opacity!
    const int remainingTime = mTimerProcessInformationGain.remainingTime();
    // If remainingTime is -1 (for whatever reason), make sure we show the particles completely
    float percentLeft = remainingTime < 0 ? mTimerProcessInformationGain.interval() : remainingTime;
    percentLeft /= (float)mTimerProcessInformationGain.interval();
    emit particleOpacity(percentLeft);
    if(remainingTime % 10 == 0) qDebug() << __PRETTY_FUNCTION__ << "percentLeft:" << percentLeft;

    static int iteration = 0;
    if(iteration++ % 100 == 0)
        emit suggestVisualization();
}

void FlightPlannerParticles::slotSetProcessingStateToIdle()
{
    qDebug() << __PRETTY_FUNCTION__;
    emit processingState(GlScene::FlightPlannerProcessingState::Idle);
}
