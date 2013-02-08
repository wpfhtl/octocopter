#include <GL/glew.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include "flightplannerparticles.h"
#include "flightplannerparticlesdialog.h"
#include "pointcloudcuda.h"

FlightPlannerParticles::FlightPlannerParticles(QWidget* parentWidget, GlWidget *glWidget, PointCloud *pointcloud) : FlightPlannerInterface(parentWidget, glWidget, pointcloud)
{
    mParticleSystem = 0;
    mParticleRenderer = 0;
    mVboGridLines = 0;

    mProcessPhysics = true;

    // register dense pointcloud for rendering. Might be moved to base class c'tor
    mGlWidget->slotPointCloudRegister(mPointCloudDense);

    mPointCloudDense->mName = "Dense";

    // For every point in this cloud, particlesystem will have to create
    // 2 32bit ints to map grid cell index to a particle index.
    mPointCloudColliders = new PointCloudCuda(
                mPointCloudDense->getBoundingBoxMin(),
                mPointCloudDense->getBoundingBoxMax(),
                256 * 1024);

    mPointCloudColliders->mName = "Colliders";

//    mPointCloudColliders->setGridSize(64, 32, 64);

    mPointCloudColliders->setMinimumPointDistance(0.5f);

    mPointCloudColliders->setColor(QColor(128,128,128,64));

    mGlWidget->slotPointCloudRegister(mPointCloudColliders);

    mDialog = new FlightPlannerParticlesDialog(parentWidget);
}

void FlightPlannerParticles::slotInitialize()
{
    qDebug() << "FlightPlannerParticles::slotInitialize()";

    mPointCloudColliders->slotInitialize();

    connect(mPointCloudDense, SIGNAL(pointsInserted(VboInfo*const,quint32,quint32)), mPointCloudColliders, SLOT(slotInsertPoints(VboInfo*const,quint32,quint32)));

    mShaderProgramGridLines = new ShaderProgram(this, "shader-default-vertex.c", "", "shader-default-fragment.c");

    mParticleSystem = new ParticleSystem(mPointCloudColliders); // ParticleSystem will draw its points as colliders from the pointcloud passed here
    mParticleRenderer = new ParticleRenderer;

    connect(mParticleSystem, SIGNAL(particleRadiusChanged(float)), mParticleRenderer, SLOT(slotSetParticleRadius(float)));
    connect(mParticleSystem, SIGNAL(vboInfoParticles(quint32,quint32,quint32)), mParticleRenderer, SLOT(slotSetVboInfoParticles(quint32,quint32,quint32)));

    connect(
                mParticleSystem,
                SIGNAL(vboInfoGridWaypointPressure(quint32,QVector3D,QVector3D,Vector3i)),
                mParticleRenderer,
                SLOT(slotSetVboInfoGridWaypointPressure(quint32,QVector3D,QVector3D,Vector3i))
                );

    mParticleSystem->slotSetParticleCount(16384);
    mParticleSystem->slotSetParticleRadius(0.5f); // balance against mOctreeCollisionObjects.setMinimumPointDistance() above
    mParticleSystem->slotSetDefaultParticlePlacement(ParticleSystem::ParticlePlacement::PlacementFillSky);
//    mParticleSystem->slotSetVolume(mScanVolumeMin, mScanVolumeMax);

    connect(mDialog, SIGNAL(deleteWayPoints()), SLOT(slotWayPointsClear()));
    connect(mDialog, SIGNAL(generateWayPoints()), SLOT(slotGenerateWaypoints()));
    connect(mDialog, SIGNAL(resetParticles()), mParticleSystem, SLOT(slotResetParticles()));
    connect(mDialog, SIGNAL(resetWaypointPressure()), mParticleSystem, SLOT(slotClearGridWayPointPressure()));
    connect(mDialog, SIGNAL(simulationParameters(const SimulationParameters*)), mParticleSystem, SLOT(slotSetSimulationParametersFromUi(const SimulationParameters*)));

    connect(mDialog, SIGNAL(processPhysicsChanged(bool)), SLOT(slotProcessPhysics(bool)));
    connect(mDialog, SIGNAL(showParticlesChanged(bool)), mParticleRenderer, SLOT(slotSetRenderParticles(bool)));
    connect(mDialog, SIGNAL(showWaypointPressureChanged(bool)), mParticleRenderer, SLOT(slotSetRenderWaypointPressure(bool)));
}

void FlightPlannerParticles::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_1)
    {
        mProcessPhysics = !mProcessPhysics;
    }
    else if(event->key() == Qt::Key_2 && mParticleSystem)
    {
        mParticleRenderer->slotSetRenderParticles(!mParticleRenderer->getRenderParticles());
    }
    else if(event->key() == Qt::Key_3 && mParticleSystem)
    {
        mParticleRenderer->slotSetRenderWaypointPressure(!mParticleRenderer->getRenderWaypointPressure());
    }
}

void FlightPlannerParticles::slotShowUserInterface()
{
    // Just show the dialog for generating-options
    mDialog->show();
}

void FlightPlannerParticles::slotGenerateWaypoints()
{

    QVector<QVector4D> waypoints(200);
    if(mParticleSystem->getRankedWaypoints(waypoints.data(), waypoints.size()))
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
//    delete mPointCloudColliders;
    cudaDeviceReset();
}


void FlightPlannerParticles::slotNewScanData(const QVector<QVector3D>* const pointList, const QVector3D* const scannerPosition)
{
    // Insert all points into mPointCloudDense
    mPointCloudDense->slotInsertPoints(pointList);

    // Insert all points into mPointCloudColliders. Lateron, caller or callee could
    // insert points only when the previous point is at least minimumDistance away
    // from the current point.
//    mPointCloudColliders->slotInsertPoints(pointList);

    //qDebug() << "FlightPlannerParticles::slotNewScanData(): appended" << pointList->size() << "points to cloud";

    emit suggestVisualization();
}


//void FlightPlannerParticles::insertPoint(const LidarPoint *const point)
//{
//    if(mPointCloudColliders)
//        mPointCloudColliders->insertPoint(new LidarPoint(*point));
//}

void FlightPlannerParticles::slotVisualize()
{
    FlightPlannerInterface::slotVisualize();

    if(!mParticleSystem)
        slotInitialize();

    // Draw the grid!
    if(mParticleSystem)
    {
        if(mVboGridLines)
        {
            mShaderProgramGridLines->bind();
            mShaderProgramGridLines->setUniformValue("useFixedColor", true);
            mShaderProgramGridLines->setUniformValue("fixedColor", QVector4D(0.2f, 0.2f, 0.2f, 0.1f));
            glBindBuffer(GL_ARRAY_BUFFER, mVboGridLines);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0); // positions
            glEnable(GL_BLEND);
            glEnable(GL_DEPTH_TEST);
            glDrawArrays(GL_LINES, 0, (mParticleSystem->gridCells().x+1) * (mParticleSystem->gridCells().y+1) * (mParticleSystem->gridCells().z+1));
            glDisable(GL_BLEND);
            glDisableVertexAttribArray(0);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            mShaderProgramGridLines->release();
        }

        if(mProcessPhysics)
            mParticleSystem->update(0.1f);

        mParticleRenderer->render();
    }

}

void FlightPlannerParticles::slotSetScanVolume(const QVector3D min, const QVector3D max)
{
    FlightPlannerInterface::slotSetScanVolume(min, max);

    if(mParticleSystem)
    {
        mParticleSystem->slotSetVolume(mScanVolumeMin, mScanVolumeMax);

        // Fill or re-fill the VBO that contains the data for rendering the grid
        QVector<QVector4D> lineData;

        for(int x=0;x<=mParticleSystem->gridCells().x;x++)
        {
            for(int y=0;y<=mParticleSystem->gridCells().y;y++)
            {
                lineData.append(QVector4D(
                                mScanVolumeMin.x() + x * ((mScanVolumeMax.x()-mScanVolumeMin.x())/mParticleSystem->gridCells().x),
                                mScanVolumeMin.y() + y * ((mScanVolumeMax.y()-mScanVolumeMin.y())/mParticleSystem->gridCells().y),
                                mScanVolumeMin.z(),
                                1.0f));

                lineData.append(QVector4D(
                                mScanVolumeMin.x() + x * ((mScanVolumeMax.x()-mScanVolumeMin.x())/mParticleSystem->gridCells().x),
                                mScanVolumeMin.y() + y * ((mScanVolumeMax.y()-mScanVolumeMin.y())/mParticleSystem->gridCells().y),
                                mScanVolumeMax.z(),
                                1.0f));
            }
        }

        for(int x=0;x<=mParticleSystem->gridCells().x;x++)
        {
            for(int z=0;z<=mParticleSystem->gridCells().z;z++)
            {
                lineData.append(QVector4D(
                                mScanVolumeMin.x() + x * ((mScanVolumeMax.x()-mScanVolumeMin.x())/mParticleSystem->gridCells().x),
                                mScanVolumeMin.y(),
                                mScanVolumeMin.z() + z * ((mScanVolumeMax.z()-mScanVolumeMin.z())/mParticleSystem->gridCells().z),
                                1.0f));

                lineData.append(QVector4D(
                                mScanVolumeMin.x() + x * ((mScanVolumeMax.x()-mScanVolumeMin.x())/mParticleSystem->gridCells().x),
                                mScanVolumeMax.y(),
                                mScanVolumeMin.z() + z * ((mScanVolumeMax.z()-mScanVolumeMin.z())/mParticleSystem->gridCells().z),
                                1.0f));
            }
        }


        for(int y=0;y<=mParticleSystem->gridCells().y;y++)
        {
            for(int z=0;z<=mParticleSystem->gridCells().z;z++)
            {
                lineData.append(QVector4D(
                                mScanVolumeMin.x(),
                                mScanVolumeMin.y() + y * ((mScanVolumeMax.y()-mScanVolumeMin.y())/mParticleSystem->gridCells().y),
                                mScanVolumeMin.z() + z * ((mScanVolumeMax.z()-mScanVolumeMin.z())/mParticleSystem->gridCells().z),
                                1.0f));

                lineData.append(QVector4D(
                                mScanVolumeMax.x(),
                                mScanVolumeMin.y() + y * ((mScanVolumeMax.y()-mScanVolumeMin.y())/mParticleSystem->gridCells().y),
                                mScanVolumeMin.z() + z * ((mScanVolumeMax.z()-mScanVolumeMin.z())/mParticleSystem->gridCells().z),
                                1.0f));
            }
        }

        // Create a VBO for the grid lines if it doesn't exist yet
        if(!mVboGridLines) glGenBuffers(1, &mVboGridLines);

        // Bind, fill and unbind the buffer
        glBindBuffer(GL_ARRAY_BUFFER, mVboGridLines);
        glBufferData(GL_ARRAY_BUFFER, lineData.size() * sizeof(QVector4D), lineData.constData(), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
}


void FlightPlannerParticles::slotWayPointReached(const WayPoint wpt)
{
    FlightPlannerInterface::slotWayPointReached(wpt);

    // When only two waypoints are left, we want to clear the ParticleSystem's grid of wayppint pressure.
    // This way, the particlesystem has the duration of the two remaining waypoints to create new waypoints
    if(mWaypointListMap.value("ahead")->size() == 2)
        mParticleSystem->slotClearGridWayPointPressure();

    //    slotCreateSafePathToNextWayPoint();
}

void FlightPlannerParticles::slotVehiclePoseChanged(const Pose* const pose)
{
    FlightPlannerInterface::slotVehiclePoseChanged(pose);

    // check for collisions?!
}
