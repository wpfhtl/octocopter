#include <GL/glew.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include "flightplannerparticles.h"
#include "pointcloudcuda.h"

FlightPlannerParticles::FlightPlannerParticles(QWidget* parentWidget, GlWidget *glWidget, PointCloud *pointcloud) : FlightPlannerInterface(parentWidget, glWidget, pointcloud)
{
    mParticleSystem = 0;
    mParticleRenderer = 0;
    mVboGridLines = 0;

    mUpdateParticleSystem = true;

    // register dense pointcloud for rendering. Might be moved to base class c'tor
    mGlWidget->slotPointCloudRegister(mPointCloudDense);

    mPointCloudDense->mName = "Dense";

    // For every point in this cloud, particlesystem will have to create
    // 2 32bit ints to map grid cell index to a particle index.
    mPointCloudColliders = new PointCloudCuda(
                mPointCloudDense->getBoundingBoxMin(),
                mPointCloudDense->getBoundingBoxMax(),
                128 * 1024);

    mPointCloudColliders->mName = "Colliders";

//    mPointCloudColliders->setGridSize(64, 32, 64);

    mPointCloudColliders->setMinimumPointDistance(0.9f);

    mPointCloudColliders->setColor(QColor(128,128,128,64));

    mGlWidget->slotPointCloudRegister(mPointCloudColliders);
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

    mParticleSystem->slotSetParticleCount(256);
    mParticleSystem->slotSetParticleRadius(0.5f); // balance against mOctreeCollisionObjects.setMinimumPointDistance() above
    mParticleSystem->slotSetDefaultParticlePlacement(ParticleSystem::ParticlePlacement::PlacementFillSky);
//    mParticleSystem->slotSetVolume(mScanVolumeMin, mScanVolumeMax);
}

void FlightPlannerParticles::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_1)
    {
        mUpdateParticleSystem = !mUpdateParticleSystem;
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

void FlightPlannerParticles::slotGenerateWaypoints()
{
    qDebug() << "FlightPlannerParticles::slotGenerateWaypoints()";
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

        if(mUpdateParticleSystem)
            mParticleSystem->update(0.5f);

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

void FlightPlannerParticles::slotProcessPhysics(bool process)
{
}

void FlightPlannerParticles::slotWayPointReached(const WayPoint wpt)
{
    FlightPlannerInterface::slotWayPointReached(wpt);

    //    slotCreateSafePathToNextWayPoint();
}

void FlightPlannerParticles::slotVehiclePoseChanged(const Pose* const pose)
{
    FlightPlannerInterface::slotVehiclePoseChanged(pose);

    // check for collisions?!
}
