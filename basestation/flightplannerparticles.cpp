#include <GL/glew.h>
//#include <GL/freeglut.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include "flightplannerparticles.h"

FlightPlannerParticles::FlightPlannerParticles(QWidget* widget) : FlightPlannerInterface(widget)
{
    mParticleSystem = 0;
    mParticleRenderer = 0;
    mVboGridLines = 0;
    mOctreeCollisionObjects = 0;
}

void FlightPlannerParticles::setupCollisionOctree()
{
    if(mOctreeCollisionObjects)
    {
        disconnect(this, SLOT(slotPointAcceptedIntoOctree(const LidarPoint*)));
        delete mOctreeCollisionObjects;
    }

    // This octree lives in host memory and holds points that are used for collisions with the CUDA particles.
    // The particles are copied into GPU memory, making the octree seemingly redundant - but we use it to discard
    // incoming points that have close neighbors already.
    mOctreeCollisionObjects = new Octree(QVector3D(-10, -10, -10), QVector3D(10, 10, 10), 100, 50*1000);
    mOctreeCollisionObjects->setMinimumPointDistance(0.85f);
    connect(mOctreeCollisionObjects, SIGNAL(pointInserted(const LidarPoint*)), SLOT(slotPointAcceptedIntoOctree(const LidarPoint*)));
}

void FlightPlannerParticles::slotInitialize()
{
    qDebug() << "FlightPlannerParticles::slotInitialize()";

    mShaderProgramGridLines = new ShaderProgram(this, "shader-default-vertex.c", "", "shader-default-fragment.c");

    mParticleSystem = new ParticleSystem(mOctree); // ParticleSystem will draw its points as colliders from  the dense mOctree
    mParticleRenderer = new ParticleRenderer;

    connect(mParticleSystem, SIGNAL(particleRadiusChanged(float)), mParticleRenderer, SLOT(slotSetParticleRadius(float)));
    connect(mParticleSystem, SIGNAL(vboInfoParticles(quint32,quint32,quint32)), mParticleRenderer, SLOT(slotSetVboInfoParticles(quint32,quint32,quint32)));
    connect(mParticleSystem, SIGNAL(vboInfoColliders(quint32,quint32)), mParticleRenderer, SLOT(slotSetVboInfoColliders(quint32,quint32)));

    mParticleSystem->slotSetParticleCount(32768);
    mParticleSystem->slotSetParticleRadius(0.60f); // balance against mOctreeCollisionObjects.setMinimumPointDistance() above
    mParticleSystem->slotSetDefaultParticlePlacement(ParticleSystem::PlacementFillSky);
//    mParticleSystem->slotSetVolume(mScanVolumeMin, mScanVolumeMax);
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
    delete mOctreeCollisionObjects;
    cudaDeviceReset();
}

void FlightPlannerParticles::insertPoint(const LidarPoint *const point)
{
    if(mOctreeCollisionObjects)
        mOctreeCollisionObjects->insertPoint(new LidarPoint(*point));
}

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
            //        glDrawArrays(GL_LINES, 0, (mParticleSystem->gridCells().x+1) * (mParticleSystem->gridCells().y+1) * (mParticleSystem->gridCells().z+1));
            glDisable(GL_BLEND);
            glDisableVertexAttribArray(0);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            mShaderProgramGridLines->release();
        }

        mParticleSystem->update(0.2f);
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

        // Re-fill mOctreeCollisionObjects data from basestations octree!
        if(mOctree)
        {
            setupCollisionOctree();
            FlightPlannerInterface::insertPointsFromNode(mOctree->root());
        }
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

void FlightPlannerParticles::slotPointAcceptedIntoOctree(const LidarPoint* point)
{
    mParticleSystem->insertPoint(point->position);
}
