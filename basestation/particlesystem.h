#ifndef PARTICLESYSTEM_H
#define PARTICLESYSTEM_H

#include "parametersparticlesystem.cuh"

#include <QTime>
#include <QDebug>
#include <QVector3D>
#include <QVector4D>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLFunctions_4_3_Core>

#include "common.h"
#include "grid.cuh"

class PointCloudCuda;

// See http://forums.nvidia.com/index.php?showtopic=173696

class ParticleSystem : public QObject, protected OPENGL_FUNCTIONS_CLASS
{
    Q_OBJECT
public:
    // Give it a pointer to the pointcloud to collide against
    ParticleSystem(PointCloudCuda *const pointCloudDense, PointCloudCuda *const pointCloudColliders, ParametersParticleSystem* const simulationParameters);
    ~ParticleSystem();

    enum class ParticlePlacement
    {
        PlacementRandom,
        PlacementGrid,
        PlacementFillSky
    };

    void update(quint8 *deviceGridMapOfWayPointPressure);

private slots:
    void slotNewCollidersInserted();

public slots:
    void slotSetDefaultParticlePlacement(const ParticlePlacement pp) { mDefaultParticlePlacement = pp; }

    // The user changed some value sin the UI. Accept all fields from SimulationParameters that make sense
    // (e.g. changing particleCount doesn't make sense, as we'd have to re-initialize). Maybe later.
    void slotSetSimulationParametersFromUi(const ParametersParticleSystem* sp)
    {
        mParametersSimulation->timeStepInner = sp->timeStepInner;
        mParametersSimulation->attraction = sp->attraction;
        mParametersSimulation->dampingMotion = sp->dampingMotion;
        mParametersSimulation->gravity = sp->gravity;
        mParametersSimulation->particleRadius = sp->particleRadius;
        mParametersSimulation->shear = sp->shear;
        mParametersSimulation->spring = sp->spring;
        mParametersSimulation->velocityFactorCollisionBoundary = sp->velocityFactorCollisionBoundary;
        mParametersSimulation->velocityFactorCollisionParticle = sp->velocityFactorCollisionParticle;
        mParametersSimulation->velocityFactorCollisionCollider = sp->velocityFactorCollisionCollider;
    }

    // Must be called:
    // - before any work is done
    // Requires:
    // - active OpenGL context (because of cudaGraphicsGLRegisterBuffer)
    // - mParametersSimulation->particleCount to be defined
    // - mPointCloudColliders must be set!
    void slotInitialize();

    void slotSetParticleRadius(float);

    void slotSetParticleCount(const quint32 count)
    {
        mParametersSimulation->particleCount = count;
        // Need to rebuild data-structures when particle count changes.
        if(mIsInitialized) freeResources();
    }

    void slotSetVolume(const Box3D &boundingBox);

    void slotResetParticles();

signals:
    void vboInfoParticles(quint32 vboPositions, quint32 particleCount, float particleRadius, Box3D particleSystemBoundingBox);

protected:
    ParametersParticleSystem* mParametersSimulation;
    // A pointer to the pointclouds holding the dense points and the colliders.
    PointCloudCuda *mPointCloudDense, *mPointCloudColliders;

    quint64 mNumberOfBytesAllocatedCpu;
    quint64 mNumberOfBytesAllocatedGpu;

    void showCollisionPositions();

    enum ParticleArray
    {
        ArrayPositions,
        ArrayVelocities
    };

    void emitVboInfo();

    void setNullPointers();

    void freeResources();

    unsigned int createVbo(quint32 size);

    void colorRamp(float t, float *r);

    void setArray(ParticleArray array, const float* data, int start, int count);

    bool mIsInitialized;

    // When the collider pointcloud updates, we need to update supporting data structures on the GPU.
    bool mUpdateMappingFromColliderToGridCell;

    ParticlePlacement mDefaultParticlePlacement;

    // CPU data
    float* mHostParticlePos;              // particle positions
    float* mHostParticleVel;              // particle velocities

    // Pointers to position and velocity data of particles on the GPU. Both are stored as float[4], so we have x,y,z,w.
    // During the collision phase, the device-code searches for all particles in the current and neighboring grid cells,
    // so ordering particle data according to grid-cell-number shows a less scattered memory access pattern and makes
    // sure that neighboring threads fetch similar data, yielding a big speedup.
    //
    // After particles have been moved according to their speed (in integrateSystem()), their position and velocity
    // arrays aren't sorted according to grid-cells anymore, because some have moved to different grid cells.
    // mDeviceParticlePos isn't needed, the data is contained in an OpenGL VBO.
    float* mDeviceParticleVel;
    float* mDeviceParticleSortedPos;
    float* mDeviceParticleSortedVel;

    float* mDeviceColliderSortedPos;

    float* mDeviceParticleCollisionPositions;

    // A map, mapping from gridcell => particle index. Length is particlecount
    unsigned int*  mDeviceParticleMapGridCell;  // grid hash value for each particle
    unsigned int*  mDeviceParticleMapIndex;     // particle index for each particle

    // A map, mapping from gridcell => collider index. Length is collidercount
    unsigned int*  mDeviceColliderMapGridCell;  // grid hash value for each collider
    unsigned int*  mDeviceColliderMapIndex;     // particle index for each collider

    unsigned int*  mDeviceParticleCellStart;    // index of start of each cell in sorted list
    unsigned int*  mDeviceParticleCellEnd;      // index of end of cell

    unsigned int*  mDeviceColliderCellStart;    // index of start of each cell in sorted list
    unsigned int*  mDeviceColliderCellEnd;      // index of end of cell

    unsigned int   mVboParticlePositions;   // vertex buffer object for particle positions
    unsigned int   mVboColliderPositions;   // vertex buffer object for collider positions

    struct cudaGraphicsResource *mCudaVboResourceParticlePositions; // handles OpenGL-CUDA exchange
    struct cudaGraphicsResource *mCudaVboResourceColliderPositions; // handles OpenGL-CUDA exchange

    struct cudaGraphicsResource *mCudaColorVboResource; // handles OpenGL-CUDA exchange
};

#endif
