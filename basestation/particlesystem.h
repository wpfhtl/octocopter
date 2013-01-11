#ifndef PARTICLESYSTEM_H
#define PARTICLESYSTEM_H

#include "particleskernel.cuh"

#include "vector_functions.h"
#include <QTime>
#include <QDebug>
#include <QVector3D>
#include <QVector4D>

#include "common.h"

class PointCloud;

// See http://forums.nvidia.com/index.php?showtopic=173696

class ParticleSystem : public QObject
{
    Q_OBJECT
public:
    // Give it a pointer to the pointcloud to collide against
    ParticleSystem(PointCloud *const pointcloud);
    ~ParticleSystem();

    enum class ParticlePlacement
    {
        PlacementRandom,
        PlacementGrid,
        PlacementFillSky
    };

    struct GridCells
    {
        quint16 x,y,z;
    };

    void update(const float deltaTime);

    GridCells gridCells()
    {
        GridCells gc;
        gc.x = mSimulationParameters.gridSize.x;
        gc.y = mSimulationParameters.gridSize.y;
        gc.z = mSimulationParameters.gridSize.z;
        return gc;
    }

public slots:
    void slotSetDefaultParticlePlacement(const ParticlePlacement pp) { mDefaultParticlePlacement = pp; }

    void slotSetParticleRadius(float);

    void slotSetParticleSpring(const float spring) { mSimulationParameters.spring = spring; }

    void slotSetParticleCount(const quint32 count)
    {
        mSimulationParameters.particleCount = count;
        // Need to rebuild data-structures when particle count changes.
        if(mIsInitialized) freeResources();
    }

    // Values between 0 and 1 make sense, something like 0.98f seems realistic
    void slotSetDampingMotion(const float damping) { mSimulationParameters.dampingMotion = damping; }

    // Needs to be negative, so that the particle reverses direction when hitting a bounding wall
    void slotSetVelocityFactorCollisionBoundary(const float factor) { mSimulationParameters.velocityFactorCollisionBoundary = factor; }

    // The more damping, the more a collision will speed up the particles. So its the inverse of what you'd expect.
    void slotSetVelocityFactorCollisionParticle(const float factor) { mSimulationParameters.velocityFactorCollisionParticle = factor; }

    void slotSetGravity(const QVector3D& gravity)
    {
        mSimulationParameters.gravity.x = gravity.x();
        mSimulationParameters.gravity.y = gravity.y();
        mSimulationParameters.gravity.z = gravity.z();
    }

    void slotSetVolume(const QVector3D& min, const QVector3D& max);

signals:
    void particleRadiusChanged(float);
    void vboInfoParticles(quint32 vboPositions, quint32 vboColor, quint32 particleCount);
    void vboInfoColliders(quint32 vboPositions, quint32 colliderCount);

protected:
    // A pointer to the pointcloud holding the pointcloud to collide particles against.
    PointCloud* mPointCloudColliders;

    quint64 mNumberOfBytesAllocatedCpu;
    quint64 mNumberOfBytesAllocatedGpu;

    enum ParticleArray
    {
        ArrayPositions,
        ArrayVelocities
    };

    QVector3D getWorldSize()
    {
            return QVector3D(
                        mSimulationParameters.worldMax.x - mSimulationParameters.worldMin.x,
                        mSimulationParameters.worldMax.y - mSimulationParameters.worldMin.y,
                        mSimulationParameters.worldMax.z - mSimulationParameters.worldMin.z);
    }

    void setNullPointers();

    void initialize();
    void freeResources();

    void placeParticles();

    unsigned int createVbo(quint32 size);

    void colorRamp(float t, float *r);

    void setArray(ParticleArray array, const float* data, int start, int count);

    bool mIsInitialized;
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
    unsigned int   mVboParticleColors;      // vertex buffer object for particle colors

    struct cudaGraphicsResource *mCudaVboResourceParticlePositions; // handles OpenGL-CUDA exchange
    struct cudaGraphicsResource *mCudaVboResourceColliderPositions; // handles OpenGL-CUDA exchange
    struct cudaGraphicsResource *mCudaColorVboResource; // handles OpenGL-CUDA exchange

    CollisionParameters mSimulationParameters;
};

#endif
