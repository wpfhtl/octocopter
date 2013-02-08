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

    void update(const float deltaTime);

    Vector3i gridCells()
    {
        Vector3i gc;
        gc.x = mSimulationParameters.gridSize.x;
        gc.y = mSimulationParameters.gridSize.y;
        gc.z = mSimulationParameters.gridSize.z;
        return gc;
    }

private slots:
    void slotNewCollidersInserted();

public slots:
    void slotSetDefaultParticlePlacement(const ParticlePlacement pp) { mDefaultParticlePlacement = pp; }

    // The user changed some value sin the UI. Accept all fields from SimulationParameters that make sense
    // (e.g. changing particleCount doesn't make sense, as we'd have to re-initialize). Maybe later.
    void slotSetSimulationParametersFromUi(const SimulationParameters* sp)
    {
        mSimulationParameters.attraction = sp->attraction;
        mSimulationParameters.dampingMotion = sp->dampingMotion;
        mSimulationParameters.gravity = sp->gravity;
        mSimulationParameters.particleRadius = sp->particleRadius;
        mSimulationParameters.shear = sp->shear;
        mSimulationParameters.spring = sp->spring;
        mSimulationParameters.velocityFactorCollisionBoundary = sp->velocityFactorCollisionBoundary;
        mSimulationParameters.velocityFactorCollisionParticle = sp->velocityFactorCollisionParticle;
    }

    void slotSetParticleRadius(float);

    void slotSetParticleSpring(const float spring) { mSimulationParameters.spring = spring; }

    void slotClearGridWayPointPressure();

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

    void slotResetParticles();

    bool getRankedWaypoints(QVector4D* const waypoints, const quint16 numberOfWaypointsRequested);

signals:
    void particleRadiusChanged(float);
    void vboInfoParticles(quint32 vboPositions, quint32 vboColor, quint32 particleCount);
    void vboInfoGridWaypointPressure(quint32 vboPressure, QVector3D gridBoundingBoxMin, QVector3D gridBoundingBoxMax, Vector3i grid);

protected:
    // A pointer to the pointcloud holding the pointcloud to collide particles against.
    PointCloud* mPointCloudColliders;

    quint64 mNumberOfBytesAllocatedCpu;
    quint64 mNumberOfBytesAllocatedGpu;

    void showWaypointPressure();
    void showCollisionPositions();

    Vector3i getGridCellCoordinate(const quint32 hash) const;
    Vector3i getGridCellCoordinate(const QVector3D &worldPos) const;

    QVector3D getGridCellCenter(const Vector3i &gridCellCoordinate) const;
    QVector3D getGridCellSize() const;
    quint32 getGridCellHash(Vector3i gridCellCoordinate) const;

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

    unsigned int createVbo(quint32 size);

    void colorRamp(float t, float *r);

    void setArray(ParticleArray array, const float* data, int start, int count);

    bool mIsInitialized;

    // When the collider pointcloud updates, we need to update supporting data structures on the GPU.
    bool mColliderPointCloudWasUpdated;

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

    // There is a gridmap of waypoint pressure on the GPU. To be useful for processing on the host, we first create a
    // list of QVector4D that corresponds to the cell's positions. Then, we sort the latter list using thrust::sort_by_key
    // (the key being the waypoint pressure) and receive a ranking of QVector4Ds. Hah!
    float*  mDeviceGridMapCellWorldPositions;

    // We copy the waypoint pressure of the grid cells from the mVboGridMapOfWayPointPressure VBO to this pointer, then use
    //
    // thrust::sort_by_key(
    //                      mDeviceGridMapWayPointPressureSorted.begin(),
    //                      mDeviceGridMapWayPointPressureSorted.end(),
    //                      mDeviceGridMapCellWorldPositions.begin(),
    //                      operator>()
    // );
    quint8* mDeviceGridMapWayPointPressureSorted;

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

    // A gridmap (same grid as always) containing values from 0 to 255. 0 means no waypoint candidates within, 255 means maximum waypoint pressure.
    unsigned int   mVboGridMapOfWayPointPressure;
    unsigned int   mVboParticlePositions;   // vertex buffer object for particle positions
    unsigned int   mVboColliderPositions;   // vertex buffer object for collider positions
    unsigned int   mVboParticleColors;      // vertex buffer object for particle colors

    struct cudaGraphicsResource *mCudaVboResourceParticlePositions; // handles OpenGL-CUDA exchange
    struct cudaGraphicsResource *mCudaVboResourceColliderPositions; // handles OpenGL-CUDA exchange

    // Waypoint-pressure-values are stored in a VBO as 8bit-unsigned-ints
    struct cudaGraphicsResource *mCudaVboResourceGridMapOfWayPointPressure; // handles OpenGL-CUDA exchange
    struct cudaGraphicsResource *mCudaColorVboResource; // handles OpenGL-CUDA exchange

    SimulationParameters mSimulationParameters;
};

#endif
