#ifndef PARTICLESYSTEM_H
#define PARTICLESYSTEM_H

#include "particleskernel.cuh"
#include "vector_functions.h"
#include <QTime>
#include <QDebug>
#include <QVector3D>
#include <QVector4D>

// See http://forums.nvidia.com/index.php?showtopic=173696

class ParticleSystem : public QObject
{
    Q_OBJECT
public:
    ParticleSystem();
    ~ParticleSystem();

    enum ParticlePlacement
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

    void insertPoint(const QVector3D& point)
    {
        if(!mIsInitialized) return;

        float point4[4];
        point4[0] = point.x();
        point4[1] = point.y();
        point4[2] = point.z();
        point4[3] = 0.0;

        setArray(ArrayPositions, point4, mNumberOfFixedParticles, 1);
        mNumberOfFixedParticles++;
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
    void vboPositionChanged(unsigned int vboPositions, unsigned int particleCount);
    void vboColorChanged(unsigned int vboColor);

protected:

    // At the beginning, the particle buffers contain only sampling particles. They have a w-component of 1.0,
    // move freely and collide with all other particles in the buffer. When we receive a new point for the
    // collision-pointcloud, we write its position into devicePositionArray[mNumberOfFixedParticles] and set
    // its w-component to 0.0, so that integration skips moving this particle. Collisions will occur as always
    quint32 mNumberOfFixedParticles;

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

    // compute the next higher power of 2 of 32-bit v
    static quint32 nextHigherPowerOfTwo(quint32 v)
    {
        // decrements, then sets all bits below its most significant bit to 1, then it increments
        v--;
        v |= v >> 1;
        v |= v >> 2;
        v |= v >> 4;
        v |= v >> 8;
        v |= v >> 16;
        return v + 1;
    }

    void setNullPointers();

    void initialize();
    void freeResources();

    void placeParticles();

    unsigned int createVbo(unsigned int size);

    void colorRamp(float t, float *r);

    void setArray(ParticleArray array, const float* data, int start, int count);

    bool mIsInitialized;
    ParticlePlacement mDefaultParticlePlacement;

    // CPU data
    float* mHostPos;              // particle positions
    float* mHostVel;              // particle velocities

//    unsigned int*  mHostCellStart;
//    unsigned int*  mHostCellEnd;

    // Pointers to position and velocity data of particles on the GPU. Both are stored as float[4], so we have x,y,z,w.
    // During the collision phase, the device-code searches for all particles in the current and neighboring grid cells,
    // so ordering particle data according to grid-cell-number shows a less scattered memory access pattern and makes
    // sure that neighboring threads fetch similar data, yielding a big speedup.
    //
    // After particles have been moved according to their speed (in integrateSystem()), their position and velocity
    // arrays aren't sorted according to grid-cells anymore, because some have moved to different grid cells.
    float* mDevicePos;
    float* mDeviceVel;
    float* mDeviceSortedPos;
    float* mDeviceSortedVel;

    // grid data for sorting method
    unsigned int*  mDeviceGridParticleHash; // grid hash value for each particle
    unsigned int*  mDeviceGridParticleIndex;// particle index for each particle
    unsigned int*  mDeviceCellStart;        // index of start of each cell in sorted list
    unsigned int*  mDeviceCellEnd;          // index of end of cell

    unsigned int   mVboPositions;      // vertex buffer object for particle positions
    unsigned int   mVboColors;         // vertex buffer object for particle colors

    struct cudaGraphicsResource *mCudaPositionVboResource; // handles OpenGL-CUDA exchange
    struct cudaGraphicsResource *mCudaColorVboResource; // handles OpenGL-CUDA exchange

    SimParams mSimulationParameters;
};

#endif
