#ifndef PARTICLESYSTEM_H
#define PARTICLESYSTEM_H

#include "particleskernel.cuh"
#include "vector_functions.h"
#include <QDebug>
#include <QVector3D>

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

//    unsigned int getVboPositions() const { return mVboPositions; }
//    unsigned int getVboColors() const { return mVboColors; }

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
        freeResources();
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


    enum ParticleArray
    {
        ArrayPositions,
        ArrayVelocities
    };


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

    unsigned int*  mHostCellStart;
    unsigned int*  mHostCellEnd;

    // GPU data
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
