#ifndef PARTICLESYSTEM_H
#define PARTICLESYSTEM_H

#include "particleskernel.cuh"
#include "vector_functions.h"
#include <QObject>
#include <QVector3D>

// See http://forums.nvidia.com/index.php?showtopic=173696

class ParticleSystem : public QObject
{
    Q_OBJECT
public:
    ParticleSystem(unsigned int numParticles, uint3 gridSize);
    ~ParticleSystem();

    enum ParticleConfig
    {
        CONFIG_RANDOM,
        CONFIG_GRID,
        _NUM_CONFIGS
    };

    enum ParticleArray
    {
        POSITION,
        VELOCITY
    };

    void update(float deltaTime);
    void reset(ParticleConfig config);

//    float* getArray(ParticleArray array);
    void   setArray(ParticleArray array, const float* data, int start, int count);

    int    getNumParticles() const { return mNumberOfParticles; }

    unsigned int getCurrentReadBuffer() const { return mPositionVboHandle; }
    unsigned int getColorBuffer()       const { return mColorVboHandle; }

//    void * getCudaPosVBO()              const { return (void *)m_cudaPosVBO; }
//    void * getCudaColorVBO()            const { return (void *)mCudaColorVbo; }

//    void dumpGrid();
//    void dumpParticles(unsigned int start, unsigned int count);

    void setDamping(float x) { mSimulationParameters.globalDamping = x; }
    void setGravity(float x) { mSimulationParameters.gravity = make_float3(0.0f, x, 0.0f); }

    void setCollideSpring(float x) { mSimulationParameters.spring = x; }
    void setCollideDamping(float x) { mSimulationParameters.damping = x; }
    void setCollideShear(float x) { mSimulationParameters.shear = x; }
    void setCollideAttraction(float x) { mSimulationParameters.attraction = x; }

//    void setColliderPos(float3 x) { mSimulationParameters.colliderPos = x; }

    void setVolume(const QVector3D& min, const QVector3D& max);

    float getParticleRadius() { return mSimulationParameters.particleRadius; }

//    float3 getColliderPos() { return mSimulationParameters.colliderPos; }
//    float getColliderRadius() { return mSimulationParameters.colliderRadius; }
//    uint3 getGridSize() { return mSimulationParameters.gridSize; }
    //    float3 getWorldOrigin() { return mSimulationParameters.worldOrigin; }
//    float3 getCellSize() { return mSimulationParameters.cellSize; }

//    void addSphere(int index, float *pos, float *vel, int r, float spacing);

public slots:
    void slotSetParticleRadius(float);

signals:
    void particleRadiusChanged(float);

protected: // methods
    ParticleSystem() {}
    unsigned int createVBO(unsigned int size);

    void colorRamp(float t, float *r);
    void initGrid(unsigned int *size, float spacing, float jitter, unsigned int numParticles);

protected: // data
    unsigned int mNumberOfParticles;

    // CPU data
    float* mHostPos;              // particle positions
    float* mHostVel;              // particle velocities

//    unsigned int*  m_hParticleHash;
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

    unsigned int   mPositionVboHandle;      // vertex buffer object for particle positions
    unsigned int   mColorVboHandle;         // vertex buffer object for particle colors
    
//    float *m_cudaPosVBO;        // these are the CUDA deviceMem Pos
//    float *mCudaColorVbo;      // these are the CUDA deviceMem Color

    struct cudaGraphicsResource *mCudaPositionVboResource; // handles OpenGL-CUDA exchange
    struct cudaGraphicsResource *mCudaColorVboResource; // handles OpenGL-CUDA exchange

    // params
    SimParams mSimulationParameters;
    uint3 mGridSize;
    unsigned int mNumberOfGridCells;
};

#endif
