#ifndef PARTICLESYSTEM_H
#define PARTICLESYSTEM_H

#include "particleskernel.cuh"
#include "vector_functions.h"

class ParticleSystem
{
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

    float* getArray(ParticleArray array);
    void   setArray(ParticleArray array, const float* data, int start, int count);

    int    getNumParticles() const { return m_numParticles; }

    unsigned int getCurrentReadBuffer() const { return m_posVbo; }
    unsigned int getColorBuffer()       const { return m_colorVBO; }

    void * getCudaPosVBO()              const { return (void *)m_cudaPosVBO; }
    void * getCudaColorVBO()            const { return (void *)m_cudaColorVBO; }

    void dumpGrid();
    void dumpParticles(unsigned int start, unsigned int count);

    void setDamping(float x) { m_params.globalDamping = x; }
    void setGravity(float x) { m_params.gravity = make_float3(0.0f, x, 0.0f); }

    void setCollideSpring(float x) { m_params.spring = x; }
    void setCollideDamping(float x) { m_params.damping = x; }
    void setCollideShear(float x) { m_params.shear = x; }
    void setCollideAttraction(float x) { m_params.attraction = x; }

    void setColliderPos(float3 x) { m_params.colliderPos = x; }

    float getParticleRadius() { return m_params.particleRadius; }
    float3 getColliderPos() { return m_params.colliderPos; }
    float getColliderRadius() { return m_params.colliderRadius; }
    uint3 getGridSize() { return m_params.gridSize; }
    float3 getWorldOrigin() { return m_params.worldOrigin; }
    float3 getCellSize() { return m_params.cellSize; }

    void addSphere(int index, float *pos, float *vel, int r, float spacing);

protected: // methods
    ParticleSystem() {}
    unsigned int createVBO(unsigned int size);

    void colorRamp(float t, float *r);
    void initGrid(unsigned int *size, float spacing, float jitter, unsigned int numParticles);

protected: // data
//    bool m_bInitialized, m_bUseOpenGL;
    unsigned int m_numParticles;

    // CPU data
    float* m_hPos;              // particle positions
    float* m_hVel;              // particle velocities

    unsigned int*  m_hParticleHash;
    unsigned int*  m_hCellStart;
    unsigned int*  m_hCellEnd;

    // GPU data
    float* m_dPos;
    float* m_dVel;

    float* m_dSortedPos;
    float* m_dSortedVel;

    // grid data for sorting method
    unsigned int*  m_dGridParticleHash; // grid hash value for each particle
    unsigned int*  m_dGridParticleIndex;// particle index for each particle
    unsigned int*  m_dCellStart;        // index of start of each cell in sorted list
    unsigned int*  m_dCellEnd;          // index of end of cell

    unsigned int   m_gridSortBits;

    unsigned int   m_posVbo;            // vertex buffer object for particle positions
    unsigned int   m_colorVBO;          // vertex buffer object for colors
    
    float *m_cudaPosVBO;        // these are the CUDA deviceMem Pos
    float *m_cudaColorVBO;      // these are the CUDA deviceMem Color

    struct cudaGraphicsResource *m_cuda_posvbo_resource; // handles OpenGL-CUDA exchange
    struct cudaGraphicsResource *m_cuda_colorvbo_resource; // handles OpenGL-CUDA exchange

    // params
    SimParams m_params;
    uint3 m_gridSize;
    unsigned int m_numGridCells;
};

#endif
