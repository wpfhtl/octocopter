#include "particlesystem.h"
#include "particlesystem_cuda.cuh"
#include "particleskernel.cuh"

#include <cuda_runtime_api.h>

#include <assert.h>
#include <math.h>
#include <memory.h>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <GL/glew.h>

ParticleSystem::ParticleSystem(unsigned int numParticles, uint3 gridSize) :
    mNumberOfParticles(numParticles),
    mHostPos(0),
    mHostVel(0),
    mDevicePos(0),
    mDeviceVel(0),
    mGridSize(gridSize)
{
    mUseOpenGl = true;
    m_numGridCells = mGridSize.x*mGridSize.y*mGridSize.z;

    // set simulation parameters
    mSimulationParameters.gridSize = mGridSize;
    mSimulationParameters.numCells = m_numGridCells;
    mSimulationParameters.numBodies = mNumberOfParticles;
    mSimulationParameters.particleRadius = 1.0f / 32.0f;
    mSimulationParameters.colliderPos = make_float3(-1.2f, -0.8f, 0.8f);
    mSimulationParameters.colliderRadius = 3.0f;

    //    m_params.cellSize = make_float3(worldSize.x / m_gridSize.x, worldSize.y / m_gridSize.y, worldSize.z / m_gridSize.z);
    float cellSize = mSimulationParameters.particleRadius * 2.0f;  // cell size equal to particle diameter
    mSimulationParameters.cellSize = make_float3(cellSize, cellSize, cellSize);
    mSimulationParameters.spring = 0.5f;
    mSimulationParameters.damping = 0.02f;
    mSimulationParameters.shear = 0.1f;
    mSimulationParameters.attraction = 0.0f;
    mSimulationParameters.boundaryDamping = -0.5f;
    mSimulationParameters.gravity = make_float3(0.0f, -0.0003f, 0.0f);
    mSimulationParameters.globalDamping = 1.0f;

    mNumberOfParticles = numParticles;

    // allocate host storage
    mHostPos = new float[mNumberOfParticles*4];
    mHostVel = new float[mNumberOfParticles*4];
    memset(mHostPos, 0, mNumberOfParticles*4*sizeof(float));
    memset(mHostVel, 0, mNumberOfParticles*4*sizeof(float));

    mHostCellStart = new uint[m_numGridCells];
    memset(mHostCellStart, 0, m_numGridCells*sizeof(uint));

    mHostCellEnd = new uint[m_numGridCells];
    memset(mHostCellEnd, 0, m_numGridCells*sizeof(uint));

    // allocate GPU data
    unsigned int memSize = sizeof(float) * 4 * mNumberOfParticles;

    if (mUseOpenGl)
    {
        mPositionVboHandle = createVBO(memSize);
        registerGLBufferObject(mPositionVboHandle, &mCudaPosVboResource);
    }
    else
    {
        cudaMalloc( (void **)&m_cudaPosVBO, memSize);
    }

    allocateArray((void**)&mDeviceVel, memSize);
    allocateArray((void**)&mDeviceSortedPos, memSize);
    allocateArray((void**)&mDeviceSortedVel, memSize);
    allocateArray((void**)&mDeviceGridParticleHash, mNumberOfParticles*sizeof(uint));
    allocateArray((void**)&mDeviceGridParticleIndex, mNumberOfParticles*sizeof(uint));
    allocateArray((void**)&mDeviceCellStart, m_numGridCells*sizeof(uint));
    allocateArray((void**)&mDeviceCellEnd, m_numGridCells*sizeof(uint));

    if(mUseOpenGl)
    {
        mColorVbo = createVBO(mNumberOfParticles*4*sizeof(float));
        registerGLBufferObject(mColorVbo, &mCudaColorVboResource);

        // fill color buffer
        glBindBufferARB(GL_ARRAY_BUFFER, mColorVbo);
        float *data = (float *) glMapBufferARB(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
        float *ptr = data;
        for(unsigned int i=0; i<mNumberOfParticles; i++)
        {
            float t = i / (float) mNumberOfParticles;
#if 0
            *ptr++ = rand() / (float) RAND_MAX;
            *ptr++ = rand() / (float) RAND_MAX;
            *ptr++ = rand() / (float) RAND_MAX;
#else
            colorRamp(t, ptr);
            ptr+=3;
#endif
            *ptr++ = 1.0f;
        }
        glUnmapBufferARB(GL_ARRAY_BUFFER);
    }
    else
    {
        cudaMalloc((void **)&mCudaColorVbo, sizeof(float) * numParticles*4);
    }

    setParameters(&mSimulationParameters);
}

void ParticleSystem::setVolume(const QVector3D& min, const QVector3D& max)
{
    mSimulationParameters.worldMin = make_float3(min.x(), min.y(), min.z());
    mSimulationParameters.worldMax = make_float3(max.x(), max.y(), max.z());
}

ParticleSystem::~ParticleSystem()
{
    delete [] mHostPos;
    delete [] mHostVel;
    delete [] mHostCellStart;
    delete [] mHostCellEnd;

    freeArray(mDeviceVel);
    freeArray(mDeviceSortedPos);
    freeArray(mDeviceSortedVel);

    freeArray(mDeviceGridParticleHash);
    freeArray(mDeviceGridParticleIndex);
    freeArray(mDeviceCellStart);
    freeArray(mDeviceCellEnd);

    if (mUseOpenGl) {
        unregisterGLBufferObject(mCudaPosVboResource);
        glDeleteBuffers(1, (const GLuint*)&mPositionVboHandle);
        glDeleteBuffers(1, (const GLuint*)&mColorVbo);
    } else {
        cudaFree(m_cudaPosVBO);
        cudaFree(mCudaColorVbo);
    }
    mNumberOfParticles = 0;
}

unsigned int ParticleSystem::createVBO(unsigned int size)
{
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    return vbo;
}

inline float lerp(float a, float b, float t)
{
    return a + t*(b-a);
}

// create a color ramp
void ParticleSystem::colorRamp(float t, float *r)
{
    const int ncolors = 7;
    float c[ncolors][3] = {
        { 1.0, 0.0, 0.0, },
        { 1.0, 0.5, 0.0, },
        { 1.0, 1.0, 0.0, },
        { 0.0, 1.0, 0.0, },
        { 0.0, 1.0, 1.0, },
        { 0.0, 0.0, 1.0, },
        { 1.0, 0.0, 1.0, },
    };
    t = t * (ncolors-1);
    int i = (int) t;
    float u = t - floor(t);
    r[0] = lerp(c[i][0], c[i+1][0], u);
    r[1] = lerp(c[i][1], c[i+1][1], u);
    r[2] = lerp(c[i][2], c[i+1][2], u);
}

// step the simulation
void ParticleSystem::update(float deltaTime)
{
    float *dPos;

    if(mUseOpenGl)
        dPos = (float *) mapGLBufferObject(&mCudaPosVboResource);
    else
        dPos = (float *) m_cudaPosVBO;

    // update constants
    setParameters(&mSimulationParameters);

    // integrate
    integrateSystem(
                dPos,
                mDeviceVel,
                deltaTime,
                mNumberOfParticles);

    // calculate grid hash
    calcHash(
                mDeviceGridParticleHash,
                mDeviceGridParticleIndex,
                dPos,
                mNumberOfParticles);

    // sort particles based on hash
    sortParticles(mDeviceGridParticleHash, mDeviceGridParticleIndex, mNumberOfParticles);

    // reorder particle arrays into sorted order and find start and end of each cell
    reorderDataAndFindCellStart(
                mDeviceCellStart,
                mDeviceCellEnd,
                mDeviceSortedPos,
                mDeviceSortedVel,
                mDeviceGridParticleHash,
                mDeviceGridParticleIndex,
		dPos,
                mDeviceVel,
                mNumberOfParticles,
		m_numGridCells);

    // process collisions
    collide(
                mDeviceVel,
                mDeviceSortedPos,
                mDeviceSortedVel,
                mDeviceGridParticleIndex,
                mDeviceCellStart,
                mDeviceCellEnd,
                mNumberOfParticles,
                m_numGridCells);

    // note: do unmap at end here to avoid unnecessary graphics/CUDA context switch
    if (mUseOpenGl) {
        unmapGLBufferObject(mCudaPosVboResource);
    }
}

/*
void ParticleSystem::dumpGrid()
{
    // dump grid information
    copyArrayFromDevice(m_hCellStart, m_dCellStart, 0, sizeof(uint)*m_numGridCells);
    copyArrayFromDevice(m_hCellEnd, m_dCellEnd, 0, sizeof(uint)*m_numGridCells);
    unsigned int maxCellSize = 0;
    for(unsigned int i=0; i<m_numGridCells; i++) {
        if (m_hCellStart[i] != 0xffffffff) {
            unsigned int cellSize = m_hCellEnd[i] - m_hCellStart[i];
            //            printf("cell: %d, %d particles\n", i, cellSize);
            if (cellSize > maxCellSize) maxCellSize = cellSize;
        }
    }
    printf("maximum particles per cell = %d\n", maxCellSize);
}

void ParticleSystem::dumpParticles(unsigned int start, unsigned int count)
{
    // debug
    copyArrayFromDevice(m_hPos, 0, &m_cuda_posvbo_resource, sizeof(float)*4*count);
    copyArrayFromDevice(m_hVel, m_dVel, 0, sizeof(float)*4*count);

    for(unsigned int i=start; i<start+count; i++) {
        //        printf("%d: ", i);
        printf("pos: (%.4f, %.4f, %.4f, %.4f)\n", m_hPos[i*4+0], m_hPos[i*4+1], m_hPos[i*4+2], m_hPos[i*4+3]);
        printf("vel: (%.4f, %.4f, %.4f, %.4f)\n", m_hVel[i*4+0], m_hVel[i*4+1], m_hVel[i*4+2], m_hVel[i*4+3]);
    }
}*/

float* ParticleSystem::getArray(ParticleArray array)
{
    float* hdata = 0;
    float* ddata = 0;
    struct cudaGraphicsResource *cudaVboResource = 0;

    if(array == POSITION)
    {
        hdata = mHostPos;
        ddata = mDevicePos;
        cudaVboResource = mCudaPosVboResource;
    }
    else if(array == VELOCITY)
    {
        hdata = mHostVel;
        ddata = mDeviceVel;
    }

    copyArrayFromDevice(hdata, ddata, &cudaVboResource, mNumberOfParticles*4*sizeof(float));
    return hdata;
}

// Sets positions or velocities of the particles.
void ParticleSystem::setArray(ParticleArray array, const float* data, int start, int count)
{
    if(array == POSITION)
    {
        if (mUseOpenGl)
        {
            unregisterGLBufferObject(mCudaPosVboResource);
            glBindBuffer(GL_ARRAY_BUFFER, mPositionVboHandle);
            glBufferSubData(GL_ARRAY_BUFFER, start*4*sizeof(float), count*4*sizeof(float), data);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            registerGLBufferObject(mPositionVboHandle, &mCudaPosVboResource);
        }
    }
    else if(array == VELOCITY)
    {
        copyArrayToDevice(mDeviceVel, data, start*4*sizeof(float), count*4*sizeof(float));
    }       
}

inline float frand()
{
    return rand() / (float) RAND_MAX;
}

void
ParticleSystem::initGrid(unsigned int *size, float spacing, float jitter, unsigned int numParticles)
{
    srand(1973);
    for(unsigned int z=0; z<size[2]; z++) {
        for(unsigned int y=0; y<size[1]; y++) {
            for(unsigned int x=0; x<size[0]; x++) {
                unsigned int i = (z*size[1]*size[0]) + (y*size[0]) + x;
                if (i < numParticles) {
                    mHostPos[i*4] = (spacing * x) + mSimulationParameters.particleRadius - 1.0f + (frand()*2.0f-1.0f)*jitter;
                    mHostPos[i*4+1] = (spacing * y) + mSimulationParameters.particleRadius - 1.0f + (frand()*2.0f-1.0f)*jitter;
                    mHostPos[i*4+2] = (spacing * z) + mSimulationParameters.particleRadius - 1.0f + (frand()*2.0f-1.0f)*jitter;
                    mHostPos[i*4+3] = 1.0f;

                    mHostVel[i*4] = 0.0f;
                    mHostVel[i*4+1] = 0.0f;
                    mHostVel[i*4+2] = 0.0f;
                    mHostVel[i*4+3] = 0.0f;
                }
            }
        }
    }
}

void
ParticleSystem::reset(ParticleConfig config)
{
    switch(config)
    {
    default:
    case CONFIG_RANDOM:
    {
        int p = 0, v = 0;
        for(unsigned int i=0; i < mNumberOfParticles; i++)
        {
            float point[3];
            point[0] = frand();
            point[1] = frand();
            point[2] = frand();
            mHostPos[p++] = 2 * (point[0] - 0.5f);
            mHostPos[p++] = 2 * (point[1] - 0.5f);
            mHostPos[p++] = 2 * (point[2] - 0.5f);
            mHostPos[p++] = 1.0f; // radius
            mHostVel[v++] = 0.0f;
            mHostVel[v++] = 0.0f;
            mHostVel[v++] = 0.0f;
            mHostVel[v++] = 0.0f;
        }
    }
    break;

    case CONFIG_GRID:
    {
        float jitter = mSimulationParameters.particleRadius*0.01f;
        unsigned int s = (int) ceilf(powf((float) mNumberOfParticles, 1.0f / 3.0f));
        unsigned int gridSize[3];
        gridSize[0] = gridSize[1] = gridSize[2] = s;
        initGrid(gridSize, mSimulationParameters.particleRadius*2.0f, jitter, mNumberOfParticles);
    }
    break;
    }

    setArray(POSITION, mHostPos, 0, mNumberOfParticles);
    setArray(VELOCITY, mHostVel, 0, mNumberOfParticles);
}

// Adds a sphere of particles into the scene...
void ParticleSystem::addSphere(int start, float *pos, float *vel, int r, float spacing)
{
    unsigned int index = start;
    for(int z=-r; z<=r; z++)
    {
        for(int y=-r; y<=r; y++)
        {
            for(int x=-r; x<=r; x++)
            {
                float dx = x*spacing;
                float dy = y*spacing;
                float dz = z*spacing;
                float l = sqrtf(dx*dx + dy*dy + dz*dz);
                float jitter = mSimulationParameters.particleRadius*0.01f;
                if ((l <= mSimulationParameters.particleRadius*2.0f*r) && (index < mNumberOfParticles))
                {
                    mHostPos[index*4]   = pos[0] + dx + (frand()*2.0f-1.0f)*jitter;
                    mHostPos[index*4+1] = pos[1] + dy + (frand()*2.0f-1.0f)*jitter;
                    mHostPos[index*4+2] = pos[2] + dz + (frand()*2.0f-1.0f)*jitter;
                    mHostPos[index*4+3] = pos[3];

                    mHostVel[index*4]   = vel[0];
                    mHostVel[index*4+1] = vel[1];
                    mHostVel[index*4+2] = vel[2];
                    mHostVel[index*4+3] = vel[3];
                    index++;
                }
            }
        }
    }

    setArray(POSITION, mHostPos, start, index);
    setArray(VELOCITY, mHostVel, start, index);
}
