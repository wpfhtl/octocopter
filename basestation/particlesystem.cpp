#include "particlesystem.h"
#include "particlesystem_cuda.cuh"

#include <GL/glew.h>
#include <cuda_gl_interop.h>
#include <cuda_runtime_api.h>

#include <assert.h>
#include <math.h>
#include <memory.h>
#include <cstdio>
#include <cstdlib>
#include <algorithm>

ParticleSystem::ParticleSystem()
{
    // set simulation parameters
    mSimulationParameters.worldMin = make_float3(0.0f, 0.0f, 0.0f);
    mSimulationParameters.worldMax = make_float3(0.0f, 0.0f, 0.0f);
    mSimulationParameters.gridSize = make_uint3(0, 0, 0);
    mSimulationParameters.particleCount = 1024;
    mSimulationParameters.dampingMotion = 0.98f;
    mSimulationParameters.velocityFactorCollisionParticle = 0.02f;
    mSimulationParameters.velocityFactorCollisionBoundary = -0.7f;
    mSimulationParameters.gravity = make_float3(0.0, -0.2f, 0.0f);
    mSimulationParameters.spring = 0.5f;
    mSimulationParameters.shear = 0.1f;
    mSimulationParameters.attraction = 0.0f;

    setNullPointers();

    mIsInitialized = false;
}

void ParticleSystem::setNullPointers()
{
    mHostPos = 0;
    mHostVel = 0;
    mHostCellStart = 0;
    mHostCellEnd = 0;
    mDeviceVel = 0;
    mDeviceSortedPos = 0;
    mDeviceSortedVel = 0;
    mDeviceGridParticleHash = 0;
    mDeviceGridParticleIndex = 0;
    mDeviceCellStart = 0;
    mDeviceCellEnd = 0;
    mVboPositions = 0;
    mVboColors = 0;
}

void ParticleSystem::slotSetVolume(const QVector3D& min, const QVector3D& max)
{
    mSimulationParameters.worldMin = make_float3(min.x(), min.y(), min.z());
    mSimulationParameters.worldMax = make_float3(max.x(), max.y(), max.z());

    if(mIsInitialized) freeResources();
}

void ParticleSystem::initialize()
{
    Q_ASSERT(!mIsInitialized);

    // Set gridsize so that the cell-edges are never longer than the particle's diameter! This assures that no more than
    // 4 particles can occupy a grid cell. Not sure whether this really is still necessary, tests show it also works with a quarter of the cells!
    mSimulationParameters.gridSize.x = nextHigherPowerOfTwo((uint)ceil(getWorldSize().x() / (mSimulationParameters.particleRadius * 2.0f))) / 8.0;
    mSimulationParameters.gridSize.y = nextHigherPowerOfTwo((uint)ceil(getWorldSize().y() / (mSimulationParameters.particleRadius * 2.0f))) / 8.0;
    mSimulationParameters.gridSize.z = nextHigherPowerOfTwo((uint)ceil(getWorldSize().z() / (mSimulationParameters.particleRadius * 2.0f))) / 8.0;

    const unsigned int numberOfCells = mSimulationParameters.gridSize.x * mSimulationParameters.gridSize.y * mSimulationParameters.gridSize.z;
    //    m_params.cellSize = make_float3(worldSize.x / m_gridSize.x, worldSize.y / m_gridSize.y, worldSize.z / m_gridSize.z);
//    float cellSize = mSimulationParameters.particleRadius * 2.0f;  // cell size equal to particle diameter
//    mSimulationParameters.cellSize = make_float3(cellSize, cellSize, cellSize);

    // allocate host storage for particle positions and velocities, then set them to zero
    mHostPos = new float[mSimulationParameters.particleCount * 4];
    mHostVel = new float[mSimulationParameters.particleCount * 4];
    memset(mHostPos, 0, mSimulationParameters.particleCount * 4 * sizeof(float));
    memset(mHostVel, 0, mSimulationParameters.particleCount * 4 * sizeof(float));

    mHostCellStart = new uint[numberOfCells];
    memset(mHostCellStart, 0, numberOfCells * sizeof(uint));

    mHostCellEnd = new uint[numberOfCells];
    memset(mHostCellEnd, 0, numberOfCells * sizeof(uint));

    // determine GPU data-size
    const unsigned int memSizeParticleQuadrupels = sizeof(float) * 4 * mSimulationParameters.particleCount;

    // Allocate GPU data
    // Create VBO with particle positions. This is later given to particle renderer for visualization
    mVboPositions = createVbo(memSizeParticleQuadrupels);
    emit vboPositionChanged(mVboPositions, mSimulationParameters.particleCount);

    cudaGraphicsGLRegisterBuffer(&mCudaPositionVboResource, mVboPositions, cudaGraphicsMapFlagsNone);
//    registerGLBufferObject(mVboPositions, &mCudaPositionVboResource);

    cudaMalloc((void**)&mDeviceVel, memSizeParticleQuadrupels);
    cudaMalloc((void**)&mDeviceSortedPos, memSizeParticleQuadrupels);
    cudaMalloc((void**)&mDeviceSortedVel, memSizeParticleQuadrupels);
    cudaMalloc((void**)&mDeviceGridParticleHash, mSimulationParameters.particleCount*sizeof(uint));
    cudaMalloc((void**)&mDeviceGridParticleIndex, mSimulationParameters.particleCount*sizeof(uint));
    cudaMalloc((void**)&mDeviceCellStart, numberOfCells*sizeof(uint));
    cudaMalloc((void**)&mDeviceCellEnd, numberOfCells*sizeof(uint));

    // added by ben:
    //cudaMalloc((void**)&mDeviceCollisionPositions, memSizeParticleQuadrupels);

    mVboColors = createVbo(memSizeParticleQuadrupels);
    emit vboColorChanged(mVboColors);

    cudaGraphicsGLRegisterBuffer(&mCudaColorVboResource, mVboColors, cudaGraphicsMapFlagsNone);
//    registerGLBufferObject(mVboPositions, &mCudaPositionVboResource);

    // fill color buffer
    glBindBufferARB(GL_ARRAY_BUFFER, mVboColors);
    float *data = (float *) glMapBufferARB(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    for(unsigned int i=0; i<mSimulationParameters.particleCount; i++)
    {
        const float t = i / (float) mSimulationParameters.particleCount;

        colorRamp(t, data);
        data+=3;
        *data++ = 1.0f;
    }
    glUnmapBufferARB(GL_ARRAY_BUFFER);

    qDebug() << "ParticleSystem::initialize(): worldsize" << getWorldSize() << "and particle radius" << mSimulationParameters.particleRadius << ": created system with" << mSimulationParameters.particleCount << "particles and" << mSimulationParameters.gridSize.x << "*" << mSimulationParameters.gridSize.y << "*" << mSimulationParameters.gridSize.z << "cells";

    setParameters(&mSimulationParameters);

    placeParticles();

    mIsInitialized = true;
}

void ParticleSystem::freeResources()
{
    Q_ASSERT(mIsInitialized);

    qDebug() << "ParticleSystem::freeResources(): freeing allocated memory...";

    delete [] mHostPos;
    delete [] mHostVel;
    delete [] mHostCellStart;
    delete [] mHostCellEnd;

    cudaFree(mDeviceVel);
    cudaFree(mDeviceSortedPos);
    cudaFree(mDeviceSortedVel);

    cudaFree(mDeviceGridParticleHash);
    cudaFree(mDeviceGridParticleIndex);
    cudaFree(mDeviceCellStart);
    cudaFree(mDeviceCellEnd);

    cudaGraphicsUnregisterResource(mCudaPositionVboResource);
    glDeleteBuffers(1, (const GLuint*)&mVboPositions);
    glDeleteBuffers(1, (const GLuint*)&mVboColors);

    setNullPointers();

    mIsInitialized = false;
}

ParticleSystem::~ParticleSystem()
{
    if(mIsInitialized) freeResources();
}

unsigned int ParticleSystem::createVbo(unsigned int size)
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
void ParticleSystem::update(const float deltaTime)
{
    if(mSimulationParameters.worldMax.x == 0.0f) return;

    if(!mIsInitialized) initialize();

    // Get a pointer to the particle positions in the device by mapping GPU mem into CPU mem
    float *dPos = (float *) mapGLBufferObject(&mCudaPositionVboResource);

    // Update constants
    setParameters(&mSimulationParameters);

    // Integrate
    integrateSystem(
                dPos,       // in/out:  particle positions
                mDeviceVel, // in/out:  particle velocities
                deltaTime,  // in:      intergate which timestep
                mSimulationParameters.particleCount);

    // Calculate grid hash
    calcHash(
                mDeviceGridParticleHash,    // out: ?
                mDeviceGridParticleIndex,   // out: ?
                dPos,                       // in:  particle positions after integration, possibly colliding
                mSimulationParameters.particleCount);

    // Sort particles based on hash. Documentation says:
    // As an additional optimization, we re-order the position and velocity arrays into sorted
    // order to improve the coherence of the texture lookups during the collision processing stage
    // So this means we can leave it out for now.
    sortParticles(mDeviceGridParticleHash, mDeviceGridParticleIndex, mSimulationParameters.particleCount);

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
                mSimulationParameters.particleCount,
                mSimulationParameters.gridSize.x * mSimulationParameters.gridSize.y * mSimulationParameters.gridSize.z);

    // process collisions
    collide(
                mDeviceVel,
                mDeviceSortedPos,
                mDeviceSortedVel,
                mDeviceGridParticleIndex,
                mDeviceCellStart,
                mDeviceCellEnd,
                mSimulationParameters.particleCount,
                mSimulationParameters.gridSize.x * mSimulationParameters.gridSize.y * mSimulationParameters.gridSize.z);

    // note: do unmap at end here to avoid unnecessary graphics/CUDA context switch
    cudaGraphicsUnmapResources(1, &mCudaPositionVboResource, 0);
}

void ParticleSystem::slotSetParticleRadius(float radius)
{
    mSimulationParameters.particleRadius = radius;
    qDebug() << "ParticleSystem::slotSetParticleRadius(): setting particle radius to" << mSimulationParameters.particleRadius;

    // The particle-radius must be at least half the cell-size. Otherwise, we might have more than 4 particles in a cell,
    // which might break collisions. I'm not sure this really is a problem, though, need to investigate further.
    // If the particles are now so small that more than 4 can fit into a cell, re-create the grid with more cells
    QVector3D cellSize;
    cellSize.setX(getWorldSize().x()/mSimulationParameters.gridSize.x);
    cellSize.setY(getWorldSize().y()/mSimulationParameters.gridSize.y);
    cellSize.setZ(getWorldSize().z()/mSimulationParameters.gridSize.z);
    const float shortestCellSide = std::min(std::min(cellSize.x(), cellSize.y()), cellSize.z());
    if(shortestCellSide > radius * 2.0f)
    {
        qDebug() << "ParticleSystem::slotSetParticleRadius(): re-setting particle system because particles are now too small for the grid";
        if(mIsInitialized) freeResources();
    }

    emit particleRadiusChanged(radius);
}

// Write positions or velocities of the particles into the GPU memory
void ParticleSystem::setArray(ParticleArray array, const float* data, int start, int count)
{
    if(array == ArrayPositions)
    {
        cudaGraphicsUnregisterResource(mCudaPositionVboResource);
        glBindBuffer(GL_ARRAY_BUFFER, mVboPositions);
        glBufferSubData(GL_ARRAY_BUFFER, start*4*sizeof(float), count*4*sizeof(float), data);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        cudaGraphicsGLRegisterBuffer(&mCudaPositionVboResource, mVboPositions, cudaGraphicsMapFlagsNone);
//        registerGLBufferObject(mVboPositions, &mCudaPositionVboResource);
    }
    else if(array == ArrayVelocities)
    {
        cudaMemcpy(
                    (char*) mDeviceVel + start*4*sizeof(float), // destination
                    data,                                       // source
                    count*4*sizeof(float),                      // count
                    cudaMemcpyHostToDevice                      // copy-kind
                    );
    }
}

inline float frand()
{
    return rand() / (float)RAND_MAX;
}

void ParticleSystem::placeParticles()
{
    switch(mDefaultParticlePlacement)
    {
    case PlacementRandom:
    {
        int p = 0, v = 0;

        qDebug() << "ParticleSystem::reset(): world min" << mSimulationParameters.worldMin.x << mSimulationParameters.worldMin.y << mSimulationParameters.worldMin.z <<
        "max" << mSimulationParameters.worldMax.x << mSimulationParameters.worldMax.y << mSimulationParameters.worldMax.z;

        for(unsigned int i=0; i < mSimulationParameters.particleCount; i++)
        {
            mHostPos[p++] = mSimulationParameters.worldMin.x + (mSimulationParameters.worldMax.x - mSimulationParameters.worldMin.x) * frand();
            mHostPos[p++] = mSimulationParameters.worldMin.y + (mSimulationParameters.worldMax.y - mSimulationParameters.worldMin.y) * frand();
            mHostPos[p++] = mSimulationParameters.worldMin.z + (mSimulationParameters.worldMax.z - mSimulationParameters.worldMin.z) * frand();
            mHostPos[p++] = 1.0f;
            mHostVel[v++] = 0.0f;
            mHostVel[v++] = 0.0f;
            mHostVel[v++] = 0.0f;
            mHostVel[v++] = 0.0f;
        }
        break;
    }

    case PlacementGrid:
    {
        const float jitter = mSimulationParameters.particleRadius * 0.01f;
        const float spacing = mSimulationParameters.particleRadius * 2.0f;

        // If we want a cube, determine the number of particles in each dimension
        unsigned int s = (int) ceilf(powf((float) mSimulationParameters.particleCount, 1.0f / 3.0f));
        unsigned int gridSize[3];
        gridSize[0] = gridSize[1] = gridSize[2] = s;

        srand(1973);
        for(unsigned int z=0; z<gridSize[2]; z++)
        {
            for(unsigned int y=0; y<gridSize[1]; y++)
            {
                for(unsigned int x=0; x<gridSize[0]; x++)
                {
                    unsigned int i = (z*gridSize[1]*gridSize[0]) + (y*gridSize[0]) + x;
                    if (i < mSimulationParameters.particleCount)
                    {
                        mHostPos[i*4+0] = (spacing * x) + mSimulationParameters.particleRadius - 1.0f + (frand()*2.0f-1.0f)*jitter;
                        mHostPos[i*4+1] = (spacing * y) + mSimulationParameters.particleRadius - 1.0f + (frand()*2.0f-1.0f)*jitter;
                        mHostPos[i*4+2] = (spacing * z) + mSimulationParameters.particleRadius - 1.0f + (frand()*2.0f-1.0f)*jitter;
                        mHostPos[i*4+3] = 1.0f;

                        mHostVel[i*4+0] = 0.0f;
                        mHostVel[i*4+1] = 0.0f;
                        mHostVel[i*4+2] = 0.0f;
                        mHostVel[i*4+3] = 0.0f;
                    }
                }
            }
        }
        break;
    }

    case PlacementFillSky:
    {
        float jitter = mSimulationParameters.particleRadius * 0.01f;
        const float spacing = mSimulationParameters.particleRadius * 2.02f;

        unsigned int particleNumber = 0;

        for(
            float y = mSimulationParameters.worldMax.y - mSimulationParameters.particleRadius;
            y >= mSimulationParameters.worldMin.y + mSimulationParameters.particleRadius && particleNumber <= mSimulationParameters.particleCount;
            y -= spacing)
        {

            for(
                float x = mSimulationParameters.worldMin.x + mSimulationParameters.particleRadius;
                x <= mSimulationParameters.worldMax.x - mSimulationParameters.particleRadius && particleNumber <= mSimulationParameters.particleCount;
                x += spacing)
            {
                for(
                    float z = mSimulationParameters.worldMin.z + mSimulationParameters.particleRadius;
                    z <= mSimulationParameters.worldMax.z - mSimulationParameters.particleRadius && particleNumber <= mSimulationParameters.particleCount;
                    z += spacing)
                {
//                    qDebug() << "moving particle" << particleNumber << "to" << x << y << z;
                    mHostPos[particleNumber*4+0] = x + (frand()-0.5) * jitter;
                    mHostPos[particleNumber*4+1] = y + (frand()-0.5) * jitter;
                    mHostPos[particleNumber*4+2] = z + (frand()-0.5) * jitter;
                    mHostPos[particleNumber*4+3] = 1.0f;

                    mHostVel[particleNumber*4+0] = 0.0f;
                    mHostVel[particleNumber*4+1] = 0.0f;
                    mHostVel[particleNumber*4+2] = 0.0f;
                    mHostVel[particleNumber*4+3] = 0.0f;

                    particleNumber++;
                }
            }
        }
        break;
    }
    }

    setArray(ArrayPositions, mHostPos, 0, mSimulationParameters.particleCount);
    setArray(ArrayVelocities, mHostVel, 0, mSimulationParameters.particleCount);
}
