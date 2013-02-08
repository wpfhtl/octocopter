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

#include "pointcloud.h"

ParticleSystem::ParticleSystem(PointCloud *const pointcloud) : mPointCloudColliders(pointcloud)
{
    // set simulation parameters
    mSimulationParameters.worldMin = make_float3(0.0f, 0.0f, 0.0f);
    mSimulationParameters.worldMax = make_float3(0.0f, 0.0f, 0.0f);
    mSimulationParameters.gridSize = make_uint3(0, 0, 0);
    mSimulationParameters.particleCount = 0;
    mSimulationParameters.dampingMotion = 0.80f;                        // used only for integration
    mSimulationParameters.velocityFactorCollisionParticle = 0.02f;
    mSimulationParameters.velocityFactorCollisionBoundary = -0.5f;
    mSimulationParameters.gravity = make_float3(0.0, -9.810f, 0.0f);
    mSimulationParameters.spring = -1.1f;
    mSimulationParameters.shear = 0.0f;
    mSimulationParameters.attraction = -0.2f;

    setNullPointers();

    mIsInitialized = false;
    mNumberOfBytesAllocatedCpu = 0;
    mNumberOfBytesAllocatedGpu = 0;

    mColliderPointCloudWasUpdated = false;

    connect(mPointCloudColliders, SIGNAL(pointsInserted(VboInfo*const,quint32,quint32)), SLOT(slotNewCollidersInserted()));
}

void ParticleSystem::slotNewCollidersInserted()
{
    mColliderPointCloudWasUpdated = true;
}

void ParticleSystem::setNullPointers()
{
    mHostParticlePos = 0;
    mHostParticleVel = 0;
    mDeviceParticleVel = 0;
    mDeviceParticleSortedPos = 0;
    mDeviceParticleSortedVel = 0;
    mDeviceParticleMapGridCell = 0;
    mDeviceParticleMapIndex = 0;
    mDeviceColliderMapGridCell = 0;
    mDeviceColliderMapIndex = 0;
    mDeviceParticleCellStart = 0;
    mDeviceParticleCellEnd = 0;
    mDeviceColliderCellStart = 0;
    mDeviceColliderCellEnd = 0;
    mVboParticlePositions = 0;
    mVboParticleColors = 0;

    mDeviceParticleCollisionPositions = 0;
    mVboGridMapOfWayPointPressure = 0;
    mDeviceGridMapCellWorldPositions = 0;
    mVboColliderPositions = 0;
    mDeviceColliderSortedPos = 0;
}

void ParticleSystem::slotSetVolume(const QVector3D& min, const QVector3D& max)
{
    mSimulationParameters.worldMin = make_float3(min.x(), min.y(), min.z());
    mSimulationParameters.worldMax = make_float3(max.x(), max.y(), max.z());

    if(mIsInitialized) freeResources();
}

void ParticleSystem::showCollisionPositions()
{
    qDebug() << "ParticleSystem::showCollisionPositions():";

    QVector4D* collisionPositionsHost = new QVector4D[mSimulationParameters.particleCount];

    cudaMemcpy(collisionPositionsHost, mDeviceParticleCollisionPositions, sizeof(QVector4D) * mSimulationParameters.particleCount, cudaMemcpyDeviceToHost);

    for(int i=0;i<mSimulationParameters.particleCount;i++)
    {
        if(fabs(collisionPositionsHost[i].x()) > 0.2)
        {
            qDebug() << "last collision of particle" << i << "was in cell" << getGridCellHash(getGridCellCoordinate(collisionPositionsHost[i].toVector3D())) << "at" << collisionPositionsHost[i];
        }
    }

    delete collisionPositionsHost;
}

void ParticleSystem::showWaypointPressure()
{
    qDebug() << "ParticleSystem::showWaypointPressure():";

    quint8* waypointPressureMapDevice = (quint8*)mapGLBufferObject(&mCudaVboResourceGridMapOfWayPointPressure);

    const quint16 numberOfGridCells = mSimulationParameters.gridSize.x * mSimulationParameters.gridSize.y * mSimulationParameters.gridSize.z;

    quint8* waypointPressureMapHost = new quint8[numberOfGridCells];

    cudaMemcpy(waypointPressureMapHost, waypointPressureMapDevice, sizeof(quint8) * numberOfGridCells, cudaMemcpyDeviceToHost);

    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfWayPointPressure, 0);

    for(int i=0;i<numberOfGridCells;i++)
    {
        Q_ASSERT(i == getGridCellHash(getGridCellCoordinate(i)));
        const Vector3i cell = getGridCellCoordinate(i);
        if(waypointPressureMapHost[i] > 0)
        {
            qDebug() << "grid cell hash" << i << "at grid-coord" << cell.x << cell.y << cell.z << "and pos" << getGridCellCenter(cell) << "has waypoint pressure" << waypointPressureMapHost[i];
        }
    }

    delete waypointPressureMapHost;
}

bool ParticleSystem::getRankedWaypoints(QVector4D* const waypoints, const quint16 numberOfWaypointsRequested)
{
    if(!mIsInitialized)
    {
        qDebug() << "ParticleSystem::getRankedWaypoints(): not initialized yet, returning.";
        return false;
    }

    qDebug() << "ParticleSystem::getRankedWaypoints():";

    Q_ASSERT(numberOfWaypointsRequested <= mSimulationParameters.gridSize.x * mSimulationParameters.gridSize.y * mSimulationParameters.gridSize.z);

    const quint32 numberOfCells = mSimulationParameters.gridSize.x * mSimulationParameters.gridSize.y * mSimulationParameters.gridSize.z;

    // Copy waypoint pressure from VBO into mDeviceGridMapWayPointPressureSorted
    quint8* gridMapOfWayPointPressure = (quint8*)mapGLBufferObject(&mCudaVboResourceGridMapOfWayPointPressure);
    cudaMemcpy(mDeviceGridMapWayPointPressureSorted, gridMapOfWayPointPressure, sizeof(quint8) * numberOfCells, cudaMemcpyDeviceToDevice);
    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfWayPointPressure, 0);

    // Fill mDeviceGridMapCellWorldPositions - this might be done only once and then copied lateron (just like the waypoint pressure above)
    fillGridMapCellWorldPositions(mDeviceGridMapCellWorldPositions, numberOfCells);

    // Sort mDeviceGridMapWayPointPressureSorted => mDeviceGridMapCellWorldPositions according to the keys DESC
    sortGridMapWayPointPressure(mDeviceGridMapWayPointPressureSorted, mDeviceGridMapCellWorldPositions, numberOfCells, numberOfWaypointsRequested);

    // Copy the @count cells with the highest waypoint pressure into @waypoints
    cudaMemcpy(waypoints, mDeviceGridMapCellWorldPositions, sizeof(QVector4D) * numberOfWaypointsRequested, cudaMemcpyDeviceToHost);

    return true;
}

void ParticleSystem::slotClearGridWayPointPressure()
{
    const quint32 numberOfCells = mSimulationParameters.gridSize.x * mSimulationParameters.gridSize.y * mSimulationParameters.gridSize.z;

    quint8* gridMapOfWayPointPressure = (quint8*)mapGLBufferObject(&mCudaVboResourceGridMapOfWayPointPressure);
    cudaMemset(gridMapOfWayPointPressure, 0, sizeof(quint8) * numberOfCells);
    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfWayPointPressure, 0);
}

// Calculate a particle's hash value (=address in grid) from its containing cell (clamping to edges)
quint32 ParticleSystem::getGridCellHash(Vector3i gridCellCoordinate) const
{
    gridCellCoordinate.x = gridCellCoordinate.x & (mSimulationParameters.gridSize.x-1);  // wrap grid, assumes size is power of 2
    gridCellCoordinate.y = gridCellCoordinate.y & (mSimulationParameters.gridSize.y-1);
    gridCellCoordinate.z = gridCellCoordinate.z & (mSimulationParameters.gridSize.z-1);

    return (gridCellCoordinate.z * mSimulationParameters.gridSize.y) * mSimulationParameters.gridSize.x
            + (gridCellCoordinate.y * mSimulationParameters.gridSize.x)
            + gridCellCoordinate.x;
}

// Given the cell hash (=gl_PrimitiveIDIn), whats the 3d-grid-coordinate of the cell's center?
// This is the reverse of particleskernel.cu -> calcGridHash(int3 gridCell).
Vector3i ParticleSystem::getGridCellCoordinate(const quint32 hash) const
{
    Vector3i cell;
    cell.x = floor(fmod(hash, mSimulationParameters.gridSize.x));
    cell.y = floor(fmod(hash, mSimulationParameters.gridSize.x * mSimulationParameters.gridSize.y) / mSimulationParameters.gridSize.x);
    cell.z = floor(fmod(hash, mSimulationParameters.gridSize.x * mSimulationParameters.gridSize.y * mSimulationParameters.gridSize.z) / (mSimulationParameters.gridSize.x * mSimulationParameters.gridSize.y));
    return cell;
}

Vector3i ParticleSystem::getGridCellCoordinate(const QVector3D& worldPos) const
{
    const QVector3D posRelativeToGridMin = worldPos - QVector3D(mSimulationParameters.worldMin.x, mSimulationParameters.worldMin.y, mSimulationParameters.worldMin.z);
    const QVector3D cellSize = getGridCellSize();

    Vector3i cell;

    cell.x = floor(posRelativeToGridMin.x() / cellSize.x());
    cell.y = floor(posRelativeToGridMin.y() / cellSize.y());
    cell.z = floor(posRelativeToGridMin.z() / cellSize.z());

    return cell;
}

QVector3D ParticleSystem::getGridCellSize() const
{
    return QVector3D(
                (mSimulationParameters.worldMax.x - mSimulationParameters.worldMin.x) / mSimulationParameters.gridSize.x,
                (mSimulationParameters.worldMax.y - mSimulationParameters.worldMin.y) / mSimulationParameters.gridSize.y,
                (mSimulationParameters.worldMax.z - mSimulationParameters.worldMin.z) / mSimulationParameters.gridSize.z
                );
}

QVector3D ParticleSystem::getGridCellCenter(const Vector3i& gridCellCoordinate) const
{
    const QVector3D cellSize = getGridCellSize();

    return QVector3D(
                mSimulationParameters.worldMin.x + (cellSize.x() * gridCellCoordinate.x) + (cellSize.x() / 2.0),
                mSimulationParameters.worldMin.y + (cellSize.y() * gridCellCoordinate.y) + (cellSize.y() / 2.0),
                mSimulationParameters.worldMin.z + (cellSize.z() * gridCellCoordinate.z) + (cellSize.z() / 2.0)
                );
}

void ParticleSystem::initialize()
{
    Q_ASSERT(!mIsInitialized);

    size_t memTotal, memFree;
    cudaMemGetInfo(&memFree, &memTotal);
    qDebug() << "ParticleSystem::initialize(): before init, device has" << memFree / 1048576 << "of" << memTotal / 1048576 << "mb free.";

    // Set gridsize so that the cell-edges are never shorter than the particle's diameter! If particles were allowed to be larger than gridcells in
    // any dimension, we couldn't find all relevant neighbors by searching through only the (3*3*3)-1 = 8 cells immediately neighboring this one.
    // We can also modify this compromise by allowing larger particles and then searching (5*5*5)-1=124 cells. Haven't tried.

    // Using less (larger) cells is possible, it means less memory being used fro the grid, but more search being done when searching for neighbors
    // because more particles now occupy a single grid cell. This might not hurt performance as long as we do not cross an unknown threshold (kernel swap size)?!
    mSimulationParameters.gridSize.x = nextHigherPowerOfTwo((uint)ceil(getWorldSize().x() / (mSimulationParameters.particleRadius * 2.0f)))/* / 8.0*/;
    mSimulationParameters.gridSize.y = nextHigherPowerOfTwo((uint)ceil(getWorldSize().y() / (mSimulationParameters.particleRadius * 2.0f)))/* / 8.0*/;
    mSimulationParameters.gridSize.z = nextHigherPowerOfTwo((uint)ceil(getWorldSize().z() / (mSimulationParameters.particleRadius * 2.0f)))/* / 8.0*/;

    // hack!
    mSimulationParameters.gridSize.x = mSimulationParameters.gridSize.y = mSimulationParameters.gridSize.z = 64;

    const unsigned int numberOfCells = mSimulationParameters.gridSize.x * mSimulationParameters.gridSize.y * mSimulationParameters.gridSize.z;
    //    m_params.cellSize = make_float3(worldSize.x / m_gridSize.x, worldSize.y / m_gridSize.y, worldSize.z / m_gridSize.z);
//    float cellSize = mSimulationParameters.particleRadius * 2.0f;  // cell size equal to particle diameter
//    mSimulationParameters.cellSize = make_float3(cellSize, cellSize, cellSize);

    // allocate host storage for particle positions and velocities, then set them to zero
    mHostParticlePos = new float[mSimulationParameters.particleCount * 4];
    mHostParticleVel = new float[mSimulationParameters.particleCount * 4];
    mNumberOfBytesAllocatedCpu += mSimulationParameters.particleCount * 8;

    memset(mHostParticlePos, 0, mSimulationParameters.particleCount * 4 * sizeof(float));
    memset(mHostParticleVel, 0, mSimulationParameters.particleCount * 4 * sizeof(float));

    // determine GPU data-size
    const unsigned int memSizeParticleQuadrupels = sizeof(float) * 4 * mSimulationParameters.particleCount;

    // Allocate GPU data
    // Create VBO with particle positions. This is later given to particle renderer for visualization
    mVboParticlePositions = createVbo(memSizeParticleQuadrupels);
    mNumberOfBytesAllocatedGpu += memSizeParticleQuadrupels;
    cudaGraphicsGLRegisterBuffer(&mCudaVboResourceParticlePositions, mVboParticlePositions, cudaGraphicsMapFlagsNone);
    mVboParticleColors = createVbo(memSizeParticleQuadrupels);
    emit vboInfoParticles(mVboParticlePositions, mVboParticleColors, mSimulationParameters.particleCount);

    // Create VBO with collider positions. This is later given to particle renderer for visualization
    mVboColliderPositions = mPointCloudColliders->getVboInfo()[0].vbo;//createVbo(sizeof(float) * 4 * mSimulationParameters.colliderCountMax);
    cudaGraphicsGLRegisterBuffer(&mCudaVboResourceColliderPositions, mVboColliderPositions, cudaGraphicsMapFlagsNone);
    // use vboInfo.size or mSimulationParameters.colliderCountMax?

    cudaMalloc((void**)&mDeviceParticleVel, memSizeParticleQuadrupels);
    mNumberOfBytesAllocatedGpu += memSizeParticleQuadrupels;

    cudaMalloc((void**)&mDeviceColliderSortedPos, sizeof(float) * 4 * mPointCloudColliders->getCapacity());
    mNumberOfBytesAllocatedGpu += sizeof(float) * 4 * mPointCloudColliders->getCapacity();

    // Here, we store the positions of each particle's last collision with the colliders
    cudaMalloc((void**)&mDeviceParticleCollisionPositions, sizeof(float) * 4 * mSimulationParameters.particleCount);
    cudaMemset(mDeviceParticleCollisionPositions, 0, sizeof(float) * 4 * mSimulationParameters.particleCount); // set to (float)-zero
    mNumberOfBytesAllocatedGpu += sizeof(float) * 4 * mSimulationParameters.particleCount;

    { // init gridmap of waypoint pressure
        // Store the gridcell-waypoint-pressure-values in a VBO as 8bit-unsigned-ints
        mVboGridMapOfWayPointPressure = createVbo(sizeof(quint8) * numberOfCells);
        cudaGraphicsGLRegisterBuffer(&mCudaVboResourceGridMapOfWayPointPressure, mVboGridMapOfWayPointPressure, cudaGraphicsMapFlagsNone);
        // Set vbo values to zero
        cudaMemset(mapGLBufferObject(&mCudaVboResourceGridMapOfWayPointPressure), 0, sizeof(quint8) * numberOfCells);
        cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfWayPointPressure, 0);

        // Allocate the same size again for the same values. Here, they can be sorted in-place. Unfortunately, there is no thrust::sort_by_key()
        // which would leave the keys (=mVboGridMapOfWayPointPressure) alone and just rearrange the values (=mDeviceGridMapCellWorldPositions).
        cudaMalloc((void**)&mDeviceGridMapWayPointPressureSorted, sizeof(quint8) * numberOfCells);

        mNumberOfBytesAllocatedGpu += 2 * sizeof(quint8) * numberOfCells;

        // Allocate a vector of float4 for the world-positions of the gridcell positions. These will be the values when sorting
        cudaMalloc((void**)&mDeviceGridMapCellWorldPositions, sizeof(float4) * numberOfCells);
        mNumberOfBytesAllocatedGpu += sizeof(float4) * numberOfCells;
    }

    // Will contain QVector4Ds of worldpos of every gridcell, sorted by waypoint pressure
    cudaMalloc((void**)&mDeviceGridMapCellWorldPositions, sizeof(float) * 4 * numberOfCells);
    mNumberOfBytesAllocatedGpu += sizeof(float) * 4 * numberOfCells;

    emit vboInfoGridWaypointPressure(
                mVboGridMapOfWayPointPressure,
                QVector3D(mSimulationParameters.worldMin.x, mSimulationParameters.worldMin.y, mSimulationParameters.worldMin.z),
                QVector3D(mSimulationParameters.worldMax.x, mSimulationParameters.worldMax.y, mSimulationParameters.worldMax.z),
                gridCells()
                );

    // Sorted according to containing grid cell.
    cudaMalloc((void**)&mDeviceParticleSortedPos, memSizeParticleQuadrupels);
    mNumberOfBytesAllocatedGpu += memSizeParticleQuadrupels;
    cudaMalloc((void**)&mDeviceParticleSortedVel, memSizeParticleQuadrupels);
    mNumberOfBytesAllocatedGpu += memSizeParticleQuadrupels;

    // These two are used to map from gridcell (=hash) to particle id (=index). If we also know in which
    // indices of these arrays grid cells start and end, we can quickly find particles in neighboring cells...
    cudaMalloc((void**)&mDeviceParticleMapGridCell, mSimulationParameters.particleCount*sizeof(uint));
    cudaMalloc((void**)&mDeviceParticleMapIndex, mSimulationParameters.particleCount*sizeof(uint));
    mNumberOfBytesAllocatedGpu += mSimulationParameters.particleCount * sizeof(uint) * 2;

    // Same thing as above, just for colliders
    cudaMalloc((void**)&mDeviceColliderMapGridCell, mPointCloudColliders->getCapacity()*sizeof(uint));
    cudaMalloc((void**)&mDeviceColliderMapIndex, mPointCloudColliders->getCapacity()*sizeof(uint));
    mNumberOfBytesAllocatedGpu += mPointCloudColliders->getCapacity() * sizeof(uint) * 2;

    // ...and thats what we do here: in mDeviceCellStart[17], you'll find
    // where in mDeviceGridParticleHash cell 17 starts!
    cudaMalloc((void**)&mDeviceParticleCellStart, numberOfCells*sizeof(uint));
    cudaMalloc((void**)&mDeviceParticleCellEnd, numberOfCells*sizeof(uint));
    mNumberOfBytesAllocatedGpu += numberOfCells * sizeof(uint) * 2;

    // Same for colliders...
    cudaMalloc((void**)&mDeviceColliderCellStart, numberOfCells*sizeof(uint));
    cudaMalloc((void**)&mDeviceColliderCellEnd, numberOfCells*sizeof(uint));
    mNumberOfBytesAllocatedGpu += numberOfCells * sizeof(uint) * 2;

    cudaGraphicsGLRegisterBuffer(&mCudaColorVboResource, mVboParticleColors, cudaGraphicsMapFlagsNone);

    // fill color buffer
    glBindBufferARB(GL_ARRAY_BUFFER, mVboParticleColors);
    float *data = (float *) glMapBufferARB(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    for(unsigned int i=0; i<mSimulationParameters.particleCount; i++)
    {
        const float t = i / (float) mSimulationParameters.particleCount;

        colorRamp(t, data);
        data+=3;
        *data++ = 0.5f; // should be the alpha value
    }
    glUnmapBufferARB(GL_ARRAY_BUFFER);

    qDebug() << "ParticleSystem::initialize(): worldsize" << getWorldSize() << "and particle radius" << mSimulationParameters.particleRadius << ": created system with" << mSimulationParameters.particleCount << "particles and" << mSimulationParameters.gridSize.x << "*" << mSimulationParameters.gridSize.y << "*" << mSimulationParameters.gridSize.z << "cells";
    qDebug() << "ParticleSystem::initialize(): allocated" << mNumberOfBytesAllocatedCpu << "bytes on CPU," << mNumberOfBytesAllocatedGpu << "bytes on GPU.";

    setParameters(&mSimulationParameters);

    slotResetParticles();

    mIsInitialized = true;

    cudaMemGetInfo(&memFree, &memTotal);
    qDebug() << "ParticleSystem::initialize(): after init, device has" << memFree / 1048576 << "of" << memTotal / 1048576 << "mb free.";
}

void ParticleSystem::freeResources()
{
    Q_ASSERT(mIsInitialized);

    qDebug() << "ParticleSystem::freeResources(): freeing allocated memory...";

    delete [] mHostParticlePos;
    delete [] mHostParticleVel;

    cudaFree(mDeviceColliderSortedPos);
    cudaFree(mDeviceParticleVel);
    cudaFree(mDeviceParticleSortedPos);
    cudaFree(mDeviceParticleSortedVel);

    cudaFree(mDeviceParticleMapGridCell);
    cudaFree(mDeviceParticleMapIndex);
    cudaFree(mDeviceParticleCellStart);
    cudaFree(mDeviceParticleCellEnd);

    cudaFree(mDeviceColliderMapGridCell);
    cudaFree(mDeviceColliderMapIndex);
    cudaFree(mDeviceColliderCellStart);
    cudaFree(mDeviceColliderCellEnd);

    cudaFree(mDeviceParticleCollisionPositions);

    cudaFree(mDeviceGridMapCellWorldPositions);
    cudaFree(mDeviceGridMapWayPointPressureSorted);

    cudaGraphicsUnregisterResource(mCudaVboResourceParticlePositions);
    glDeleteBuffers(1, (const GLuint*)&mVboParticlePositions);
    glDeleteBuffers(1, (const GLuint*)&mVboParticleColors);

    cudaGraphicsUnregisterResource(mCudaVboResourceGridMapOfWayPointPressure);
    glDeleteBuffers(1, (const GLuint*)&mVboGridMapOfWayPointPressure);

    setNullPointers();

    mIsInitialized = false;
}

ParticleSystem::~ParticleSystem()
{
    if(mIsInitialized) freeResources();
}

unsigned int ParticleSystem::createVbo(quint32 size)
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

    QTime startTime = QTime::currentTime();
    startTime.start();

    // Get a pointer to the particle positions in the device by mapping GPU mem into CPU mem
    float *deviceParticlePositions = (float*)mapGLBufferObject(&mCudaVboResourceParticlePositions);

//    qDebug() << "ParticleSystem::update(): 0: getting pointer finished at" << startTime.elapsed();

    // Update constants
    setParameters(&mSimulationParameters);

//    qDebug() << "ParticleSystem::update(): 1: setting parameters finished at" << startTime.elapsed();

    quint8 *deviceGridMapOfWayPointPressure = (quint8*)mapGLBufferObject(&mCudaVboResourceGridMapOfWayPointPressure);

    // Integrate
    integrateSystem(
                deviceParticlePositions,                    // in/out: The particle positions, unsorted
                mDeviceParticleVel,                         // in/out: The particle velocities, unsorted
                deviceGridMapOfWayPointPressure,            // output: A grid mapping the 3d-space to waypoint pressure. Cells with high values (255) should be visited for information gain.
                mDeviceParticleCollisionPositions,          // input:  The particle's last collision position (or 0 if it didn't collide yet)
                deltaTime,                                  // input:  Timestep to be used for integration
                mSimulationParameters.particleCount         // input:  The number of particles
                );

    cudaGraphicsUnmapResources(1, &mCudaVboResourceGridMapOfWayPointPressure, 0);

//    showWaypointPressure();

//    qDebug() << "ParticleSystem::update(): 2: integrating system finished at" << startTime.elapsed();

    // Now that particles have been moved, they might be contained in different grid cells. So recompute the
    // mapping gridCell => particleIndex. This will allow fast neighbor searching in the grid during collision phase.
    computeMappingFromGridCellToParticle(
                mDeviceParticleMapGridCell,                 // output: The key - part of the particle gridcell->index map, unsorted
                mDeviceParticleMapIndex,                    // output: The value-part of the particle gridcell->index map, unsorted
                deviceParticlePositions,                    // input:  The particle positions after integration, unsorted and possibly colliding with other particles
                mSimulationParameters.particleCount);       // input:  The number of particles, one thread per particle

//    qDebug() << "ParticleSystem::update(): 3: computing particle spatial hash table finished at" << startTime.elapsed();

    // Sort the mapping gridCell => particleId on gridCell
    sortGridOccupancyMap(mDeviceParticleMapGridCell, mDeviceParticleMapIndex, mSimulationParameters.particleCount);

//    qDebug() << "ParticleSystem::update(): 4: sorting particle spatial hash table finished at" << startTime.elapsed();

    // Reorder particle arrays into sorted order and find start and end of each cell. The ordering of the particles is useful
    // only for the particle/particle-collisions. The collision code writes the post-collision velocities back into the
    // original, unsorted particle velocity array, where is will be used again in the next iteration's integrateSystem().
    sortParticlePosAndVelAccordingToGridCellAndFillCellStartAndEndArrays(
                mDeviceParticleCellStart,                           // output: At which index in mDeviceMapParticleIndex does cell X start?
                mDeviceParticleCellEnd,                             // output: At which index in mDeviceMapParticleIndex does cell X end?
                mDeviceParticleSortedPos,                   // output: The particle positions, sorted by gridcell
                mDeviceParticleSortedVel,                   // output: The particle velocities, sorted by gridcell
                mDeviceParticleMapGridCell,                 // input:  The key - part of the particle gridcell->index map, unsorted
                mDeviceParticleMapIndex,                    // input:  The value-part of the particle gridcell->index map, unsorted
                deviceParticlePositions,                    // input:  The particle-positions, unsorted
                mDeviceParticleVel,                         // input:  The particle-velocities, unsorted
                mSimulationParameters.particleCount,        // input:  The number of particles
                mSimulationParameters.gridSize.x * mSimulationParameters.gridSize.y * mSimulationParameters.gridSize.z  // input: Number of grid cells
                );

//    qDebug() << "ParticleSystem::update(): 5: computing particle navigation tables and sorting particles finished at" << startTime.elapsed();

    // Same for colliders
    if(mColliderPointCloudWasUpdated)
    {
        float *deviceColliderPositions = (float*)mapGLBufferObject(&mCudaVboResourceColliderPositions);

        computeMappingFromGridCellToParticle(
                    mDeviceColliderMapGridCell,                 // output: The key - part of the collider gridcell->index map, unsorted
                    mDeviceColliderMapIndex,                    // output: The value-part of the collider gridcell->index map, unsorted
                    deviceColliderPositions,                    // input:  The collider positions (no integration), unsorted and possibly colliding with particles
                    mPointCloudColliders->getVboInfo()[0].size  // input:  The number of colliders, one thread per particle
                    );

//        qDebug() << "ParticleSystem::update(): 6: computing collider spatial hash table finished at" << startTime.elapsed();

        sortGridOccupancyMap(mDeviceColliderMapGridCell, mDeviceColliderMapIndex, mPointCloudColliders->getVboInfo()[0].size);

//        qDebug() << "ParticleSystem::update(): 7: sorting collider spatial hash table finished at" << startTime.elapsed();

        sortParticlePosAndVelAccordingToGridCellAndFillCellStartAndEndArrays(
                    mDeviceColliderCellStart,                   // output: At which index in mDeviceMapColliderIndex does cell X start?
                    mDeviceColliderCellEnd,                     // output: At which index in mDeviceMapColliderIndex does cell X end?
                    mDeviceColliderSortedPos,                   // output: The collider positions, sorted by gridcell
                    0,                                          // output: The collider velocities, sorted by gridcell / they have no vel, so pass 0
                    mDeviceColliderMapGridCell,                 // input:  The key - part of the collider gridcell->index map, unsorted
                    mDeviceColliderMapIndex,                    // input:  The value-part of the collider gridcell->index map, unsorted
                    deviceColliderPositions,                    // input:  The particle-positions, unsorted
                    0,                                          // input:  The particle-velocities, unsorted / they have no vel, so pass 0
                    mPointCloudColliders->getVboInfo()[0].size, // input:  The number of colliders
                    mSimulationParameters.gridSize.x * mSimulationParameters.gridSize.y * mSimulationParameters.gridSize.z  // input: Number of grid cells
                    );

        cudaGraphicsUnmapResources(1, &mCudaVboResourceColliderPositions, 0);

        mColliderPointCloudWasUpdated = false;
    }

//    qDebug() << "ParticleSystem::update(): 8: computing collider navigation tables and sorting particles finished at" << startTime.elapsed();

    // process collisions between particles
    collideParticlesWithParticlesAndColliders(
                mDeviceParticleVel,                         // output: The particle velocities
                mDeviceParticleCollisionPositions,          // output: Every particle's position of last collision, or 0.0/0.0/0.0 if none occurred.

                mDeviceParticleSortedPos,                   // input:  The particle positions, sorted by gridcell
                mDeviceParticleSortedVel,                   // input:  The particle velocities, sorted by gridcell
                mDeviceParticleMapIndex,                    // input:  The value-part of the particle gridcell->index map, sorted by gridcell
                mDeviceParticleCellStart,                   // input:  At which index in mDeviceMapParticleIndex does cell X start?
                mDeviceParticleCellEnd,                     // input:  At which index in mDeviceMapParticleIndex does cell X end?

                mDeviceColliderSortedPos,                   // input:  The collider positions, sorted by gridcell
                mDeviceColliderMapIndex,                    // input:  The value-part of the collider gridcell->index map, sorted by gridcell
                mDeviceColliderCellStart,                   // input:  At which index in mDeviceMapColliderIndex does cell X start?
                mDeviceColliderCellEnd,                     // input:  At which index in mDeviceMapColliderIndex does cell X end?

                mSimulationParameters.particleCount,        // input:  How many particles to collide against other particles (one thread per particle)
                mSimulationParameters.gridSize.x * mSimulationParameters.gridSize.y * mSimulationParameters.gridSize.z  // input: Number of grid cells
                );

//    showCollisionPositions();

//    qDebug() << "ParticleSystem::update(): 9: colliding particles finished at" << startTime.elapsed();

    // Unmap at end here to avoid unnecessary graphics/CUDA context switch.
    // Once unmapped, the resource may not be accessed by CUDA until they
    // are mapped again. This function provides the synchronization guarantee
    // that any CUDA work issued  before ::cudaGraphicsUnmapResources()
    // will complete before any subsequently issued graphics work begins.
    cudaGraphicsUnmapResources(1, &mCudaVboResourceParticlePositions, 0);

    size_t memTotal, memFree;
    cudaMemGetInfo(&memFree, &memTotal);
//    qDebug() << "ParticleSystem::update(): finished, fps:" << 1000.0f/startTime.elapsed() << "free mem:" << memFree / 1048576;
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
        cudaGraphicsUnregisterResource(mCudaVboResourceParticlePositions);

        glBindBuffer(GL_ARRAY_BUFFER, mVboParticlePositions);
        glBufferSubData(GL_ARRAY_BUFFER, start*4*sizeof(float), count*4*sizeof(float), data);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        cudaGraphicsGLRegisterBuffer(&mCudaVboResourceParticlePositions, mVboParticlePositions, cudaGraphicsMapFlagsNone);
    }
    else if(array == ArrayVelocities)
    {
        cudaMemcpy(
                    (char*) mDeviceParticleVel + start*4*sizeof(float), // destination
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

void ParticleSystem::slotResetParticles()
{
    switch(mDefaultParticlePlacement)
    {
    case ParticlePlacement::PlacementRandom:
    {
        int p = 0, v = 0;

        qDebug() << "ParticleSystem::reset(): world min" << mSimulationParameters.worldMin.x << mSimulationParameters.worldMin.y << mSimulationParameters.worldMin.z <<
        "max" << mSimulationParameters.worldMax.x << mSimulationParameters.worldMax.y << mSimulationParameters.worldMax.z;

        for(unsigned int i=0; i < mSimulationParameters.particleCount; i++)
        {
            mHostParticlePos[p++] = mSimulationParameters.worldMin.x + (mSimulationParameters.worldMax.x - mSimulationParameters.worldMin.x) * frand();
            mHostParticlePos[p++] = mSimulationParameters.worldMin.y + (mSimulationParameters.worldMax.y - mSimulationParameters.worldMin.y) * frand();
            mHostParticlePos[p++] = mSimulationParameters.worldMin.z + (mSimulationParameters.worldMax.z - mSimulationParameters.worldMin.z) * frand();
            mHostParticlePos[p++] = 1.0f;

            mHostParticleVel[v++] = 0.0f;
            mHostParticleVel[v++] = 0.0f;
            mHostParticleVel[v++] = 0.0f;
            mHostParticleVel[v++] = 0.0f;
        }
        break;
    }

    case ParticlePlacement::PlacementGrid:
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
                        mHostParticlePos[i*4+0] = (spacing * x) + mSimulationParameters.particleRadius - 1.0f + (frand()*2.0f-1.0f)*jitter;
                        mHostParticlePos[i*4+1] = (spacing * y) + mSimulationParameters.particleRadius - 1.0f + (frand()*2.0f-1.0f)*jitter;
                        mHostParticlePos[i*4+2] = (spacing * z) + mSimulationParameters.particleRadius - 1.0f + (frand()*2.0f-1.0f)*jitter;
                        mHostParticlePos[i*4+3] = 1.0f;

                        mHostParticleVel[i*4+0] = 0.0f;
                        mHostParticleVel[i*4+1] = 0.0f;
                        mHostParticleVel[i*4+2] = 0.0f;
                        mHostParticleVel[i*4+3] = 0.0f;
                    }
                }
            }
        }
        break;
    }

    case ParticlePlacement::PlacementFillSky:
    {
        float jitter = mSimulationParameters.particleRadius * 0.1f;
        const float spacing = mSimulationParameters.particleRadius * 2.02f;

        unsigned int particleNumber = 0;

        for(
            float y = mSimulationParameters.worldMax.y - mSimulationParameters.particleRadius;
            y >= mSimulationParameters.worldMin.y + mSimulationParameters.particleRadius && particleNumber < mSimulationParameters.particleCount;
            y -= spacing)
        {

            for(
                float x = mSimulationParameters.worldMin.x + mSimulationParameters.particleRadius;
                x <= mSimulationParameters.worldMax.x - mSimulationParameters.particleRadius && particleNumber < mSimulationParameters.particleCount;
                x += spacing)
            {
                for(
                    float z = mSimulationParameters.worldMin.z + mSimulationParameters.particleRadius;
                    z <= mSimulationParameters.worldMax.z - mSimulationParameters.particleRadius && particleNumber < mSimulationParameters.particleCount;
                    z += spacing)
                {
//                    qDebug() << "moving particle" << particleNumber << "to" << x << y << z;
                    mHostParticlePos[particleNumber*4+0] = x + (frand()-0.5) * jitter;
                    mHostParticlePos[particleNumber*4+1] = y + (frand()-0.5) * jitter;
                    mHostParticlePos[particleNumber*4+2] = z + (frand()-0.5) * jitter;
                    mHostParticlePos[particleNumber*4+3] = 1.0f;

                    mHostParticleVel[particleNumber*4+0] = 0.0f;
                    mHostParticleVel[particleNumber*4+1] = 0.0f;
                    mHostParticleVel[particleNumber*4+2] = 0.0f;
                    mHostParticleVel[particleNumber*4+3] = 0.0f;

                    particleNumber++;
                }
            }
        }
        break;
    }
    }

    setArray(ArrayPositions, mHostParticlePos, 0, mSimulationParameters.particleCount);
    setArray(ArrayVelocities, mHostParticleVel, 0, mSimulationParameters.particleCount);
}

/*
void ParticleSystem::appendCollidersFromOctree()
{
    // mOctreeDense probably contains more points now than it contained last time. The points are already on the GPU
    // as they're stored in a VBO for rendering. So use a CUDA kernel to check for each of these points whether it
    // should be inserted into the colliderpos list.
    mOctreeDense->updateVbo();
}
*/
