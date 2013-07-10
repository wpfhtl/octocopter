#include "particlesystem.h"
#include "particlesystem.cuh"
#include <thrust/version.h>

#include <cuda_gl_interop.h>
#include <cuda_runtime_api.h>
#include "cudahelper.h"
#include "cudahelper.cuh"

#include <assert.h>
#include <math.h>
#include <memory.h>
#include <cstdio>
#include <cstdlib>
#include <algorithm>

#include <openglutilities.h>
#include "pointcloudcuda.h"

ParticleSystem::ParticleSystem(PointCloudCuda *const pointCloudDense, PointCloudCuda *const pointCloudColliders, ParametersParticleSystem *const simulationParameters) :
    mPointCloudDense(pointCloudDense),
    mPointCloudColliders(pointCloudColliders)
{
    qDebug() << "ParticleSystem::ParticleSystem(): using thrust version" << THRUST_MAJOR_VERSION << "." << THRUST_MINOR_VERSION;

    // Our collision cloud shouldn't contain points outside the bbox - that'd be a waste of resources
    mPointCloudColliders->setAcceptPointsOutsideBoundingBox(false);

    // set simulation parameters
    mParametersSimulation = simulationParameters;

    setNullPointers();

    mIsInitialized = false;
    mNumberOfBytesAllocatedCpu = 0;
    mNumberOfBytesAllocatedGpu = 0;

    // We set this to true, so that even with no colliders present, the cellIndexStart-
    // vector will be initialized with 0x777777 once on startup
    mUpdateMappingFromColliderToGridCell = true;

    mParametersSimulation->gridParticleSystem.worldMin = make_float3(-56.0f, -8.0f, -50.0f);
    mParametersSimulation->gridParticleSystem.worldMax = make_float3(8.0f, 26.0f, 14.0f);

    connect(mPointCloudColliders, SIGNAL(pointsInserted(PointCloud*const,quint32,quint32)), SLOT(slotNewCollidersInserted()));
}

void ParticleSystem::slotNewCollidersInserted()
{
    mUpdateMappingFromColliderToGridCell = true;
}

void ParticleSystem::setNullPointers()
{
    //return;
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
    mVboColliderPositions = 0;
    mDeviceColliderSortedPos = 0;
}

void ParticleSystem::slotSetVolume(const QVector3D& min, const QVector3D& max)
{
    // Set the new volume, and re-set some parts of the datastructure only if
    // the size has changed. Otherwise, we can simply move the whole system.
    mParametersSimulation->gridParticleSystem.worldMin = make_float3(min.x(), min.y(), min.z());
    mParametersSimulation->gridParticleSystem.worldMax = make_float3(max.x(), max.y(), max.z());

    mPointCloudColliders->setBoundingBox(min, max);

    // TODO: re-initialize when size changes!

    // We moved the particle system. This means that A) the system's sparse collider-pointcloud now
    // contains points outside of the particle system and B) the dense pointcloud contains points
    // that are NOT in the collider-pointcloud - but should be. So, we re-populate the sparse point-
    // cloud with points from the dense pointcloud.
    mPointCloudColliders->slotReset();
    mPointCloudColliders->slotInsertPoints(mPointCloudDense);

    // Usually, the mapping from collider to grid-cells only changes when the collider pointcloud changes.
    // But when we move the grid (relative to the colliders), the mapping also needs an update!
    mUpdateMappingFromColliderToGridCell = true;

    emit vboInfoParticles(
                mVboParticlePositions,
                mVboParticleColors,
                mParametersSimulation->particleCount,
                getVector(mParametersSimulation->gridParticleSystem.worldMin),
                getVector(mParametersSimulation->gridParticleSystem.worldMax)
                );
}

void ParticleSystem::showCollisionPositions()
{
    qDebug() << "ParticleSystem::showCollisionPositions():";

    QVector4D* collisionPositionsHost = new QVector4D[mParametersSimulation->particleCount];

    cudaMemcpy(collisionPositionsHost, mDeviceParticleCollisionPositions, sizeof(QVector4D) * mParametersSimulation->particleCount, cudaMemcpyDeviceToHost);

    for(int i=0;i<mParametersSimulation->particleCount;i++)
    {
        if(fabs(collisionPositionsHost[i].x()) > 0.2)
        {
            qDebug() << "last collision of particle" << i << "was in cell" <<
                        //getGridCellHash(mSimulationParameters->gridParticleSystem, getGridCellCoordinate(mSimulationParameters->gridParticleSystem, collisionPositionsHost[i].toVector3D())) << "at" << collisionPositionsHost[i];
                        mParametersSimulation->gridParticleSystem.getCellHash(mParametersSimulation->gridParticleSystem.getCellCoordinate(CudaHelper::cudaConvert(collisionPositionsHost[i].toVector3D()))) << "at" << collisionPositionsHost[i];
        }
    }

    delete collisionPositionsHost;
}

void ParticleSystem::initialize()
{
    Q_ASSERT(!mIsInitialized);

    // This needs to be called only once, but here it might get called more often. Shouldn't be a problem.
    initializeOpenGLFunctions();

    size_t memTotal, memFree;
    cudaSafeCall(cudaMemGetInfo(&memFree, &memTotal));
    qDebug() << "ParticleSystem::initialize(): before init, device has" << memFree / 1048576 << "of" << memTotal / 1048576 << "mb free.";

    // Set gridsize so that the cell-edges are never shorter than the particle's diameter! If particles were allowed to be larger than gridcells in
    // any dimension, we couldn't find all relevant neighbors by searching through only the (3*3*3)-1 = 8 cells immediately neighboring this one.
    // We can also modify this compromise by allowing larger particles and then searching (5*5*5)-1=124 cells. Haven't tried.

    // Using less (larger) cells is possible, it means less memory being used for the grid, but more search being done when searching for neighbors
    // because more particles now occupy a single grid cell. This might not hurt performance as long as we do not cross an unknown threshold (kernel swap size)?!
    const QVector3D particleSystemWorldSize = CudaHelper::cudaConvert(mParametersSimulation->gridParticleSystem.getWorldSize());
    mParametersSimulation->gridParticleSystem.cells.x = nextHigherPowerOfTwo((uint)ceil(particleSystemWorldSize.x() / (mParametersSimulation->particleRadius * 2.0f)))/* / 8.0*/;
    mParametersSimulation->gridParticleSystem.cells.y = nextHigherPowerOfTwo((uint)ceil(particleSystemWorldSize.y() / (mParametersSimulation->particleRadius * 2.0f)))/* / 8.0*/;
    mParametersSimulation->gridParticleSystem.cells.z = nextHigherPowerOfTwo((uint)ceil(particleSystemWorldSize.z() / (mParametersSimulation->particleRadius * 2.0f)))/* / 8.0*/;

    // hack!
    mParametersSimulation->gridParticleSystem.cells.x = mParametersSimulation->gridParticleSystem.cells.y = mParametersSimulation->gridParticleSystem.cells.z = 64;

    const quint32 numberOfCells = mParametersSimulation->gridParticleSystem.cells.x * mParametersSimulation->gridParticleSystem.cells.y * mParametersSimulation->gridParticleSystem.cells.z;
    //    m_params.cellSize = make_float3(worldSize.x / m_gridSize.x, worldSize.y / m_gridSize.y, worldSize.z / m_gridSize.z);
//    float cellSize = mSimulationParameters->particleRadius * 2.0f;  // cell size equal to particle diameter
//    mSimulationParameters->cellSize = make_float3(cellSize, cellSize, cellSize);

    // allocate host storage for particle positions and velocities, then set them to zero
    mHostParticlePos = new float[mParametersSimulation->particleCount * 4];
    mHostParticleVel = new float[mParametersSimulation->particleCount * 4];
    mNumberOfBytesAllocatedCpu += mParametersSimulation->particleCount * 8;

    memset(mHostParticlePos, 0, mParametersSimulation->particleCount * 4 * sizeof(float));
    memset(mHostParticleVel, 0, mParametersSimulation->particleCount * 4 * sizeof(float));

    // determine GPU data-size
    const unsigned int memSizeParticleQuadrupels = sizeof(float) * 4 * mParametersSimulation->particleCount;

    // Allocate GPU data
    // Create VBO with particle positions. This is later given to particle renderer for visualization
    mVboParticlePositions = OpenGlUtilities::createVbo(memSizeParticleQuadrupels);
    mNumberOfBytesAllocatedGpu += memSizeParticleQuadrupels;
    cudaSafeCall(cudaGraphicsGLRegisterBuffer(&mCudaVboResourceParticlePositions, mVboParticlePositions, cudaGraphicsMapFlagsNone));
    mVboParticleColors = OpenGlUtilities::createVbo(memSizeParticleQuadrupels);

    emit vboInfoParticles(
                mVboParticlePositions,
                mVboParticleColors,
                mParametersSimulation->particleCount,
                getVector(mParametersSimulation->gridParticleSystem.worldMin),
                getVector(mParametersSimulation->gridParticleSystem.worldMax)
                );

    // Create VBO with collider positions. This is later given to particle renderer for visualization
    mVboColliderPositions = mPointCloudColliders->getVboInfo()[0].vbo;//OpenGlUtilities::createVbo(sizeof(float) * 4 * mSimulationParameters->colliderCountMax);
    cudaSafeCall(cudaGraphicsGLRegisterBuffer(&mCudaVboResourceColliderPositions, mVboColliderPositions, cudaGraphicsMapFlagsNone));
    // use vboInfo.size or mSimulationParameters->colliderCountMax?

    cudaSafeCall(cudaMalloc((void**)&mDeviceParticleVel, memSizeParticleQuadrupels));
    mNumberOfBytesAllocatedGpu += memSizeParticleQuadrupels;

    cudaSafeCall(cudaMalloc((void**)&mDeviceColliderSortedPos, sizeof(float) * 4 * mPointCloudColliders->getCapacity()));
    mNumberOfBytesAllocatedGpu += sizeof(float) * 4 * mPointCloudColliders->getCapacity();

    // Here, we store the positions of each particle's last collision with the colliders
    cudaSafeCall(cudaMalloc((void**)&mDeviceParticleCollisionPositions, sizeof(float) * 4 * mParametersSimulation->particleCount));
    cudaSafeCall(cudaMemset(mDeviceParticleCollisionPositions, 0, sizeof(float) * 4 * mParametersSimulation->particleCount)); // set to (float)-zero
    mNumberOfBytesAllocatedGpu += sizeof(float) * 4 * mParametersSimulation->particleCount;

    // Sorted according to containing grid cell.
    cudaSafeCall(cudaMalloc((void**)&mDeviceParticleSortedPos, memSizeParticleQuadrupels));
    mNumberOfBytesAllocatedGpu += memSizeParticleQuadrupels;
    cudaSafeCall(cudaMalloc((void**)&mDeviceParticleSortedVel, memSizeParticleQuadrupels));
    mNumberOfBytesAllocatedGpu += memSizeParticleQuadrupels;

    // These two are used to map from gridcell (=hash) to particle id (=index). If we also know in which
    // indices of these arrays grid cells start and end, we can quickly find particles in neighboring cells...
    cudaSafeCall(cudaMalloc((void**)&mDeviceParticleMapGridCell, mParametersSimulation->particleCount*sizeof(uint)));
    cudaSafeCall(cudaMalloc((void**)&mDeviceParticleMapIndex, mParametersSimulation->particleCount*sizeof(uint)));
    mNumberOfBytesAllocatedGpu += mParametersSimulation->particleCount * sizeof(uint) * 2;

    // Same thing as above, just for colliders
    cudaSafeCall(cudaMalloc((void**)&mDeviceColliderMapGridCell, mPointCloudColliders->getCapacity()*sizeof(uint)));
    cudaSafeCall(cudaMalloc((void**)&mDeviceColliderMapIndex, mPointCloudColliders->getCapacity()*sizeof(uint)));
    mNumberOfBytesAllocatedGpu += mPointCloudColliders->getCapacity() * sizeof(uint) * 2;

    // ...and thats what we do here: in mDeviceCellStart[17], you'll find
    // where in mDeviceGridParticleHash cell 17 starts!
    cudaSafeCall(cudaMalloc((void**)&mDeviceParticleCellStart, numberOfCells*sizeof(uint)));
    cudaSafeCall(cudaMalloc((void**)&mDeviceParticleCellEnd, numberOfCells*sizeof(uint)));
    mNumberOfBytesAllocatedGpu += numberOfCells * sizeof(uint) * 2;

    // Same for colliders...
    cudaSafeCall(cudaMalloc((void**)&mDeviceColliderCellStart, numberOfCells*sizeof(uint)));
    cudaSafeCall(cudaMalloc((void**)&mDeviceColliderCellEnd, numberOfCells*sizeof(uint)));
    mNumberOfBytesAllocatedGpu += numberOfCells * sizeof(uint) * 2;

    cudaSafeCall(cudaGraphicsGLRegisterBuffer(&mCudaColorVboResource, mVboParticleColors, cudaGraphicsMapFlagsNone));

    // fill color buffer
    glBindBuffer(GL_ARRAY_BUFFER, mVboParticleColors);
    float *data = (float *) glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    for(unsigned int i=0; i<mParametersSimulation->particleCount; i++)
    {
        const float t = i / (float) mParametersSimulation->particleCount;

        colorRamp(t, data);
        data+=3;
        *data++ = 0.5f; // should be the alpha value
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    qDebug() << "ParticleSystem::initialize(): worldsize" << CudaHelper::cudaConvert(mParametersSimulation->gridParticleSystem.getWorldSize()) << "and particle radius" << mParametersSimulation->particleRadius << ": created system with" << mParametersSimulation->particleCount << "particles and" << mParametersSimulation->gridParticleSystem.cells.x << "*" << mParametersSimulation->gridParticleSystem.cells.y << "*" << mParametersSimulation->gridParticleSystem.cells.z << "cells";
    qDebug() << "ParticleSystem::initialize(): allocated" << mNumberOfBytesAllocatedCpu << "bytes on CPU," << mNumberOfBytesAllocatedGpu << "bytes on GPU.";

    copyParametersToGpu(mParametersSimulation);

    mIsInitialized = true;

    slotResetParticles();

    cudaSafeCall(cudaMemGetInfo(&memFree, &memTotal));
    qDebug() << "ParticleSystem::initialize(): after init, device has" << memFree / 1048576 << "of" << memTotal / 1048576 << "mb free.";
}

void ParticleSystem::freeResources()
{
    Q_ASSERT(mIsInitialized);

    qDebug() << "ParticleSystem::freeResources(): freeing allocated memory...";

    delete [] mHostParticlePos;
    delete [] mHostParticleVel;

    cudaSafeCall(cudaFree(mDeviceColliderSortedPos));
    cudaSafeCall(cudaFree(mDeviceParticleVel));
    cudaSafeCall(cudaFree(mDeviceParticleSortedPos));
    cudaSafeCall(cudaFree(mDeviceParticleSortedVel));

    cudaSafeCall(cudaFree(mDeviceParticleMapGridCell));
    cudaSafeCall(cudaFree(mDeviceParticleMapIndex));
    cudaSafeCall(cudaFree(mDeviceParticleCellStart));
    cudaSafeCall(cudaFree(mDeviceParticleCellEnd));

    cudaSafeCall(cudaFree(mDeviceColliderMapGridCell));
    cudaSafeCall(cudaFree(mDeviceColliderMapIndex));
    cudaSafeCall(cudaFree(mDeviceColliderCellStart));
    cudaSafeCall(cudaFree(mDeviceColliderCellEnd));

    cudaSafeCall(cudaFree(mDeviceParticleCollisionPositions));


    cudaSafeCall(cudaGraphicsUnregisterResource(mCudaVboResourceParticlePositions));
    OpenGlUtilities::deleteVbo(mVboParticlePositions);
    OpenGlUtilities::deleteVbo(mVboParticleColors);

    setNullPointers();

    mIsInitialized = false;
}

ParticleSystem::~ParticleSystem()
{
    if(mIsInitialized) freeResources();
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
void ParticleSystem::update(quint8 *deviceGridMapOfWayPointPressure)
{
    if(mParametersSimulation->gridParticleSystem.worldMax.x == 0.0f) return;

    if(!mIsInitialized) initialize();

//    cudaDeviceSynchronize();

    QTime startTime = QTime::currentTime();
    startTime.start();

    // Get a pointer to the particle positions in the device by mapping GPU mem into CPU mem
    float *deviceParticlePositions = (float*)CudaHelper::mapGLBufferObject(&mCudaVboResourceParticlePositions);

//    qDebug() << "ParticleSystem::update(): 0: getting pointer finished at" << startTime.elapsed();

    // Update constants
    copyParametersToGpu(mParametersSimulation);

//    qDebug() << "ParticleSystem::update(): 1: setting parameters finished at" << startTime.elapsed();

    // Get a pointer to the "__constant__ ParametersParticleSystem parametersParticleSystem" on the device
    ParametersParticleSystem* paramsParticleSystem;
    getDeviceAddressOfParametersParticleSystem(&paramsParticleSystem);

    // Integrate
    integrateSystem(
                deviceParticlePositions,                    // in/out: The particle positions, unsorted
                mDeviceParticleVel,                         // in/out: The particle velocities, unsorted
                deviceGridMapOfWayPointPressure,            // output: A grid mapping the 3d-space to waypoint pressure. Cells with high values (255) should be visited for information gain.
                mDeviceParticleCollisionPositions,          // input:  The particle's last collision position (or 0 if it didn't collide yet)
                paramsParticleSystem,                       // input:  The particle system's parameters
                mParametersSimulation->particleCount);

//    showInformationGain();

//    qDebug() << "ParticleSystem::update(): 2: integrating system finished at" << startTime.elapsed();

    // Now that particles have been moved, they might be contained in different grid cells. So recompute the
    // mapping gridCell => particleIndex. This will allow fast neighbor searching in the grid during collision phase.
    computeMappingFromPointToGridCell(
                mDeviceParticleMapGridCell,                 // output: The key - part of the particle gridcell->index map, unsorted
                mDeviceParticleMapIndex,                    // output: The value-part of the particle gridcell->index map, unsorted
                deviceParticlePositions,                    // input:  The particle positions after integration, unsorted and possibly colliding with other particles
                &paramsParticleSystem->gridParticleSystem,
                mParametersSimulation->particleCount);       // input:  The number of particles, one thread per particle

//    qDebug() << "ParticleSystem::update(): 3: computing particle spatial hash table finished at" << startTime.elapsed();

    // Sort the mapping gridCell => particleId on gridCell
    sortGridOccupancyMap(mDeviceParticleMapGridCell, mDeviceParticleMapIndex, mParametersSimulation->particleCount);

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
                mParametersSimulation->particleCount,        // input:  The number of particles
                mParametersSimulation->gridParticleSystem.getCellCount()  // input: Number of grid cells
                );

//    qDebug() << "ParticleSystem::update(): 5: computing particle navigation tables and sorting particles finished at" << startTime.elapsed();

    // Same for colliders
    if(mUpdateMappingFromColliderToGridCell)
    {
        float *deviceColliderPositions = (float*)CudaHelper::mapGLBufferObject(&mCudaVboResourceColliderPositions);

        computeMappingFromPointToGridCell(
                    mDeviceColliderMapGridCell,                 // output: The key - part of the collider gridcell->index map, unsorted
                    mDeviceColliderMapIndex,                    // output: The value-part of the collider gridcell->index map, unsorted
                    deviceColliderPositions,                    // input:  The collider positions (no integration), unsorted and possibly colliding with particles
                    &paramsParticleSystem->gridParticleSystem,
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
                    mParametersSimulation->gridParticleSystem.cells.x * mParametersSimulation->gridParticleSystem.cells.y * mParametersSimulation->gridParticleSystem.cells.z  // input: Number of grid cells
                    );

        cudaGraphicsUnmapResources(1, &mCudaVboResourceColliderPositions, 0);

        mUpdateMappingFromColliderToGridCell = false;
    }

//    qDebug() << "ParticleSystem::update(): 8: computing collider navigation tables and sorting particles finished at" << startTime.elapsed();

    // process collisions between particles
    collideParticlesWithParticlesAndColliders(
                mDeviceParticleVel,                         // output: The particle velocities
                deviceParticlePositions,                    // output: The w-component is changed whenever a particle has hit a collider. Used just for visualization.
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

                mParametersSimulation->particleCount,        // input:  How many particles to collide against other particles (one thread per particle)
                mParametersSimulation->gridParticleSystem.cells.x * mParametersSimulation->gridParticleSystem.cells.y * mParametersSimulation->gridParticleSystem.cells.z  // input: Number of grid cells
                );

//    showCollisionPositions();

//    qDebug() << "ParticleSystem::update(): 9: colliding particles finished at" << startTime.elapsed();

    // Unmap at end here to avoid unnecessary graphics/CUDA context switch.
    // Once unmapped, the resource may not be accessed by CUDA until they
    // are mapped again. This function provides the synchronization guarantee
    // that any CUDA work issued  before ::cudaGraphicsUnmapResources()
    // will complete before any subsequently issued graphics work begins.
    cudaGraphicsUnmapResources(1, &mCudaVboResourceParticlePositions, 0);

//    cudaDeviceSynchronize();

    size_t memTotal, memFree;
    cudaMemGetInfo(&memFree, &memTotal);
    //qDebug() << "ParticleSystem::update(): finished," << startTime.elapsed() << "ms, fps:" << 1000.0f/startTime.elapsed() << "free mem:" << memFree / 1048576;
}

void ParticleSystem::slotSetParticleRadius(float radius)
{
    mParametersSimulation->particleRadius = radius;
    qDebug() << "ParticleSystem::slotSetParticleRadius(): setting particle radius to" << mParametersSimulation->particleRadius;

    // The particle-radius must be at least half the cell-size. Otherwise, we might have more than 4 particles in a cell,
    // which might break collisions. I'm not sure this really is a problem, though, need to investigate further.
    // If the particles are now so small that more than 4 can fit into a cell, re-create the grid with more cells

    QVector3D cellSize = CudaHelper::cudaConvert(mParametersSimulation->gridParticleSystem.getCellSize());
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
    if(!mIsInitialized) return;

    // When re-setting particles, also reset their position of last collision!
    cudaSafeCall(cudaMemset(mDeviceParticleCollisionPositions, 0, mParametersSimulation->particleCount * 4 * sizeof(float)));

    switch(mDefaultParticlePlacement)
    {
    case ParticlePlacement::PlacementRandom:
    {
        int p = 0, v = 0;

        qDebug() << "ParticleSystem::reset(): world min" << mParametersSimulation->gridParticleSystem.worldMin.x << mParametersSimulation->gridParticleSystem.worldMin.y << mParametersSimulation->gridParticleSystem.worldMin.z <<
        "max" << mParametersSimulation->gridParticleSystem.worldMax.x << mParametersSimulation->gridParticleSystem.worldMax.y << mParametersSimulation->gridParticleSystem.worldMax.z;

        for(unsigned int i=0; i < mParametersSimulation->particleCount; i++)
        {
            mHostParticlePos[p++] = mParametersSimulation->gridParticleSystem.worldMin.x + (mParametersSimulation->gridParticleSystem.worldMax.x - mParametersSimulation->gridParticleSystem.worldMin.x) * frand();
            mHostParticlePos[p++] = mParametersSimulation->gridParticleSystem.worldMin.y + (mParametersSimulation->gridParticleSystem.worldMax.y - mParametersSimulation->gridParticleSystem.worldMin.y) * frand();
            mHostParticlePos[p++] = mParametersSimulation->gridParticleSystem.worldMin.z + (mParametersSimulation->gridParticleSystem.worldMax.z - mParametersSimulation->gridParticleSystem.worldMin.z) * frand();
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
        const float jitter = mParametersSimulation->particleRadius * 0.01f;
        const float spacing = mParametersSimulation->particleRadius * 2.0f;

        // If we want a cube, determine the number of particles in each dimension
        unsigned int s = (int) ceilf(powf((float) mParametersSimulation->particleCount, 1.0f / 3.0f));
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
                    if (i < mParametersSimulation->particleCount)
                    {
                        mHostParticlePos[i*4+0] = (spacing * x) + mParametersSimulation->particleRadius - 1.0f + (frand()*2.0f-1.0f)*jitter;
                        mHostParticlePos[i*4+1] = (spacing * y) + mParametersSimulation->particleRadius - 1.0f + (frand()*2.0f-1.0f)*jitter;
                        mHostParticlePos[i*4+2] = (spacing * z) + mParametersSimulation->particleRadius - 1.0f + (frand()*2.0f-1.0f)*jitter;
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
        float jitter = mParametersSimulation->particleRadius * 0.1f;
        const float spacing = mParametersSimulation->particleRadius * 2.02f;

        unsigned int particleNumber = 0;

        for(
            float y = mParametersSimulation->gridParticleSystem.worldMax.y - mParametersSimulation->particleRadius;
            y >= mParametersSimulation->gridParticleSystem.worldMin.y + mParametersSimulation->particleRadius && particleNumber < mParametersSimulation->particleCount;
            y -= spacing)
        {

            for(
                float x = mParametersSimulation->gridParticleSystem.worldMin.x + mParametersSimulation->particleRadius;
                x <= mParametersSimulation->gridParticleSystem.worldMax.x - mParametersSimulation->particleRadius && particleNumber < mParametersSimulation->particleCount;
                x += spacing)
            {
                for(
                    float z = mParametersSimulation->gridParticleSystem.worldMin.z + mParametersSimulation->particleRadius;
                    z <= mParametersSimulation->gridParticleSystem.worldMax.z - mParametersSimulation->particleRadius && particleNumber < mParametersSimulation->particleCount;
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

    setArray(ArrayPositions, mHostParticlePos, 0, mParametersSimulation->particleCount);
    setArray(ArrayVelocities, mHostParticleVel, 0, mParametersSimulation->particleCount);
}
