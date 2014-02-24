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
    mPointCloudColliders->setOutlierTreatment(PointCloud::OutlierTreatment::Remove);

    // set simulation parameters
    mParametersSimulation = simulationParameters;

    setNullPointers();

    mIsInitialized = false;
    mNumberOfBytesAllocatedCpu = 0;
    mNumberOfBytesAllocatedGpu = 0;

    // We set this to true, so that even with no colliders present, the cellIndexStart-
    // vector will be initialized with 0x777777 once on startup
    mUpdateMappingFromColliderToGridCell = true;

    mParametersSimulation->gridParticleSystem.worldMin = make_float3(-32.0f, -6.0f, -32.0f);
    mParametersSimulation->gridParticleSystem.worldMax = make_float3(32.0f, 26.0f, 32.0f);

    connect(mPointCloudColliders, SIGNAL(pointsInserted(PointCloudCuda*const,quint32,quint32)), SLOT(slotNewCollidersInserted()));

    slotSetDefaultParticlePlacement(ParticleSystem::ParticlePlacement::PlacementFillSky);
}

void ParticleSystem::slotNewCollidersInserted()
{
    mUpdateMappingFromColliderToGridCell = true;
}

void ParticleSystem::setNullPointers()
{
    mHostParticlePos = nullptr;
    mHostParticleVel = nullptr;
    mDeviceParticleVel = nullptr;
    mDeviceParticleSortedPos = nullptr;
    mDeviceParticleSortedVel = nullptr;
    mDeviceParticleMapGridCell = nullptr;
    mDeviceParticleMapIndex = nullptr;
    mDeviceColliderMapGridCell = nullptr;
    mDeviceColliderMapIndex = nullptr;
    mDeviceColliderCellStart = nullptr;
    mDeviceParticleCellEnd = nullptr;
    mDeviceColliderCellStart = nullptr;
    mDeviceColliderCellEnd = nullptr;
    mDeviceParticleCollisionPositions = nullptr;
    mDeviceColliderSortedPos = nullptr;
    mVboParticlePositions = 0;
    mVboColliderPositions = 0;
}

void ParticleSystem::slotSetVolume(const Box3D& boundingBox)
{
    // Set the new volume
    mParametersSimulation->gridParticleSystem.worldMin = CudaHelper::convert(boundingBox.min);
    mParametersSimulation->gridParticleSystem.worldMax = CudaHelper::convert(boundingBox.max);

    if(mIsInitialized) freeResources();

    // We moved the particle system. This means that A) the system's sparse collider-pointcloud now
    // contains points outside of the particle system and B) the dense pointcloud contains points
    // that are NOT in the collider-pointcloud - but should be. So, we re-populate the sparse point-
    // cloud with points from the dense pointcloud.
    if(mPointCloudDense->getNumberOfPointsStored())
        mPointCloudColliders->slotInsertPoints(
                    mPointCloudDense,
                    0,
                    std::min(mPointCloudDense->getNumberOfPointsStored(), mPointCloudDense->getCapacity()));

    // Usually, the mapping from collider to grid-cells only changes when the collider pointcloud changes.
    // But when we move the grid (relative to the colliders), the mapping also needs an update!
    mUpdateMappingFromColliderToGridCell = true;

    //slotInitialize(); will happen lazily on update
}

void ParticleSystem::slotSetSimulationParameters(const ParametersParticleSystem* const sp)
{
    // We just accept these parameters
    mParametersSimulation->timeStepInner = sp->timeStepInner;
    mParametersSimulation->attraction = sp->attraction;
    mParametersSimulation->dampingMotion = sp->dampingMotion;
    mParametersSimulation->colliderRadius = sp->colliderRadius;
    mParametersSimulation->gravity = sp->gravity;
    mParametersSimulation->shear = sp->shear;
    mParametersSimulation->spring = sp->spring;
    mParametersSimulation->velocityFactorCollisionBoundary = sp->velocityFactorCollisionBoundary;
    mParametersSimulation->velocityFactorCollisionParticle = sp->velocityFactorCollisionParticle;
    mParametersSimulation->velocityFactorCollisionCollider = sp->velocityFactorCollisionCollider;

    slotSetParticleRadius(sp->particleRadius); // this will emit vboInfoAndParameters, so set it last!
}

void ParticleSystem::slotSetParticleRadius(float radius)
{
    qDebug() << "ParticleSystem::slotSetParticleRadius(): setting particle radius from" << mParametersSimulation->particleRadius << "to" << radius;
    mParametersSimulation->particleRadius = radius;

    // Need to rebuild data-structures when particle radius or -count changes.
    if(mIsInitialized) freeResources();

    //slotInitialize(); will happen lazily on update
}

void ParticleSystem::slotEmitVboInfoAndParameters()
{
    qDebug() << __PRETTY_FUNCTION__ << "emitting VBO info.";
    emit vboInfoParticles(
                mVboParticlePositions,
                mParametersSimulation->particleCount,
                mParametersSimulation->particleRadius,
                Box3D(
                    CudaHelper::convert(mParametersSimulation->gridParticleSystem.worldMin),
                    CudaHelper::convert(mParametersSimulation->gridParticleSystem.worldMax)
                    )
                );

    qDebug() << __PRETTY_FUNCTION__ << "telling UI about changed parameters...";
    emit parametersChanged(mParametersSimulation);
    qDebug() << __PRETTY_FUNCTION__ << "done.";
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
                        mParametersSimulation->gridParticleSystem.getCellHash(mParametersSimulation->gridParticleSystem.getCellCoordinate(CudaHelper::convert(collisionPositionsHost[i].toVector3D()))) << "at" << collisionPositionsHost[i];
        }
    }

    delete collisionPositionsHost;
}

void ParticleSystem::slotInitialize()
{
    qDebug() << __PRETTY_FUNCTION__;
    if(mIsInitialized)
    {
        qDebug() << __PRETTY_FUNCTION__ << "already initialized, returning.";
        return;
    }

    OpenGlUtilities::checkError();

    // This needs to be called only once, but here it might get called more often. Shouldn't be a problem.
    initializeOpenGLFunctions();

    size_t memTotal, memFree;
    cudaSafeCall(cudaMemGetInfo(&memFree, &memTotal));
    qDebug() << __PRETTY_FUNCTION__ << "before init, device has" << memFree / 1048576 << "of" << memTotal / 1048576 << "mb free.";

    mUpdateMappingFromColliderToGridCell = true;

    // Change particleCount, so that it makes sense to use with particleRadius: we want the top third to be populated with particles.
    float volumeTotal = mParametersSimulation->gridParticleSystem.getWorldVolume();
    // Abstract particles to boxes, as that's how they'll be arranged
    float volumeParticle = pow(mParametersSimulation->particleRadius*2, 3);
    // Use as many particles as needed, but never more than, say, 32k. 64k might also work, but we don't need the density for a 32m^3 scanVolume.
    mParametersSimulation->particleCount = (volumeTotal / volumeParticle) / 3;
    if(mParametersSimulation->particleCount > 32768) mParametersSimulation->particleCount = 32768;

    // Set gridsize so that the cell-edges are never shorter than the particle's diameter! If particles were allowed to be larger than gridcells in
    // any dimension, we couldn't find all relevant neighbors by searching through only the (3*3*3)-1 = 26 cells immediately neighboring this one.
    // We can also modify this compromise by allowing larger particles and then searching (5*5*5)-1 = 124 cells. Haven't tried.

    // Using less (larger) cells is possible, it means less memory being used for the grid, but more search being done when searching for neighbors
    // because more particles now occupy a single grid cell. This might not hurt performance as long as we do not cross an unknown threshold (kernel swap size)?!
    const QVector3D particleSystemWorldSize = CudaHelper::convert(mParametersSimulation->gridParticleSystem.getWorldSize());
    mParametersSimulation->gridParticleSystem.cells.x = nextHigherPowerOfTwo(particleSystemWorldSize.x() / mParametersSimulation->particleRadius) / 2;
    mParametersSimulation->gridParticleSystem.cells.x = qBound(2, (int)mParametersSimulation->gridParticleSystem.cells.x, 128);

    mParametersSimulation->gridParticleSystem.cells.y = nextHigherPowerOfTwo(particleSystemWorldSize.y() / mParametersSimulation->particleRadius) / 2;
    mParametersSimulation->gridParticleSystem.cells.y = qBound(2, (int)mParametersSimulation->gridParticleSystem.cells.y, 128);

    mParametersSimulation->gridParticleSystem.cells.z = nextHigherPowerOfTwo(particleSystemWorldSize.z() / mParametersSimulation->particleRadius) / 2;
    mParametersSimulation->gridParticleSystem.cells.z = qBound(2, (int)mParametersSimulation->gridParticleSystem.cells.z, 128);

    const quint32 numberOfCells = mParametersSimulation->gridParticleSystem.getCellCount();
    // allocate host storage for particle positions and velocities, then set them to zero
    mHostParticlePos = new float[mParametersSimulation->particleCount * 4];
    mHostParticleVel = new float[mParametersSimulation->particleCount * 4];
    mNumberOfBytesAllocatedCpu += mParametersSimulation->particleCount * 8;

    memset(mHostParticlePos, 0, mParametersSimulation->particleCount * 4 * sizeof(float));
    memset(mHostParticleVel, 0, mParametersSimulation->particleCount * 4 * sizeof(float));

    // determine GPU data-size
    const quint32 memSizeParticleQuadrupels = sizeof(float) * 4 * mParametersSimulation->particleCount;

    // Allocate GPU data
    // Create VBO with particle positions. This is later given to particle renderer for visualization
    mVboParticlePositions = OpenGlUtilities::createVbo(memSizeParticleQuadrupels);
    mNumberOfBytesAllocatedGpu += memSizeParticleQuadrupels;
    cudaSafeCall(cudaGraphicsGLRegisterBuffer(&mCudaVboResourceParticlePositions, mVboParticlePositions, cudaGraphicsMapFlagsNone));

    // Create VBO with collider positions. This is later given to particle renderer for visualization
    mVboColliderPositions = mPointCloudColliders->getRenderInfo()->at(0)->vbo;//OpenGlUtilities::createVbo(sizeof(float) * 4 * mSimulationParameters->colliderCountMax);
    //qDebug() << "vbo colliderpos is" << mVboColliderPositions;
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

    qDebug() << __PRETTY_FUNCTION__ << "worldsize" << CudaHelper::convert(mParametersSimulation->gridParticleSystem.getWorldSize()) << "and particle radius" << mParametersSimulation->particleRadius << ": created system with" << mParametersSimulation->particleCount << "particles and" << mParametersSimulation->gridParticleSystem.cells.x << "*" << mParametersSimulation->gridParticleSystem.cells.y << "*" << mParametersSimulation->gridParticleSystem.cells.z << "cells";
    qDebug() << __PRETTY_FUNCTION__ << "allocated" << mNumberOfBytesAllocatedCpu << "bytes on CPU," << mNumberOfBytesAllocatedGpu << "bytes on GPU.";

    copyParametersToGpu(mParametersSimulation);

    mIsInitialized = true;

    slotResetParticles();

    slotEmitVboInfoAndParameters();

    cudaSafeCall(cudaMemGetInfo(&memFree, &memTotal));
    qDebug() << __PRETTY_FUNCTION__ << "after init, device has" << memFree / 1048576 << "of" << memTotal / 1048576 << "mb free.";
}

void ParticleSystem::freeResources()
{
    Q_ASSERT(mIsInitialized);

    qDebug() << __PRETTY_FUNCTION__ << "freeing allocated memory...";

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

    setNullPointers();

    // mVboParticlePositions should now be zero, emit it and glScene won't render it
    slotEmitVboInfoAndParameters();

    mIsInitialized = false;
    qDebug() << __PRETTY_FUNCTION__ << "done.";
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
    //qDebug() << __PRETTY_FUNCTION__;

    //if(mParametersSimulation->gridParticleSystem.worldMax.x == 0.0f) return; // why this?

    if(!mIsInitialized) slotInitialize();

    QTime startTime = QTime::currentTime();
    startTime.start();

    // Get a pointer to the particle positions in the device by mapping GPU mem into CPU mem
    float *deviceParticlePositions = (float*)CudaHelper::mapGLBufferObject(&mCudaVboResourceParticlePositions);

    //qDebug() << __PRETTY_FUNCTION__ << "0: getting pointer finished at" << startTime.elapsed();

    // Update constants
    copyParametersToGpu(mParametersSimulation);

    //qDebug() << __PRETTY_FUNCTION__ << "1: setting parameters finished at" << startTime.elapsed();

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

    //qDebug() << __PRETTY_FUNCTION__ << "2: integrating system finished at" << startTime.elapsed();

    // Now that particles have been moved, they might be contained in different grid cells. So recompute the
    // mapping gridCell => particleIndex. This will allow fast neighbor searching in the grid during collision phase.
    computeMappingFromPointToGridCell(
                mDeviceParticleMapGridCell,                 // output: The key - part of the particle gridcell->index map, unsorted
                mDeviceParticleMapIndex,                    // output: The value-part of the particle gridcell->index map, unsorted
                deviceParticlePositions,                    // input:  The particle positions after integration, unsorted and possibly colliding with other particles
                &paramsParticleSystem->gridParticleSystem,
                mParametersSimulation->particleCount);       // input:  The number of particles, one thread per particle

    //qDebug() << __PRETTY_FUNCTION__ << "3: computing particle spatial hash table finished at" << startTime.elapsed();

    // Sort the mapping gridCell => particleId on gridCell
    sortGridOccupancyMap(mDeviceParticleMapGridCell, mDeviceParticleMapIndex, mParametersSimulation->particleCount);

    //qDebug() << __PRETTY_FUNCTION__ << "4: sorting particle spatial hash table finished at" << startTime.elapsed();

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

    //qDebug() << __PRETTY_FUNCTION__ << "5: computing particle navigation tables and sorting particles finished at" << startTime.elapsed();

    // Same for colliders
    if(mUpdateMappingFromColliderToGridCell)
    {
        float *deviceColliderPositions = (float*)CudaHelper::mapGLBufferObject(&mCudaVboResourceColliderPositions);

        computeMappingFromPointToGridCell(
                    mDeviceColliderMapGridCell,                 // output: The key - part of the collider gridcell->index map, unsorted
                    mDeviceColliderMapIndex,                    // output: The value-part of the collider gridcell->index map, unsorted
                    deviceColliderPositions,                    // input:  The collider positions (no integration), unsorted and possibly colliding with particles
                    &paramsParticleSystem->gridParticleSystem,
                    mPointCloudColliders->getRenderInfo()->at(0)->size  // input:  The number of colliders, one thread per particle
                );

        //qDebug() << __PRETTY_FUNCTION__ << "6: computing collider spatial hash table finished at" << startTime.elapsed();

        sortGridOccupancyMap(mDeviceColliderMapGridCell, mDeviceColliderMapIndex, mPointCloudColliders->getRenderInfo()->at(0)->size);

        //qDebug() << __PRETTY_FUNCTION__ << "7: sorting collider spatial hash table finished at" << startTime.elapsed();

        sortParticlePosAndVelAccordingToGridCellAndFillCellStartAndEndArrays(
                    mDeviceColliderCellStart,                   // output: At which index in mDeviceMapColliderIndex does cell X start?
                    mDeviceColliderCellEnd,                     // output: At which index in mDeviceMapColliderIndex does cell X end?
                    mDeviceColliderSortedPos,                   // output: The collider positions, sorted by gridcell
                    0,                                          // output: The collider velocities, sorted by gridcell / they have no vel, so pass 0
                    mDeviceColliderMapGridCell,                 // input:  The key - part of the collider gridcell->index map, unsorted
                    mDeviceColliderMapIndex,                    // input:  The value-part of the collider gridcell->index map, unsorted
                    deviceColliderPositions,                    // input:  The particle-positions, unsorted
                    0,                                          // input:  The particle-velocities, unsorted / they have no vel, so pass 0
                    mPointCloudColliders->getRenderInfo()->at(0)->size, // input:  The number of colliders
                mParametersSimulation->gridParticleSystem.cells.x * mParametersSimulation->gridParticleSystem.cells.y * mParametersSimulation->gridParticleSystem.cells.z  // input: Number of grid cells
                );

        cudaGraphicsUnmapResources(1, &mCudaVboResourceColliderPositions, 0);

        mUpdateMappingFromColliderToGridCell = false;
    }

    //qDebug() << __PRETTY_FUNCTION__ << "8: computing collider navigation tables and sorting particles finished at" << startTime.elapsed();

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

    //showCollisionPositions();

    //qDebug() << __PRETTY_FUNCTION__ << "9: colliding particles finished at" << startTime.elapsed();

    // Unmap at end here to avoid unnecessary graphics/CUDA context switch.
    // Once unmapped, the resource may not be accessed by CUDA until they
    // are mapped again. This function provides the synchronization guarantee
    // that any CUDA work issued  before ::cudaGraphicsUnmapResources()
    // will complete before any subsequently issued graphics work begins.
    cudaGraphicsUnmapResources(1, &mCudaVboResourceParticlePositions, 0);

    size_t memTotal, memFree;
    cudaMemGetInfo(&memFree, &memTotal);
    //qDebug() << __PRETTY_FUNCTION__ << "finished," << startTime.elapsed() << "ms, fps:" << 1000.0f/startTime.elapsed() << "free mem:" << memFree / 1048576;
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
        cudaSafeCall(cudaMemcpy(
                    (char*) mDeviceParticleVel + start*4*sizeof(float), // destination
                    data,                                       // source
                    count*4*sizeof(float),                      // count
                    cudaMemcpyHostToDevice                      // copy-kind
                    ));
    }
}

inline float frand()
{
    return rand() / (float)RAND_MAX;
}

void ParticleSystem::slotResetParticles()
{
    qDebug() << __PRETTY_FUNCTION__;

    if(!mIsInitialized) slotInitialize();

    // When re-setting particles, also reset their position of last collision!
    cudaSafeCall(cudaMemset(mDeviceParticleCollisionPositions, 0, mParametersSimulation->particleCount * 4 * sizeof(float)));

    switch(mDefaultParticlePlacement)
    {
    case ParticlePlacement::PlacementRandom:
    {
        int p = 0, v = 0;

        qDebug() << __PRETTY_FUNCTION__ << "world min" << mParametersSimulation->gridParticleSystem.worldMin.x << mParametersSimulation->gridParticleSystem.worldMin.y << mParametersSimulation->gridParticleSystem.worldMin.z <<
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
