// Fix for gcc 4.7
#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#include "thrust/device_ptr.h"
#include "thrust/device_vector.h"
#include "thrust/for_each.h"
#include "thrust/iterator/zip_iterator.h"
#include "thrust/sort.h"

#include "particleskernel.cu"
#include "cuda.h"

#include <QDebug>

void copyParametersToGpu(SimulationParameters *hostParams)
{
    // Copy parameters to constant memory. This was synchronous once, I changed
    // it to be asynchronous. Shouldn't cause any harm, even if parameters were
    // applied one frame too late.
    cudaMemcpyToSymbol/*Async*/(params, hostParams, sizeof(SimulationParameters));
}

void integrateSystem(float *particlePositions, float *particleVelocities, uint8_t* gridWaypointPressure, float* particleCollisionPositions, float deltaTime, uint numParticles)
{
// old thrust version. Cannot write to the non-linear waypointpressure position when using thrust tuples.
//    thrust::device_ptr<float4> d_pos4((float4*)pos);
//    thrust::device_ptr<float4> d_vel4((float4*)vel);
//    thrust::device_ptr<float4> d_pcp4((float4*)particleCollisionPositions);
//    thrust::device_ptr<uint8_t> d_gwpp((uint8_t*)gridWaypointPressure);

//    thrust::for_each(
//                thrust::make_zip_iterator(thrust::make_tuple(d_pos4, d_vel4, d_pcp4, d_gwpp)),
//                thrust::make_zip_iterator(thrust::make_tuple(d_pos4 + numParticles, d_vel4 + numParticles, d_pcp4 + numParticles, d_gwpp + numParticles)),
//                integrate_functor(deltaTime));

    if(numParticles == 0) return;

    uint numThreads, numBlocks;
    computeExecutionKernelGrid(numParticles, 256, numBlocks, numThreads);

    // execute the kernel
    integrateSystemD<<< numBlocks, numThreads >>>(
                                                    (float4*)particlePositions,          // in/out: particle positions
                                                    (float4*)particleVelocities,         // in/out: particle velocities
                                                    gridWaypointPressure,       // in/out: grid containing quint8-cells with waypoint-pressure values (80-255)
                                                    (float4*)particleCollisionPositions, // input:  particle positions
                                                    deltaTime,
                                                    numParticles);

    // check if kernel invocation generated an error
    cudaCheckSuccess("integrateSystem");
}

// Calculates a hash for each particle. The hash value is ("based on") its cell id.
void computeMappingFromGridCellToParticle(
        uint*  gridParticleHash,
        uint*  gridParticleIndex,
        float* pos,
        int    numParticles)
{
    if(numParticles == 0) return;

    uint numThreads, numBlocks;
    computeExecutionKernelGrid(numParticles, 256, numBlocks, numThreads);

    // execute the kernel
    computeMappingFromGridCellToParticleD<<< numBlocks, numThreads >>>(gridParticleHash,
                                           gridParticleIndex,
                                           (float4 *) pos,
                                           numParticles);

    // check if kernel invocation generated an error
    cudaCheckSuccess("computeMappingFromGridCellToParticleD");
}

void sortParticlePosAndVelAccordingToGridCellAndFillCellStartAndEndArrays(
        uint*  cellStart,
        uint*  cellEnd,
        float* sortedPos,
        float* sortedVel,
        uint*  gridParticleHash,
        uint*  gridParticleIndex,
        float* oldPos,
        float* oldVel,
        uint   numParticles,
        uint   numCells)
{
    // set all cells to empty
    cudaMemset(cellStart, 0xffffffff, numCells*sizeof(uint));

    if(numParticles == 0) return;

    uint numThreads, numBlocks;
    computeExecutionKernelGrid(numParticles, 256, numBlocks, numThreads);


#if USE_TEX
    cudaBindTexture(0, oldPosTex, oldPos, numParticles*sizeof(float4));
    if(oldVel && sortedVel) cudaBindTexture(0, oldVelTex, oldVel, numParticles*sizeof(float4));
#endif

    // Number of bytes in shared memory that is allocated for each (thread)block.
    uint smemSize = sizeof(uint)*(numThreads+1);

    sortPosAndVelAccordingToGridCellAndFillCellStartAndEndArraysD<<< numBlocks, numThreads, smemSize>>>(
                                                                         cellStart,
                                                                         cellEnd,
                                                                         (float4 *) sortedPos,
                                                                         (float4 *) sortedVel,
                                                                         gridParticleHash,
                                                                         gridParticleIndex,
                                                                         (float4 *) oldPos,
                                                                         (float4 *) oldVel,
                                                                         numParticles);

    cudaCheckSuccess("sortPosAndVelAccordingToGridCellAndFillCellStartAndEndArraysD");

#if USE_TEX
    cudaUnbindTexture(oldPosTex);
    if(oldVel && sortedVel) cudaUnbindTexture(oldVelTex);
#endif
}

void collideParticlesWithParticlesAndColliders(
        float* newVel,              // output: The particle velocities
        float* particleCollisionPositions,          // output: Every particle's position of last collision, or 0.0/0.0/0.0 if none occurred.

        float* particlePosSorted,   // input:  The particle positions, sorted by gridcell
        float* particleVelSorted,   // input:  The particle velocities, sorted by gridcell
        uint*  particleMapIndex,    // input:  The value-part of the particle gridcell->index map, sorted by gridcell
        uint*  particleCellStart,   // input:  At which index in mDeviceMapParticleIndex does cell X start?
        uint*  particleCellEnd,     // input:  At which index in mDeviceMapParticleIndex does cell X end?

        float* colliderSortedPos,   // input:  The collider positions, sorted by gridcell
        uint*  colliderMapIndex,    // input:  The value-part of the collider gridcell->index map, sorted by gridcell
        uint*  colliderCellStart,   // input:  At which index in mDeviceMapColliderIndex does cell X start?
        uint*  colliderCellEnd,     // input:  At which index in mDeviceMapColliderIndex does cell X end?

        uint   numParticles,        // input:  How many particles to collide against other particles (one thread per particle)
        uint   numCells             // input:  Number of grid cells
        )
{
#if USE_TEX
    cudaBindTexture(0, oldPosTex, sortedPos, numParticles*sizeof(float4));
    cudaBindTexture(0, oldVelTex, sortedVel, numParticles*sizeof(float4));
    cudaBindTexture(0, cellStartTex, cellStart, numCells*sizeof(uint));
    cudaBindTexture(0, cellEndTex, cellEnd, numCells*sizeof(uint));
#endif

    // thread per particle
    uint numThreads, numBlocks;
    computeExecutionKernelGrid(numParticles, 64, numBlocks, numThreads);

    // execute the kernel
    collideParticlesWithParticlesAndCollidersD<<< numBlocks, numThreads >>>(
                                                                              (float4*)newVel,
                                                                              (float4*)particleCollisionPositions,

                                                                              (float4*)particlePosSorted,
                                                                              (float4*)particleVelSorted,
                                                                              particleMapIndex,
                                                                              particleCellStart,
                                                                              particleCellEnd,

                                                                              (float4*)colliderSortedPos,
                                                                              colliderMapIndex,
                                                                              colliderCellStart,
                                                                              colliderCellEnd,

                                                                              numParticles);

    // check if kernel invocation generated an error
    cudaCheckSuccess("collideParticlesWithParticlesD");

#if USE_TEX
    cudaUnbindTexture(oldPosTex);
    cudaUnbindTexture(oldVelTex);
    cudaUnbindTexture(cellStartTex);
    cudaUnbindTexture(cellEndTex);
#endif
}

void sortGridOccupancyMap(uint *dGridParticleHash, uint *dGridParticleIndex, uint numParticles)
{
    if(numParticles > 0)
        thrust::sort_by_key(thrust::device_ptr<uint>(dGridParticleHash),                // KeysBeginning
                            thrust::device_ptr<uint>(dGridParticleHash + numParticles), // KeysEnd
                            thrust::device_ptr<uint>(dGridParticleIndex));              // ValuesBeginning

    // check if kernel invocation generated an error
    cudaCheckSuccess("sortGridOccupancyMap");
}


// Fill mDeviceGridMapCellWorldPositions - this might be done only once and then copied lateron (just like the waypoint pressure above)
void fillGridMapCellWorldPositions(float* gridMapCellWorldPositions, uint numCells)
{
    // thread per cell
    uint numThreads, numBlocks;
    computeExecutionKernelGrid(numCells, 64, numBlocks, numThreads);

    fillGridMapCellWorldPositionsD<<< numBlocks, numThreads >>>(
                                                                  (float4*)gridMapCellWorldPositions,
                                                                  numCells);
}

void moveGridMapWayPointPressureValuesByWorldPositionOffset(quint8 *gridMapOfWayPointPressure, float* offset, unsigned int numberOfCells)
{
    // thread per cell
    uint numThreads, numBlocks;
    computeExecutionKernelGrid(1/*numberOfCells*/, 64, numBlocks, numThreads);

    qDebug() << "kernel exec:" << numberOfCells << numBlocks << numThreads << offset[0] << offset[1] << offset[2];

    moveGridMapWayPointPressureValuesByWorldPositionOffsetD<<< numBlocks, numThreads >>>(
                                                                                           (uint8_t*)gridMapOfWayPointPressure,
                                                                                           (float3*)offset,
                                                                                           numberOfCells);
    qDebug() << "kernel exec done, checking...";

    cudaCheckSuccess("moveGridMapWayPointPressureValuesByWorldPositionOffset");
}

// Sort mDeviceGridMapWayPointPressureSorted => mDeviceGridMapCellWorldPositions according to the keys DESC
void sortGridMapWayPointPressure(uint8_t* gridMapWayPointPressureSorted, float* gridMapCellWorldPositions, uint numCells, uint numWaypointsRequested)
{
    if(numCells > 0)
    {
        thrust::sort_by_key(thrust::device_ptr<uint8_t>(gridMapWayPointPressureSorted),             // KeysBeginning
                            thrust::device_ptr<uint8_t>(gridMapWayPointPressureSorted + numCells),  // KeysEnd
                            thrust::device_ptr<float4>((float4*)gridMapCellWorldPositions),         // ValuesBeginning
                            thrust::greater<int>());                                                // In descending order

        // Now we want to copy the waypointpressure-value for all requested waypoints from gridMapWayPointPressureSorted(quint8) to gridMapCellWorldPositions.w(float)
        thrust::device_ptr<float4>  d_cwp((float4*)gridMapCellWorldPositions);
        thrust::device_ptr<uint8_t> d_wpp((uint8_t*)gridMapWayPointPressureSorted);

        thrust::for_each(
                    thrust::make_zip_iterator(thrust::make_tuple(d_wpp, d_cwp)),
                    thrust::make_zip_iterator(thrust::make_tuple(d_wpp + numWaypointsRequested, d_cwp + numWaypointsRequested)),
                    copy_functor());
    }

    // check if kernel invocation generated an error
    cudaCheckSuccess("sortGridMapWayPointPressure");
}
