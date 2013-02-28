// Fix for gcc 4.7
#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#include "grid.cuh"
//#include "simulationparameters.cuh"
#include "cudahelper.h"
#include "cudahelper.cuh"
#include "helper_math.h"

#ifdef __CUDACC__

__device__ float3 Grid::getWorldSize() const
{
    return make_float3(
                worldMax.x - worldMin.x,
                worldMax.y - worldMin.y,
                worldMax.z - worldMin.z);
}

__device__ float3 Grid::getWorldCenter() const
{
    return make_float3(worldMin.x, worldMin.y, worldMin.z) + getWorldSize()/2.0f;
}

// Calculate a particle's hash value (=address in grid) from its containing cell (clamping to edges)
__device__ unsigned int Grid::getCellHash(int3 gridCellCoordinate) const
{
    gridCellCoordinate.x = gridCellCoordinate.x & (cells.x-1);  // wrap grid, assumes size is power of 2
    gridCellCoordinate.y = gridCellCoordinate.y & (cells.y-1);
    gridCellCoordinate.z = gridCellCoordinate.z & (cells.z-1);

    return (gridCellCoordinate.z * cells.y) * cells.x
            + (gridCellCoordinate.y * cells.x)
            + gridCellCoordinate.x;
}

// Given the cell hash (=gl_PrimitiveIDIn), whats the 3d-grid-coordinate of the cell's center?
// This is the reverse of particleskernel.cu -> calcGridHash(int3 gridCell).
__host__ __device__ int3 Grid::getCellCoordinate(const unsigned int hash) const
{
    int3 cell;
    cell.x = floor(fmod((double)hash, cells.x));
    cell.y = floor(fmod((double)hash, cells.x * cells.y) / cells.x);
    cell.z = floor(fmod((double)hash, cells.x * cells.y * cells.z) / (cells.x * cells.y));
    return cell;
}

__device__ int3 Grid::getCellCoordinate(const float3 &worldPos) const
{
    const float3 posRelativeToGridMin = worldPos - worldMin;
    const float3 cellSize = getCellSize();

    int3 cell;

    cell.x = floor(posRelativeToGridMin.x / cellSize.x);
    cell.y = floor(posRelativeToGridMin.y / cellSize.y);
    cell.z = floor(posRelativeToGridMin.z / cellSize.z);

    return cell;
}

__device__ float3 Grid::getCellCenter(const int3& gridCellCoordinate) const
{
    const float3 cellSize = getCellSize();

    return make_float3(
                worldMin.x + (cellSize.x * gridCellCoordinate.x) + (cellSize.x / 2.0f),
                worldMin.y + (cellSize.y * gridCellCoordinate.y) + (cellSize.y / 2.0f),
                worldMin.z + (cellSize.z * gridCellCoordinate.z) + (cellSize.z / 2.0f)
                );
}
__device__ float3 Grid::getCellSize() const
{
    return make_float3(
                (worldMax.x - worldMin.x) / cells.x,
                (worldMax.y - worldMin.y) / cells.y,
                (worldMax.z - worldMin.z) / cells.z
                );
}

// Calculate grid hash value for each particle
__global__
void computeMappingFromGridCellToParticleD(
        uint*   gridParticleHash,  // output
        uint*   gridParticleIndex, // output
        float4* pos,               // input: particle positions
        Grid*   grid,              // input: pointer to the grid being used
        uint    numParticles)
{
    const unsigned int index = getThreadIndex1D();
    if(index >= numParticles) return;

    volatile float4 p = pos[index];

    // In which grid cell does the particle live?
    int3 gridPos = grid->getCellCoordinate(make_float3(p.x, p.y, p.z));

    // Calculate the particle's hash from the grid-cell. This means particles in the same cell have the same hash
    uint hash = grid->getCellHash(gridPos);

    // This array is the key-part of the map, mapping cellId (=hash) to particleIndex. The term "map" is not
    // exactly correct, because there can be multiple keys (because one cell can store many particles)
    gridParticleHash[index] = hash;

    // It seems stupid to fill an array like "array[x]=x". But this array is the value-part of a map and will get sorted according to the keys (=gridParticleHash)
    gridParticleIndex[index] = index;
}

// rearrange particle data into sorted order (sorted according to containing grid cell), and find the start of each cell in the sorted hash array
__global__
void sortPosAndVelAccordingToGridCellAndFillCellStartAndEndArraysD(
        uint*   cellStart,         // output: cell start index
        uint*   cellEnd,           // output: cell end index
        float4* posSorted,         // output: sorted positions, sorted according to the containing gridcell
        float4* velSorted,         // output: sorted velocities, sorted according to the containing gridcell
        uint*   gridParticleHash,  // input:  sorted grid hashes
        uint*   gridParticleIndex, // input:  sorted particle indices
        float4* posUnsorted,       // input:  unsorted position array
        float4* velUnsorted,       // input:  unsorted velocity array
        uint    numParticles       // input:  number of particles/colliders
        )
{
    const unsigned int threadIndex = getThreadIndex1D();

    // This resides in shared memory space of the threadBlock, lives as
    // long as the block and is accessible from all threads in the block.
    // Its size (in bytes) is defined at runtime through the Ns parameter
    // in the <<Dg, Db, Ns, S>> expression of the caller.
    // Here, its set to ((ThreadsInBlock + 1) elements)
    extern __shared__ uint sharedHash[];

    uint hash;

    // When particleCount is smaller than a multiple of the block size, the remaining threads do nothing.
    if(threadIndex < numParticles)
    {
        hash = gridParticleHash[threadIndex];

        // Load hash data into shared memory so that we can look at neighboring
        // particle's hash value without loading two hash values per thread
        sharedHash[threadIdx.x+1] = hash; // => key of the sorted map

        if(threadIndex > 0 && threadIdx.x == 0)
        {
            // first thread in block must load neighbor particle hash
            sharedHash[0] = gridParticleHash[threadIndex-1];
        }
    }

    __syncthreads();

    if (threadIndex < numParticles)
    {
        // If this particle has a different cell index to the previous particle then it must be the
        // first particle in the cell, so store the index of this particle in the cell. As it isn't
        // the first particle, it must also be the cell end of the previous particle's cell
        if(threadIndex == 0 || hash != sharedHash[threadIdx.x])
        {
            cellStart[hash] = threadIndex;
            if (threadIndex > 0)
                cellEnd[sharedHash[threadIdx.x]] = threadIndex;
        }

        if(threadIndex == numParticles - 1)
        {
            cellEnd[hash] = threadIndex + 1;
        }

        // Now use the sorted index to reorder the pos and vel data
        uint sortedIndex = gridParticleIndex[threadIndex]; // => value of the sorted map
        float4 pos, vel;

        // Only use vel if passed vels are non-zero. This way, we can use this method also for colliders, which have no velocities.
        pos = posUnsorted[sortedIndex];
        if(velUnsorted && velSorted)
            vel = velUnsorted[sortedIndex];

        // ben: hier if() beenden, dann syncthreads() und dann nicht in sortedPos schreiben, sondern in oldPos? Bräuchte ich dann noch zwei pos/vel container?
        posSorted[threadIndex] = pos;
        if(velUnsorted && velSorted) velSorted[threadIndex] = vel;
    }
}




// Calculates a hash for each particle. The hash value is ("based on") its cell id.
void computeMappingFromGridCellToParticle(
        uint*  gridParticleHash,
        uint*  gridParticleIndex,
        float* pos,
        Grid*  grid,
        int    numParticles)
{
    if(numParticles == 0) return;

    uint numThreads, numBlocks;
    computeExecutionKernelGrid(numParticles, KERNEL_LAUNCH_BLOCKSIZE, numBlocks, numThreads);

    // execute the kernel
    computeMappingFromGridCellToParticleD<<< numBlocks, numThreads >>>(gridParticleHash,
                                           gridParticleIndex,
                                           (float4 *) pos,
                                           grid,
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
    computeExecutionKernelGrid(numParticles, KERNEL_LAUNCH_BLOCKSIZE, numBlocks, numThreads);

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
}
#endif // __CUDACC__

#if false
QVector3D Grid::getWorldSizeQt() const
{
    return QVector3D(
                worldMax.x - worldMin.x,
                worldMax.y - worldMin.y,
                worldMax.z - worldMin.z);
}

QVector3D Grid::getWorldCenterQt() const
{
    return QVector3D(worldMin.x, worldMin.y, worldMin.z) + getWorldSizeQt() / 2.0f;
}

int3 Grid::getCellCoordinate(const QVector3D& worldPos) const
{
    const QVector3D posRelativeToGridMin = worldPos - QVector3D(worldMin.x, worldMin.y, worldMin.z);
    const QVector3D cellSize = getCellSizeQt();

    int3 cell;

    cell.x = floor(posRelativeToGridMin.x() / cellSize.x());
    cell.y = floor(posRelativeToGridMin.y() / cellSize.y());
    cell.z = floor(posRelativeToGridMin.z() / cellSize.z());

    return cell;
}


QVector3D Grid::getCellSizeQt() const
{
    return QVector3D(
                (worldMax.x - worldMin.x) / cells.x,
                (worldMax.y - worldMin.y) / cells.y,
                (worldMax.z - worldMin.z) / cells.z
                );
}

QVector3D Grid::getCellCenterQt(const int3& gridCellCoordinate) const
{
    const QVector3D cellSize = getCellSizeQt();

    return QVector3D(
                worldMin.x + (cellSize.x() * gridCellCoordinate.x) + (cellSize.x() / 2.0f),
                worldMin.y + (cellSize.y() * gridCellCoordinate.y) + (cellSize.y() / 2.0f),
                worldMin.z + (cellSize.z() * gridCellCoordinate.z) + (cellSize.z() / 2.0f)
                );
}

#endif
