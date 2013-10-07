// Fix for gcc 4.7
#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#include "grid.cuh"
#include "cudahelper.cuh"
#include "helper_math.h"

#include <stdio.h>

__host__ __device__ float3 Grid::getWorldSize() const
{
    return make_float3(
                worldMax.x - worldMin.x,
                worldMax.y - worldMin.y,
                worldMax.z - worldMin.z);
}

__host__ __device__ float3 Grid::getWorldCenter() const
{
    return make_float3(worldMin.x, worldMin.y, worldMin.z) + getWorldSize()/2.0f;
}

// Calculate a particle cell's hash value (=address in grid) from its containing cell (clamping to edges)
__host__ __device__ unsigned int Grid::getCellHash(int3 gridCellCoordinate) const
{
//    if(gridCellCoordinate.x >= cells.x || gridCellCoordinate.y >= cells.y || gridCellCoordinate.z >= cells.z)
//        return -1;

//    if(gridCellCoordinate.x < 0 || gridCellCoordinate.y < 0 || gridCellCoordinate.z < 0)
//        return -1;

    // If bufsize is a power of 2, then instead of X % bufsize do X & (bufsize-1)
    // This avoids the slow modulo-operator, which uses weird floating-point arithmetic

//    gridCellCoordinate.x = gridCellCoordinate.x & (cells.x-1);  // wrap grid, assumes size is power of 2
//    gridCellCoordinate.y = gridCellCoordinate.y & (cells.y-1);
//    gridCellCoordinate.z = gridCellCoordinate.z & (cells.z-1);
    gridCellCoordinate.x = gridCellCoordinate.x % cells.x;
    gridCellCoordinate.y = gridCellCoordinate.y % cells.y;
    gridCellCoordinate.z = gridCellCoordinate.z % cells.z;

    return (gridCellCoordinate.z * cells.y) * cells.x
            + (gridCellCoordinate.y * cells.x)
            + gridCellCoordinate.x;
}

// Just like the one above, but returns -1 for invalid cells (returns SIGNED int)
__host__ __device__ int Grid::getSafeCellHash(int3 gridCellCoordinate) const
{
    if(
            gridCellCoordinate.x >= cells.x
            || gridCellCoordinate.y >= cells.y
            || gridCellCoordinate.z >= cells.z
            || gridCellCoordinate.x < 0
            || gridCellCoordinate.y < 0
            || gridCellCoordinate.z < 0)
    {
        printf("error, I was asked for a non-existing cell's hash!\n");
        return -1;
    }

    return (gridCellCoordinate.z * cells.y) * cells.x
            + (gridCellCoordinate.y * cells.x)
            + gridCellCoordinate.x;
}

// Given the cell hash (=gl_PrimitiveIDIn), whats the 3d-grid-coordinate of the cell's center?
// This is the reverse of particleskernel.cu -> calcGridHash(int3 gridCell).
__host__ __device__ int3 Grid::getCellCoordinate(const unsigned int hash) const
{
    // http://stackoverflow.com/questions/12252826/modular-arithmetic-on-the-gpu says we should
    // use unsigned int for modulo operands (not result). Luckily, this fits our data types.
    int3 cell;

//    cell.x = floor(fmod((double)hash, cells.x));
//    cell.y = floor(fmod((double)hash, cells.x * cells.y) / cells.x);
//    cell.z = floor(fmod((double)hash, cells.x * cells.y * cells.z) / (cells.x * cells.y));

    cell.x = (hash % cells.x);
    cell.y = (hash % (cells.x * cells.y)) / cells.x;
    cell.z = (hash % (cells.x * cells.y * cells.z) / (cells.x * cells.y));

    return cell;
}

__host__ __device__ int3 Grid::getCellCoordinate(const float3 &worldPos) const
{
    /* this simple if takes 20% wall time of the markCollidingPoint kernel!!!*/
     if(     worldPos.x < worldMin.x ||
            worldPos.y < worldMin.y ||
            worldPos.z < worldMin.z ||
            worldPos.x > worldMax.x ||
            worldPos.y > worldMax.y ||
            worldPos.z > worldMax.z)
    {
        printf("Grid::getCellCoordinate(): pos %.2f/%.2f/%.2f is not in grid from %.2f/%.2f/%.2f to %.2f/%.2f/%.2f\n",
               worldPos.x, worldPos.y, worldPos.z,
               worldMin.x, worldMin.y, worldMin.z,
               worldMax.x, worldMax.y, worldMax.z);

        return make_int3(-1, -1, -1);
    }

    const float3 posRelativeToGridMin = worldPos - worldMin;
    const float3 cellSize = getCellSize();

    int3 cell;

    cell.x = floor(posRelativeToGridMin.x / cellSize.x);
    cell.y = floor(posRelativeToGridMin.y / cellSize.y);
    cell.z = floor(posRelativeToGridMin.z / cellSize.z);

    return cell;
}

__host__ __device__ float3 Grid::getCellCenter(const int3& gridCellCoordinate) const
{
    const float3 cellSize = getCellSize();

    return make_float3(
                worldMin.x + (cellSize.x * gridCellCoordinate.x) + (cellSize.x / 2.0f),
                worldMin.y + (cellSize.y * gridCellCoordinate.y) + (cellSize.y / 2.0f),
                worldMin.z + (cellSize.z * gridCellCoordinate.z) + (cellSize.z / 2.0f)
                );
}
__host__ __device__ float3 Grid::getCellSize() const
{
    return make_float3(
                (worldMax.x - worldMin.x) / cells.x,
                (worldMax.y - worldMin.y) / cells.y,
                (worldMax.z - worldMin.z) / cells.z
                );
}

__host__ __device__ uint3 Grid::getOptimumResolution(const float minDist)
{
    uint3 cells;

    // What would be the ideal cell count in this dimension?
    cells.x = floor((worldMax.x - worldMin.x) / minDist);
    cells.y = floor((worldMax.y - worldMin.y) / minDist);
    cells.z = floor((worldMax.z - worldMin.z) / minDist);

    // Make sure that it doesn't grow beyond hardware limits (256*32*256 = 2M)
    cells.x = cudaBound(1, cells.x, 511);
    cells.y = cudaBound(1, cells.y, 63);
    cells.z = cudaBound(1, cells.z, 511);

    // And use a power of two, as this is needed by Grid::getCellHash().
    // TODO: Actually, that limitation should be removed, because using more grid cells means we're missing neighbors that we should check against!
    cells.x = nextHigherPowerOfTwo(cells.x);
    cells.y = nextHigherPowerOfTwo(cells.y);
    cells.z = nextHigherPowerOfTwo(cells.z);

    return cells;
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

    if(threadIndex < numParticles)
    {
        // If this particle has a different cell index to the previous particle then it must be the
        // first particle in the cell, so store the index of this particle in the cell. As it isn't
        // the first particle, it must also be the cell end of the previous particle's cell
        if(threadIndex == 0 || hash != sharedHash[threadIdx.x])
        {
            cellStart[hash] = threadIndex;
            if(threadIndex > 0)
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

        // I once thought that I could __syncthreads() here to make sure that all threads have fetched the unsorted positions. Then, I would
        // simply write to the sorted indices in the same array. This in-place sort would save me a second array of the same size.
        // Unfortunately, when we read from e.g. posUnsorted[123], __syncthreads() and write to posUnsorted[789], then its unlikely that
        // posUnsorted[789] will have been read by another thread after __syncthreads(). This is because the thread reading posUnsorted[789]
        // is scheduled much later, in another thread block. So, this method can only be used when the number of points is less than the
        // number of points/particles whose indices we're trying to sort. This makes it unpracticable. We need a second positions array!
        // I guess the last sentence should have been: .. can only be used when the # of parallel threads is higher than the # of points.
        posSorted[threadIndex] = pos;
        if(velUnsorted && velSorted) velSorted[threadIndex] = vel;
    }
}



// Calculate grid hash value for each particle
__global__
void computeMappingFromGridCellToParticleD(
        uint*   gridParticleHash,  // output
        uint*   gridParticleIndex, // output
        float4* pos,               // input: particle positions
        const Grid* const   grid,              // input: pointer to the grid being used
        uint    numParticles)
{
    const unsigned int index = getThreadIndex1D();
    if(index >= numParticles) return;

    volatile float4 worldPos = pos[index];

    // Do not process points outside the grid!
    if(     worldPos.x < grid->worldMin.x ||
            worldPos.y < grid->worldMin.y ||
            worldPos.z < grid->worldMin.z ||
            worldPos.x > grid->worldMax.x ||
            worldPos.y > grid->worldMax.y ||
            worldPos.z > grid->worldMax.z)
    {
        printf("computeMappingFromGridCellToParticleD(): particle %d is not in grid, this is an error!\n");
        return;
    }

    // In which grid cell does the particle live?
    const int3 gridCellCoordinate = grid->getCellCoordinate(make_float3(worldPos.x, worldPos.y, worldPos.z));

    // Calculate the particle's hash from the grid-cell. This means particles in the same cell have the same hash
    int hash = grid->getCellHash(gridCellCoordinate);

    // getCellHash() returns -1 if pos is not in grid.
    if(hash >= 0)
    {
        // This array is the key-part of the map, mapping cellId (=hash) to particleIndex. The term "map" is not
        // exactly correct, because there can be multiple keys (because one cell can store many particles)
        gridParticleHash[index] = hash;

        // It seems stupid to fill an array like "array[x]=x". But this array is the value-part of a map and will get sorted according to the keys (=gridParticleHash)
        gridParticleIndex[index] = index;
    }
}


// Calculates a hash for each particle. The hash value is ("based on") its cell id.
void computeMappingFromPointToGridCell(
        uint*  gridParticleHash,
        uint*  gridParticleIndex,
        float* pos,
        const Grid* const grid,
        int    numParticles)
{
    if(numParticles == 0) return;

    uint numThreads, numBlocks;
    computeExecutionKernelGrid(numParticles, 64, numBlocks, numThreads);

    Grid ben;
    cudaMemcpy(&ben, grid, sizeof(Grid), cudaMemcpyDeviceToHost);
//    qDebug() << "grid on device:"
//             << ben.worldMin.x << ben.worldMin.y << ben.worldMin.z
//             << ben.worldMax.x << ben.worldMax.y << ben.worldMax.z
//             << ben.cells.x << ben.cells.y << ben.cells.z;

    // execute the kernel
    computeMappingFromGridCellToParticleD<<< numBlocks, numThreads >>>(
                                                                         gridParticleHash,
                                                                         gridParticleIndex,
                                                                         (float4 *) pos,
                                                                         grid,
                                                                         numParticles
                                                                         );

    // check if kernel invocation generated an error
    cudaCheckSuccess("computeMappingFromPointToGridCell");
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
    if(numParticles == 0) return;

    // set all cells to empty
    cudaMemset(cellStart, 0xffffffff, numCells*sizeof(uint));

    uint numThreads, numBlocks;
    computeExecutionKernelGrid(numParticles, 64, numBlocks, numThreads);

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
