// Fix for gcc 4.7
#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#include "grid.cuh"
//#include "cudahelper.h"
#include "cudahelper.cuh"
#include "helper_math.h"

#include "parameterspathplanner.cuh"

// simulation parameters in constant memory
__constant__ ParametersPathPlanner parametersPathPlanner;

void copyParametersToGpu(ParametersPathPlanner *hostParams)
{
    // Copy parameters to constant memory. This was synchronous once, I changed
    // it to be asynchronous. Shouldn't cause any harm, even if parameters were
    // applied one frame too late.
    cudaSafeCall(cudaMemcpyToSymbolAsync(parametersPathPlanner, hostParams, sizeof(ParametersPathPlanner)));
}

/*
void getDeviceAddressOfParametersPathPlanner(ParametersPathPlanner** ptr)
{
    cudaSafeCall(cudaGetSymbolAddress((void**)ptr, parametersPathPlanner));
}*/

__global__
void fillOccupancyGridD(unsigned char* gridValues, float4* colliderPos, unsigned int numColliders)
{
    uint particleIndex = getThreadIndex1D();
    if (particleIndex >= numColliders) return;

    float3 particleToCollidePos = make_float3(colliderPos[particleIndex]);

    // get grid-cell of particle
    int3 particleGridCell = parametersPathPlanner.grid.getCellCoordinate(particleToCollidePos);

    // The cell-hash IS the offset in memory, as cells are adressed linearly
    uint cellHash = parametersPathPlanner.grid.getCellHash(particleGridCell);

    gridValues[cellHash] = 255;
}

void fillOccupancyGrid(unsigned char* gridValues, float* colliderPos, unsigned int numColliders, unsigned int numCells, cudaStream_t *stream)
{
    if(numColliders == 0) return;

    // set all cells to empty
    cudaMemset(gridValues, 0, numCells * sizeof(unsigned char));

    uint numThreads, numBlocks;
    computeExecutionKernelGrid(numColliders, 64, numBlocks, numThreads);

    // Number of bytes in shared memory that is allocated for each (thread)block.
//    uint smemSize = sizeof(uint)*(numThreads+1);

    fillOccupancyGridD<<< numBlocks, numThreads, 0, *stream>>>(gridValues, (float4*)colliderPos, numColliders);

    cudaCheckSuccess("fillOccupancyGrid");
}



__device__
void growCellsD(unsigned char* gridValues, unsigned int numCells)
{
    uint cellIndex = getThreadIndex1D();
    if (cellIndex >= numCells) return;

    // get grid-cell of particle
    int3 threadGridCellCoordinate = parametersPathPlanner.grid.getCellCoordinate(cellIndex);

    unsigned int lowestNonNullNeighbor = 1000; // higher than char could ever be
    unsigned int ownValue = gridValues[cellIndex];

    for(int z=-1; z<=1; z++)
    {
        for(int y=-1; y<=1; y++)
        {
            for(int x=-1; x<=1; x++)
            {
                int3 neighbourGridCellCoordinate = threadGridCellCoordinate + make_int3(x, y, z);

                uint neighbourGridCellOffset = parametersPathPlanner.grid.getCellHash(neighbourGridCellCoordinate);

                if(gridValues[neighbourGridCellOffset] < lowestNonNullNeighbor && gridValues[neighbourGridCellOffset] != 0)
                    lowestNonNullNeighbor = gridValues[neighbourGridCellOffset];
            }
        }
    }

    // Overwrite our cell's value (0 or the lowest distance to "start" Write into our cell if
    if(lowestNonNullNeighbor != 1000 && lowestNonNullNeighbor != 255 && (lowestNonNullNeighbor + 1) < ownValue)
        gridValues[cellIndex] = lowestNonNullNeighbor + 1;
}

__global__
void computePathD(unsigned char* gridValues, unsigned int numCells, float4* waypoints)
{
    // Set the cell containing "start" to 1!
    int3 startGridCellCoordinate = parametersPathPlanner.grid.getCellCoordinate(parametersPathPlanner.start);
    uint startGridCellOffset = parametersPathPlanner.grid.getCellHash(startGridCellCoordinate);
    gridValues[startGridCellOffset] = 1;

    int3 goalGridCellCoordinate = parametersPathPlanner.grid.getCellCoordinate(parametersPathPlanner.goal);
    uint goalGridCellOffset = parametersPathPlanner.grid.getCellHash(goalGridCellCoordinate);

    // Now launch the growCellsD until the goal cell is reached
    uint numThreads, numBlocks;
    computeExecutionKernelGrid(numCells, 64, numBlocks, numThreads);

    uint numIterations = 0;
    do
    {
        growCellsD<<< numBlocks, numThreads >>>(gridValues, numCells);
        numIterations++;
    }
    while(gridValues[goalGridCellOffset] != 0 && numIterations < 512);

    if(numIterations < 512)
    {
        // use this ONE thread to collect the waypoints
        ...
    }
    else
    {
        // Tell the caller we failed by setting the first waypoint to all-zero.
        waypoints[0] = make_float4(0.0, 0.0, 0.0, 0.0);
    }

    // Now traverse from goal back to start and save the world positions in waypoints

}

void computePath(unsigned char* gridValues, unsigned int numCells, float* waypoints, cudaStream_t *stream)
{
    //uint numThreads, numBlocks;
    //computeExecutionKernelGrid(numCells, 64, numBlocks, numThreads);
    //computePathD<<< numBlocks, numThreads, 0, *stream>>>(gridValues, waypoints);

    // Only start one thread. This will use dynamic parallelism to spawn as many as needed as often as needed. Sweet!
    computePathD<<< 1, 1, 0, *stream>>>(gridValues, numCells, (float4*)waypoints);
}
