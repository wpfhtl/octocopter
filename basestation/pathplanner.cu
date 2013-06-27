// Fix for gcc 4.7
#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#include "grid.cuh"
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

    unsigned int lowestNonNullNeighbor = 1000; // higher than u_int8_t could ever be
    unsigned int ownValue = gridValues[cellIndex];

    // When growing, we do not use the 26 3d-neighbors, but only 6 (up/down, left/right, front/back).
    // This means that we do not grow diagonally, and later do not travel diagonally. Thats better as
    // it keep us from going diagonally between two occupied cells.
    /*for(int z=-1; z<=1; z++)
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
    }*/

    int3 neighbors[6];
    neighbors[0] = make_int3(1, 0, 0);
    neighbors[1] = make_int3(-1, 0, 0);
    neighbors[2] = make_int3(0, 1, 0);
    neighbors[3] = make_int3(0, -1, 0);
    neighbors[4] = make_int3(0, 0, 1);
    neighbors[5] = make_int3(0, 0, -1);

    for(int i=0;i<6;i++)
    {
        int3 neighbourGridCellCoordinate = threadGridCellCoordinate + neighbors[i];

        uint neighbourGridCellOffset = parametersPathPlanner.grid.getCellHash(neighbourGridCellCoordinate);

        if(gridValues[neighbourGridCellOffset] < lowestNonNullNeighbor && gridValues[neighbourGridCellOffset] != 0)
            lowestNonNullNeighbor = gridValues[neighbourGridCellOffset];
    }

    // Overwrite our cell's value (0 or the lowest distance to "start" Write into our cell if
    if(lowestNonNullNeighbor != 1000 && lowestNonNullNeighbor != 255 && (lowestNonNullNeighbor + 1) < ownValue)
        gridValues[cellIndex] = lowestNonNullNeighbor + 1;
}

__global__
void computePathD(unsigned char* gridValues, unsigned int numCells)
{
    uint cellIndex = getThreadIndex1D();

    int3 goalGridCellCoordinate = parametersPathPlanner.grid.getCellCoordinate(parametersPathPlanner.goal);
    uint goalGridCellOffset = parametersPathPlanner.grid.getCellHash(goalGridCellCoordinate);

    // Only act if the goal is not occupied and don't use more threads than cells
    if(gridValues[goalGridCellOffset] == 0 && cellIndex < numCells)
    {
        // Let the first thread set the cell containing "start" to 1!
        if(cellIndex == 0)
        {
            int3 startGridCellCoordinate = parametersPathPlanner.grid.getCellCoordinate(parametersPathPlanner.start);
            uint startGridCellOffset = parametersPathPlanner.grid.getCellHash(startGridCellCoordinate);
            gridValues[startGridCellOffset] = 1;
        }

        uint numIterations = 0;
        do
        {
            growCellsD(gridValues, numCells);
            numIterations++;
        }
        while(gridValues[goalGridCellOffset] == 0 && numIterations < 512);
    }
}

__global__
void retrievePathD(unsigned char* gridValues, float4* waypoints)
{
    int3 goalGridCellCoordinate = parametersPathPlanner.grid.getCellCoordinate(parametersPathPlanner.goal);
    uint goalGridCellOffset = parametersPathPlanner.grid.getCellHash(goalGridCellCoordinate);

    uint valueInGoalCell = gridValues[goalGridCellOffset];

    if(valueInGoalCell == 0)
    {
        // Tell the caller we failed to find a valid path by setting the first waypoint to all-zero.
        waypoints[0] = make_float4(0.0, 0.0, 0.0, 0.0);
    }
    else if(valueInGoalCell == 255)
    {
        // Tell the caller we failed to find a valid path because of an occupied target cell.
        waypoints[0] = make_float4(0.0, 0.0, 0.0, 1.0);
    }
    else
    {
        // Use this ONE thread to collect all the waypoints. The first float4 will contain
        // the number of waypoints including start and goal. The next float4s will be those
        // waypoints. Add 0.1 so we can cast to int without losing something.
        waypoints[0] = make_float4(valueInGoalCell + 0.1);

        // Set the last waypoint, which equals the goal position
        waypoints[valueInGoalCell] = make_float4(parametersPathPlanner.goal);

        // Now traverse from goal back to start and save the world positions in waypoints
        uint stepsToStartCell = valueInGoalCell;
        int3 cellCoordinate = goalGridCellCoordinate;

        do
        {
            // We are at cellCoordinate and found a value of distance. Now check all neighbors
            // until we find one with a smaller value. Thats the path backwards towards the goal.

            bool foundNextCellTowardsTarget = false;

            // See above for reasons for using only a 6-cell-neighborhood
            int3 neighbors[6];
            neighbors[0] = make_int3(1, 0, 0);
            neighbors[1] = make_int3(-1, 0, 0);
            neighbors[2] = make_int3(0, 1, 0);
            neighbors[3] = make_int3(0, -1, 0);
            neighbors[4] = make_int3(0, 0, 1);
            neighbors[5] = make_int3(0, 0, -1);

            for(int i=0; i<6 && !foundNextCellTowardsTarget; i++)
            {
                int3 neighbourCellCoordinate = cellCoordinate + neighbors[i];

                uint neighbourCellOffset = parametersPathPlanner.grid.getCellHash(neighbourCellCoordinate);

                u_int8_t neighborValue = gridValues[neighbourCellOffset];

                if(neighborValue < stepsToStartCell)
                {
                    // We found a neighbor with a smaller distance. Use it!
                    cellCoordinate = neighbourCellCoordinate;

                    // Append our current cell's position to the waypoint list.
                    waypoints[neighborValue] = make_float4(parametersPathPlanner.grid.getCellCenter(cellCoordinate));

                    // Escape those 3 for-loops to continue searching from this next cell.
                    foundNextCellTowardsTarget = true;

                    // Update distance to start-position, should be a simple decrement.
                    stepsToStartCell = neighborValue;
                }
            }

            /*for(int z=-1; z<=1 && !foundNextCellTowardsTarget; z++)
            {
                for(int y=1; y>=-1 && !foundNextCellTowardsTarget; y--) // try the higher cells first!
                {
                    for(int x=-1; x<=1 && !foundNextCellTowardsTarget; x++)
                    {
                        int3 neighbourCellCoordinate = cellCoordinate + make_int3(x, y, z);

                        uint neighbourCellOffset = parametersPathPlanner.grid.getCellHash(neighbourCellCoordinate);

                        uint neighborValue = gridValues[neighbourCellOffset];

                        if(neighborValue < stepsToStartCell)
                        {
                            // We found a neighbor with a smaller distance. Use it!
                            cellCoordinate = neighbourCellCoordinate;

                            // Append our current cell's position to the waypoint list.
                            waypoints[neighborValue - 1] = make_float4(parametersPathPlanner.grid.getCellCenter(cellCoordinate));

                            // Escape those 3 for-loops to continue searching from this next cell.
                            foundNextCellTowardsTarget = true;

                            // Update distance to start-position, should be a simple decrement.
                            stepsToStartCell = neighborValue;
                        }
                    }
                }
            }*/
        }
        while(stepsToStartCell > 1);

        // waypoints[1] was filled above with the cell-center. But we want it to be the start-position, which
        // - although contained in the cell - is probably not exactly its center.
        waypoints[1] = make_float4(parametersPathPlanner.start);
    }
}

void computePath(unsigned char* gridValues, unsigned int numCells, float *waypoints, cudaStream_t *stream)
{
    uint numThreads, numBlocks;
    computeExecutionKernelGrid(numCells, 64, numBlocks, numThreads);
    computePathD<<< numBlocks, numThreads, 0, *stream>>>(gridValues, numCells);

    retrievePathD<<< 1, 1, 0, *stream>>>(gridValues, (float4*)waypoints);
}
