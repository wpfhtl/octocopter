// Fix for gcc 4.7
#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#include "grid.cuh"
#include "cudahelper.cuh"
#include "helper_math.h"

#include "parameterspathplanner.cuh"

// only for printf debugging
#include <stdlib.h>
#include <stdio.h>

// simulation parameters in constant memory
__constant__ ParametersPathPlanner parametersPathPlanner;

void copyParametersToGpu(ParametersPathPlanner *hostParams)
{
    // Copy parameters to constant memory.
    cudaSafeCall(cudaMemcpyToSymbol(parametersPathPlanner, hostParams, sizeof(ParametersPathPlanner)));
}

__global__
void fillOccupancyGridD(u_int8_t* gridValues, float4* colliderPos, unsigned int numColliders)
{
    uint particleIndex = getThreadIndex1D();

    if(particleIndex >= numColliders) return;
    float3 particleToCollidePos = make_float3(colliderPos[particleIndex]);

    // get grid-cell of particle
    int3 particleGridCell = parametersPathPlanner.grid.getCellCoordinate(particleToCollidePos);

    // The cell-hash IS the offset in memory, as cells are adressed linearly
    int cellHash = parametersPathPlanner.grid.getCellHash2(particleGridCell);

    if(cellHash >= 0) gridValues[cellHash] = 255;
}

__global__
void dilateOccupancyGridD(u_int8_t* gridValues, unsigned int numCells)
{
    // Dilate the occupied cells for additional safety. This also allows expanding routes diagonally
    // later-on, as its ok to pass diagonally between occupied cells after dilation
    uint cellIndex = getThreadIndex1D();
    if(cellIndex >= numCells) return;

    u_int8_t ownValue = gridValues[cellIndex];
    if(ownValue == 255) return;

    int3 threadGridCellCoordinate = parametersPathPlanner.grid.getCellCoordinate(cellIndex);

    for(int z=-1;z<=1;z++)
    {
        for(int y=-1;y<=1;y++)
        {
            for(int x=-1;x<=1;x++)
            {
                const int3 neighbourGridCellCoordinate = threadGridCellCoordinate + make_int3(x,y,z);
                const int neighbourGridCellIndex = parametersPathPlanner.grid.getCellHash2(neighbourGridCellCoordinate);
                if(cellIndex == 0)
                    printf("cellIndex 0, coord 0/0/0 neighbor %d/%d/%d\n", x, y, z);

                if(neighbourGridCellIndex >= 0 && gridValues[neighbourGridCellIndex] == 255)
                {
                    // Because CUDA works using thread-batches, we cannot just load all cells, then compute and then store all cells.
                    // Using batches would mean we would dilate a part of the grid, then load the neighboring part and dilate the
                    // dilation again, making almost all of the grid become occupied.
                    // For this reason, we say that 255 is occupied and 254 is dilated-occupied. This way, we don't need two grids. Hah!
                    gridValues[cellIndex] = 254;
                    return;
                }
            }
        }
    }
}

void fillOccupancyGrid(unsigned char* gridValues, float* colliderPos, unsigned int numColliders, unsigned int numCells, cudaStream_t *stream)
{
    if(numColliders == 0) return;

    // set all cells to empty
    cudaMemset(gridValues, 0, numCells * sizeof(unsigned char));

    uint numThreads, numBlocks;
    computeExecutionKernelGrid(numColliders, 64, numBlocks, numThreads);

    fillOccupancyGridD<<< numBlocks, numThreads, 0, *stream>>>(gridValues, (float4*)colliderPos, numColliders);
    cudaCheckSuccess("fillOccupancyGrid");
}

void dilateOccupancyGrid(unsigned char* gridValues, unsigned int numCells, cudaStream_t *stream)
{
    if(numCells == 0) return;

    uint numThreads, numBlocks;
    computeExecutionKernelGrid(numCells, 64, numBlocks, numThreads);

    dilateOccupancyGridD<<< numBlocks, numThreads, 0, *stream>>>(gridValues, numCells);
    cudaCheckSuccess("dilateOccupancyGridD");
}

__device__
void growCellsD(u_int8_t* gridValues, uint numCells, uint cellIndex)
{
    if(cellIndex >= numCells) return;

    // get grid-cell of particle
    int3 threadGridCellCoordinate = parametersPathPlanner.grid.getCellCoordinate(cellIndex);

    u_int8_t lowestNonNullNeighbor = 254; // thats a dilated cell's value
    u_int8_t ownValue = gridValues[cellIndex];

    // Check all neighbors for the lowest value d != 0,254,255 and put d++ into our own cell.
    for(int z=-1;z<=1;z++)
    {
        for(int y=-1;y<=1;y++)
        {
            for(int x=-1;x<=1;x++)
            {
                const int3 neighbourGridCellCoordinate = threadGridCellCoordinate + make_int3(x,y,z);
                const int neighbourGridCellIndex = parametersPathPlanner.grid.getCellHash2(neighbourGridCellCoordinate);

                // Border-cells might ask for neighbors outside of the grid. getCellHash2() returns -1 in those cases.
                if(neighbourGridCellIndex >= 0)
                {
                    const u_int8_t neighborValue = gridValues[neighbourGridCellIndex];

                    // Find the lowest neighbor that is neither 0 nor 255
                    if(neighborValue < lowestNonNullNeighbor && neighborValue != 0)
                        lowestNonNullNeighbor = neighborValue;
                }
            }
        }
    }

    // Write our cell's value. A cell first contains a 0, then the neighborCellValue+1. Once it does
    // contain a value, it will never change. We're only interested in replacing the value with lower
    // numbers, but since the values spread like a wave, that'll never happen.
    if(lowestNonNullNeighbor < 254 && ownValue == 0)
    {
        //printf("found value %d in neighbor, setting cell %d to %d\n", lowestNonNullNeighbor, cellIndex, lowestNonNullNeighbor + 1);
        gridValues[cellIndex] = lowestNonNullNeighbor + 1;
    }
    else
    {
        /*printf("failed to find an interesting neighbor for cell %d (%3d/%3d/%3d)\n",
               cellIndex,
               threadGridCellCoordinate.x,
               threadGridCellCoordinate.y,
               threadGridCellCoordinate.z);*/
    }
}

__global__
void computePathD(u_int8_t* gridValues, unsigned int numCells)
{
    uint cellIndex = getThreadIndex1D();

    int3 cellCoordinateGoal = parametersPathPlanner.grid.getCellCoordinate(parametersPathPlanner.goal);
    int cellIndexGoal = parametersPathPlanner.grid.getCellHash2(cellCoordinateGoal);

    // Only act if the goal is not occupied
    if(gridValues[cellIndexGoal] == 0 && cellIndexGoal >= 0)
    {
        // Let the first thread set the cell containing "start" to 1!
        if(cellIndex == 0)
        {
            int3 cellCoordinateStart = parametersPathPlanner.grid.getCellCoordinate(parametersPathPlanner.start);
            int cellIndexStart = parametersPathPlanner.grid.getCellHash2(cellCoordinateStart);
            //printf("setting start cell %d to 1, monitoring goal cell %d\n", startGridCellIndex, goalGridCellIndex);
            gridValues[cellIndexStart] = 1;
        }

        // We iterate 2 times the longest straight way through the grid. Thats a quite random choice :|
        uint maxIterations = max(max(parametersPathPlanner.grid.cells.x, parametersPathPlanner.grid.cells.y), parametersPathPlanner.grid.cells.z) * 50;

//        int allCellsReached = 0;

        for(int i=0;i<maxIterations*2 /*&& gridValues[cellIndexGoal] == 0*/; i++)
//        while(!allCellsReached)
        {
            // We want to sync all threads after writing. Because all threads need to reach this barrier,
            // we cannot check cellIndex < numCells above and do that in growCells instead.
            // syncthreads() before first iteration, so that writing 1 into goalcell has an effect.
            __syncthreads();

            if(cellIndex < numCells)
            {
                // get grid-cell of particle
                int3 cellCoordinateThread = parametersPathPlanner.grid.getCellCoordinate(cellIndex);

                u_int8_t lowestNonNullNeighbor = 254; // thats a dilated cell's value
                u_int8_t ownValue = gridValues[cellIndex];

                // Check all neighbors for the lowest value d != 0,254,255 and put d++ into our own cell.

                int3 cellCoordinateMinimum = make_int3(-1, -1, -1);
                for(int z=-1;z<=1;z++)
                {
                    for(int y=-1;y<=1;y++)
                    {
                        for(int x=-1;x<=1;x++)
                        {
                            const int3 cellCoordinateNeighbor = cellCoordinateThread + make_int3(x,y,z);
                            const int neighbourGridCellIndex = parametersPathPlanner.grid.getCellHash2(cellCoordinateNeighbor);

                            // Border-cells might ask for neighbors outside of the grid. getCellHash2() returns -1 in those cases.
                            if(neighbourGridCellIndex >= 0)
                            {
                                const u_int8_t neighborValue = gridValues[neighbourGridCellIndex];

                                // Find the lowest neighbor that is neither 0 nor 255
                                if(neighborValue < lowestNonNullNeighbor && neighborValue != 0)
                                {
                                    cellCoordinateMinimum = cellCoordinateNeighbor;
                                    lowestNonNullNeighbor = neighborValue;
                                }
                            }
                        }
                    }
                }

                // Write our cell's value. A cell first contains a 0, then the neighborCellValue+1. Once it does
                // contain a value, it will never change. We're only interested in replacing the value with lower
                // numbers, but since the values spread like a wave, that'll never happen.
                if(lowestNonNullNeighbor < 254 && ownValue == 0)
                {
                    printf("cell (%d/%d/%d) found value %d in neighbor (%d/%d/%d), setting cell %d to %d\n",
                           cellCoordinateThread.x, cellCoordinateThread.y, cellCoordinateThread.z,
                           lowestNonNullNeighbor,
                           cellCoordinateMinimum.x, cellCoordinateMinimum.y, cellCoordinateMinimum.z,
                           cellIndex, lowestNonNullNeighbor + 1);

                    gridValues[cellIndex] = lowestNonNullNeighbor + 1;
                }

                // finish if ALL threads say that they have a non-null value.
//                allCellsReached = __all(gridValues[cellIndex] != 0);
            }

            /*
            if(cellIndex == 0)
                printf("computePathD(): iteration %d, value at goal cell (%d/%d/%d) after growing: %d.\n",
                       i,
                       cellCoordinateGoal.x,
                       cellCoordinateGoal.y,
                       cellCoordinateGoal.z,
                       gridValues[cellIndexGoal]);*/
        }
    }
}

__global__
void retrievePathD(unsigned char* gridValues, float4* waypoints)
{
    int3 goalGridCellCoordinate = parametersPathPlanner.grid.getCellCoordinate(parametersPathPlanner.goal);
    int goalGridCellOffset = parametersPathPlanner.grid.getCellHash2(goalGridCellCoordinate);

    uint valueInGoalCell = gridValues[goalGridCellOffset];
    /*printf("retrievePathD(): value in goal cell at %.2f/%.2f/%.2f is %d.\n",
           parametersPathPlanner.goal.x,
           parametersPathPlanner.goal.y,
           parametersPathPlanner.goal.z,
           valueInGoalCell);*/

    if(valueInGoalCell == 0)
    {
        // Tell the caller we failed to find a valid path by setting the first waypoint to all-zero.
        waypoints[0] = make_float4(0.0, 0.0, 0.0, 0.0);
    }
    else if(valueInGoalCell == 255 || valueInGoalCell == 254)
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

            for(int z=-1;z<=1 && !foundNextCellTowardsTarget;z++)
            {
                for(int y=1;y>=-1 && !foundNextCellTowardsTarget;y--) // check higher paths first, for safety.
                {
                    for(int x=-1;x<=1 && !foundNextCellTowardsTarget;x++)
                    {
                        int3 neighbourCellCoordinate = cellCoordinate + make_int3(x,y,z);

                        int neighbourCellIndex = parametersPathPlanner.grid.getCellHash2(neighbourCellCoordinate);

                        if(neighbourCellIndex >= 0)
                        {
                            u_int8_t neighborValue = gridValues[neighbourCellIndex];

                            if(neighborValue < stepsToStartCell)
                            {
                                // We found a neighbor with a smaller distance. Use it!
                                cellCoordinate = neighbourCellCoordinate;

                                // Append our current cell's position to the waypoint list.
                                float3 cellCenter = parametersPathPlanner.grid.getCellCenter(cellCoordinate);
                                //printf("retrievePathD(): found next cell towards start at %.2f/%.2f/%.2f%d.\n", cellCenter.x, cellCenter.y, cellCenter.z);
                                waypoints[neighborValue] = make_float4(cellCenter);

                                // Escape those 3 for-loops to continue searching from this next cell.
                                foundNextCellTowardsTarget = true;

                                // Update distance to start-position, should be a simple decrement.
                                stepsToStartCell = neighborValue;
                            }
                        }
                    }
                }
            }
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

    printf("computePath(): computing path in %d cells.\n", numCells);
    computePathD<<< numBlocks, numThreads, 0, *stream>>>(gridValues, numCells);
    cudaCheckSuccess("computePathD");

    retrievePathD<<< 1, 1, 0, *stream>>>(gridValues, (float4*)waypoints);
    cudaCheckSuccess("retrievePathD");
}
