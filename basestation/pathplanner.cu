// Fix for gcc 4.7
#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#include "grid.cuh"
#include "cudahelper.cuh"
#include "helper_math.h"

#include "parameterspathplanner.cuh"
#include "pathplanner.cuh"

// only for printf debugging
#include <stdlib.h>
#include <stdio.h>

// simulation parameters in constant memory
__constant__ ParametersPathPlanner parametersPathPlanner;
__constant__ Grid growingGrid;

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
                //if(cellIndex == 0) printf("cellIndex 0, coord 0/0/0 neighbor %d/%d/%d\n", x, y, z);

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
int bound(int min, int value, int max)
{
    if(value < min)
        return min;
    else if(value > max)
        return max;
    else
        return value;
}

__global__ void markStartCellD(u_int8_t* gridValues)
{
    int3 cellCoordinateStart = parametersPathPlanner.grid.getCellCoordinate(parametersPathPlanner.start);
    int cellIndexStart = parametersPathPlanner.grid.getCellHash2(cellCoordinateStart);

    if(cellIndexStart > 0)
    {
        printf("markStartCellD(): setting start cell %d to 1\n", cellIndexStart);
        gridValues[cellIndexStart] = 1;
    }
    else
    {
        printf("markStartCellD(): start cell %.1f/%.1f/%.1f is outside grid!\n", parametersPathPlanner.start.x, parametersPathPlanner.start.y, parametersPathPlanner.start.z);
    }
}

__global__
void growGridD(u_int8_t* gridValues, Grid subGrid)
{
    uint subGridCellHash = getThreadIndex1D();
    if(subGridCellHash >= subGrid.getCellCount()) return;

    float3 subGridCellCenter = subGrid.getCellCenter(subGrid.getCellCoordinate(subGridCellHash));
    int3 superGridCellCoordinate = parametersPathPlanner.grid.getCellCoordinate(subGridCellCenter);
    unsigned int superGridCellHash = parametersPathPlanner.grid.getCellHash(superGridCellCoordinate);

    u_int8_t lowestNonNullNeighbor = 254; // thats a dilated cell's value
    u_int8_t ownValue = gridValues[superGridCellHash];

    if(ownValue == 0)
    {
        // Check all neighbors for the lowest value d != 0,254,255 and put d++ into our own cell.
        for(int z=-1;z<=1;z++)
        {
            for(int y=-1;y<=1;y++)
            {
                for(int x=-1;x<=1;x++)
                {
                    // don't look into our own cell for neighboring values!
                    if(x == 0 && y == 0 && z == 0)
                    {
                        //printf("will not check myself for neighbors.\n");
                        continue;
                    }

                    const int3 neighbourGridCellCoordinate = superGridCellCoordinate + make_int3(x,y,z);

                    const int neighbourGridCellIndex = parametersPathPlanner.grid.getCellHash2(neighbourGridCellCoordinate);

                    // Border-cells might ask for neighbors outside of the grid. getCellHash2() returns -1 in those cases.
                    if(neighbourGridCellIndex >= 0)
                    {
                        const u_int8_t neighborValue = gridValues[neighbourGridCellIndex];

                        // Find the lowest neighbor that is neither 0 nor 255
                        if(neighborValue < lowestNonNullNeighbor && neighborValue != 0)
                            lowestNonNullNeighbor = neighborValue;
                    }
                    else
                    {
                        // @subGrid should be clamped to the super grid, so this happens only when checking the non-existing neighbors of border cells
                        /*printf("bug, neighborgridcellindex is %d, super-coord was %d/%d/%d, neighbor-coord was %d/%d/%d\n",
                               neighbourGridCellIndex,
                               superGridCellCoordinate.x,
                               superGridCellCoordinate.y,
                               superGridCellCoordinate.z,
                               neighbourGridCellCoordinate.x,
                               neighbourGridCellCoordinate.y,
                               neighbourGridCellCoordinate.z);*/
                    }
                }
            }
        }

        // Write our cell's value. A cell first contains a 0, then the neighborCellValue+1. Once it does
        // contain a value, it will never change. We're only interested in replacing the value with lower
        // numbers, but since the values spread like a wave, that'll never happen.
        if(lowestNonNullNeighbor < 254/* && ownValue == 0*/)
        {
            /*printf("found value %d in neighbor, setting sub-cell %d / super-cell %d (%d/%d/%d) from %d to %d\n",
                   lowestNonNullNeighbor,
                   subGridCellHash,
                   superGridCellHash,
                   superGridCellCoordinate.x, superGridCellCoordinate.y, superGridCellCoordinate.z,
                   ownValue,
                   lowestNonNullNeighbor + 1);*/

            gridValues[superGridCellHash] = lowestNonNullNeighbor + 1;
        }
        else
        {
            /*printf("failed to find an interesting neighbor for sub-grid-cell %d, super-grid-cell %d (%3d/%3d/%3d) with value %d\n",
                   subGridCellHash,
                   superGridCellHash,
                   superGridCellCoordinate.x,
                   superGridCellCoordinate.y,
                   superGridCellCoordinate.z,
                   ownValue);*/
        }
    }
    else
    {
        /*printf("sub-grid-cell %d, super-grid-cell %d (%3d/%3d/%3d) already has value %d\n",
               subGridCellHash,
               superGridCellHash,
               superGridCellCoordinate.x,
               superGridCellCoordinate.y,
               superGridCellCoordinate.z,
               ownValue);*/
    }
}

void markStartCell(unsigned char* gridValues, cudaStream_t *stream)
{
    // set the cell containing "start" to 1!
    markStartCellD<<<1, 1, 0, *stream>>>(gridValues);
    cudaCheckSuccess("markStartCellD");
}

void growGrid(unsigned char* gridValues, ParametersPathPlanner* parameters, cudaStream_t *stream)
{
    uint numThreads, numBlocks;

    const int3 cellCoordinateStart = parameters->grid.getCellCoordinate(parameters->start);


    const unsigned int longestSideCellCount = parameters->grid.getLongestSideCellCount();

    int3 iterationCellMin, iterationCellMax, lastCellMin, lastCellMax;

    for(unsigned int i=1;i<longestSideCellCount;i++)
    {
        iterationCellMin = parameters->grid.clampCellCoordinate(cellCoordinateStart + make_int3(-i, -i, -i));
        iterationCellMax = parameters->grid.clampCellCoordinate(cellCoordinateStart + make_int3(+i, +i, +i));

        if(iterationCellMin == lastCellMin && iterationCellMax == lastCellMax)
        {
            // cell coordinates haven't changed, so we have grown the whole grid.
            break;
        }
        else
        {
            lastCellMin = iterationCellMin;
            lastCellMax = iterationCellMax;
        }

        Grid iterationGrid;
        iterationGrid.cells.x = iterationCellMax.x - iterationCellMin.x + 1;
        iterationGrid.cells.y = iterationCellMax.y - iterationCellMin.y + 1;
        iterationGrid.cells.z = iterationCellMax.z - iterationCellMin.z + 1;
        float3 superGridCellSize = parameters->grid.getCellSize();
        iterationGrid.worldMin = parameters->grid.getCellCenter(iterationCellMin) - superGridCellSize/2;
        iterationGrid.worldMax = parameters->grid.getCellCenter(iterationCellMax) + superGridCellSize/2;

//        cudaSafeCall(cudaMemcpyToSymbol(growingGrid, iterationGrid, sizeof(Grid)));

        computeExecutionKernelGrid(iterationGrid.getCellCount(), 64, numBlocks, numThreads);
        //printf("growGrid(): growing grid in %d cells.\n", iterationGrid.getCellCount());
        growGridD<<< numBlocks, numThreads, 0, *stream>>>(gridValues, iterationGrid);
        cudaCheckSuccess("growGridD");
    }
}

__global__
void checkGoalCellD(unsigned char* gridValues, unsigned int numCells, unsigned int searchRange, unsigned int *status)
{
    int3 goalGridCellCoordinate = parametersPathPlanner.grid.getCellCoordinate(parametersPathPlanner.goal);
    int goalGridCellOffset = parametersPathPlanner.grid.getCellHash2(goalGridCellCoordinate);

    uint valueInGoalCell = gridValues[goalGridCellOffset];

    printf("checkGoalCellD(): value in goal cell at %.2f/%.2f/%.2f is %d.\n",
           parametersPathPlanner.goal.x,
           parametersPathPlanner.goal.y,
           parametersPathPlanner.goal.z,
           valueInGoalCell);

    if(valueInGoalCell < 254)
    {
        return;
    }
    else
    {
        // Cell is occupied or dilated-occupied! Try to find an empty neighbor!

        // With searchRange = 3, create an array {1,-1,2,-2,3,-3}
        float *neighborsSearchOrder = new float[searchRange * 2];
        for(int i=1;i<=searchRange;i++)
        {
            neighborsSearchOrder[2*i-2] = i;
            neighborsSearchOrder[2*i-1] = -i;
        }

        //for(...)

        delete neighborsSearchOrder;
    }
}


// This method checks whether the goal cell is occupied. If so, it tries
// to find a free neighboring cell that can be used instead.
GoalCellStatus checkGoalCell(unsigned char* gridValues, unsigned int numCells, unsigned int searchRange, cudaStream_t *stream)
{
    if(numCells == 0) return GoalCellBlocked;

    u_int32_t* statusDevice;
    cudaSafeCall(cudaMalloc((void**)statusDevice, sizeof(u_int32_t)));

    checkGoalCellD<<< 1, 1, 0, *stream>>>(gridValues, numCells, searchRange, statusDevice);
    cudaCheckSuccess("checkGoalCell");

    u_int32_t statusHost;
    cudaSafeCall(cudaMemcpy(&statusHost, statusDevice, sizeof(u_int32_t), cudaMemcpyDeviceToHost));

    if(statusHost == 0)
    {
        return GoalCellFree;
    }
    else if(statusHost == 1)
    {
        return GoalCellMoved;
    }
    else
    {
        return GoalCellBlocked;
    }
}



__global__
void retrievePathD(unsigned char* gridValues, float4* waypoints)
{
    int3 gridCellCoordinateGoal = parametersPathPlanner.grid.getCellCoordinate(parametersPathPlanner.goal);
    int gridCellOffsetGoal = parametersPathPlanner.grid.getCellHash2(gridCellCoordinateGoal);

    int3 gridCellCoordinateStart = parametersPathPlanner.grid.getCellCoordinate(parametersPathPlanner.start);

    uint valueInGoalCell = gridValues[gridCellOffsetGoal];
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
        int3 cellCoordinate = gridCellCoordinateGoal;

        // Saves the direction/offset that we step to get to the next cell.
        int3 cellOffset;

        do
        {
            // We are at cellCoordinate and found a value of distance. Now check all neighbors
            // until we find one with a smaller value. Thats the path backwards towards the goal.

            bool foundNextCellTowardsTarget = false;

            if(!foundNextCellTowardsTarget)
            {
                // Paths found using the three nested loops below often look strange, because we search
                // in certain directions first. To prevent this, we first search the cell towards the
                // direction of the goal...

                cellOffset = make_int3(
                            cudaBound(-1, gridCellCoordinateStart.x - cellCoordinate.x, 1),
                            cudaBound(-1, gridCellCoordinateStart.y - cellCoordinate.y, 1),
                            cudaBound(-1, gridCellCoordinateStart.z - cellCoordinate.z, 1));

                int3 neighbourCellCoordinate = cellCoordinate + cellOffset;
                int neighbourCellIndex = parametersPathPlanner.grid.getCellHash2(neighbourCellCoordinate);
                if(neighbourCellIndex < 0) printf("bug!!!");

                u_int8_t neighborValue = gridValues[neighbourCellIndex];

                if(neighborValue < stepsToStartCell)
                {
                    printf("retrievePathD(): found next cell by steppig towards target: %d/%d/%d\n", cellOffset.x, cellOffset.y, cellOffset.z);

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

            if(!foundNextCellTowardsTarget)
            {
                // ...if that doesn't work, we search in the same direction that we came
                // from in the last step. This will at least give a straight path.

                int3 neighbourCellCoordinate = cellCoordinate + cellOffset;
                int neighbourCellIndex = parametersPathPlanner.grid.getCellHash2(neighbourCellCoordinate);

                // by simply repeating the last step, we might walk out of the grid. In that case, try the next method.
                if(neighbourCellIndex < 0) break;

                u_int8_t neighborValue = gridValues[neighbourCellIndex];

                if(neighborValue < stepsToStartCell)
                {
                    printf("retrievePathD(): found next cell by repeating last step: %d/%d/%d\n", cellOffset.x, cellOffset.y, cellOffset.z);

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

            // now try the brute-force approach...
            for(int z=-1;z<=1 && !foundNextCellTowardsTarget;z++)
            {
                for(int y=-1;y<=1 && !foundNextCellTowardsTarget;y++) // check lower paths first
                {
                    for(int x=-1;x<=1 && !foundNextCellTowardsTarget;x++)
                    {
                        cellOffset = make_int3(x,y,z);
                        int3 neighbourCellCoordinate = cellCoordinate + cellOffset;

                        int neighbourCellIndex = parametersPathPlanner.grid.getCellHash2(neighbourCellCoordinate);

                        if(neighbourCellIndex >= 0)
                        {
                            u_int8_t neighborValue = gridValues[neighbourCellIndex];

                            if(neighborValue < stepsToStartCell)
                            {
                                printf("retrievePathD(): found next cell using all neighbors: %d/%d/%d\n", cellOffset.x, cellOffset.y, cellOffset.z);

                                // We found a neighbor with a smaller distance. Use it!
                                cellCoordinate = neighbourCellCoordinate;

                                // Append our current cell's position to the waypoint list.
                                float3 cellCenter = parametersPathPlanner.grid.getCellCenter(cellCoordinate);
                                //printf("retrievePathD(): found next cell towards start at %.2f/%.2f/%.2f%d.\n", cellCenter.x, cellCenter.y, cellCenter.z);

                                // The w-component doesn't matter here, so set to zero. Later on, the w-component
                                // will be set to 1 if it turns out that the waypoint is in a now-occupied cell.
                                waypoints[neighborValue] = make_float4(cellCenter, 0.0);

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


void retrievePath(unsigned char* gridValues, float *waypoints, cudaStream_t *stream)
{
    retrievePathD<<< 1, 1, 0, *stream>>>(gridValues, (float4*)waypoints);
    cudaCheckSuccess("retrievePathD");
}

__global__ void testWayPointCellOccupancyD(unsigned char*  gridValues, float4* upcomingWayPoints, unsigned int numberOfWayPoints)
{
    uint waypointIndex = getThreadIndex1D();
    if(waypointIndex >= numberOfWayPoints) return;

    float4 waypoint = upcomingWayPoints[waypointIndex];

    const int3 gridCellCoordinate = parametersPathPlanner.grid.getCellCoordinate(make_float3(waypoint.x, waypoint.y, waypoint.z));
    const int gridCellHash = parametersPathPlanner.grid.getCellHash2(gridCellCoordinate);

    if(gridCellHash < 0 || gridCellHash > parametersPathPlanner.grid.getCellCount())
        printf("testWayPointCellOccupancyD(): bug!!!\n");

    if(gridValues[gridCellHash] > 253)
    {
        waypoint.w = 1.0;
        upcomingWayPoints[waypointIndex] = waypoint;
    }
}

void testWayPointCellOccupancy(unsigned char*  gridValues, float* upcomingWayPoints, unsigned int numberOfWayPoints, cudaStream_t *stream)
{
    // We take the grid values and the float4-waypoints (with the first element defining the waypoint-count) and set conflicting

    // The number of waypoints available is in upcomingWayPoints[0], but thats in device memory space. So,
    // just start a sufficient number of threads and let the superfuous ones hang out for a while.
    uint numThreads, numBlocks;
    computeExecutionKernelGrid(numberOfWayPoints, 64, numBlocks, numThreads);

    testWayPointCellOccupancyD<<< numBlocks, numThreads, 0, *stream>>>(gridValues, (float4*)upcomingWayPoints, numberOfWayPoints);
    cudaCheckSuccess("fillOccupancyGrid");
}
