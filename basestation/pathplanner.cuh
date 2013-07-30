#ifndef PATHPLANNER_CUH
#define PATHPLANNER_CUH

#include <stdint.h>
#include "vector_types.h"
#include "vector_functions.h"
#include "parameterspathplanner.cuh"

enum GoalCellStatus
{
    GoalCellFree,       // The goal cell was checked and is free in the occupancy grid
    GoalCellMoved,      // The goal cell was checked and occupied, but a neighboring free cell was found
    GoalCellBlocked     // The goal cell was checked and occupied. No free neighbors were found in range
};

void copyParametersToGpu(ParametersPathPlanner *hostParams);

void fillOccupancyGrid(
        unsigned char*  gridValues,
        float*          colliderPos,
        unsigned int    numColliders,
        unsigned int    numCells,
        cudaStream_t*   stream);

void dilateOccupancyGrid(
        unsigned char* gridValues,
        unsigned int numCells,
        cudaStream_t *stream);

void retrievePath(unsigned char*  gridValues,
        float*          waypoints,
        cudaStream_t    *stream);

void markStartCell(
        unsigned char* gridValues,
        cudaStream_t *stream);

void growGrid(
        unsigned char* gridValues,
        ParametersPathPlanner* parameters,
        cudaStream_t *stream);

#endif
