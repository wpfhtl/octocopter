#ifndef PATHPLANNER_CUH
#define PATHPLANNER_CUH

#include <stdint.h>
#include "vector_types.h"
#include "vector_functions.h"
#include "parameterspathplanner.cuh"

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

void computePath(
        unsigned char*  gridValues,
        unsigned int    numCells,
        float*          waypoints,
        cudaStream_t    *stream);

#endif
