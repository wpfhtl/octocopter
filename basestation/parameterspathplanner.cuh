#ifndef PARAMETERSPATHPLANNER_CUH
#define PARAMETERSPATHPLANNER_CUH

#include "grid.cuh"

// This file exists to share this struct between .cpp and .cu-code,
// both pathplanner.h and pathplanner.cu include it.

struct ParametersPathPlanner
{
    Grid grid;
    float3 start;
    float3 goal;

    void initialize()
    {
        grid.worldMin = make_float3(-32.0f, -4.0f, -32.0f);
        grid.worldMax = make_float3(32.0f, 60.0f, 32.0f);
        grid.cells = make_uint3(64, 64, 64);
    }
};

#endif
