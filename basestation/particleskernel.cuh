#ifndef PARTICLES_KERNEL_H
#define PARTICLES_KERNEL_H

#define USE_TEX 1

#if USE_TEX
#define FETCH(t, i) tex1Dfetch(t##Tex, i)
#else
#define FETCH(t, i) t[i]
#endif

#include "vector_types.h"
//typedef unsigned int uint;

// simulation parameters
struct SimParams {
    float3 colliderPos;
    float  colliderRadius;    

    float3 gravity;
    float globalDamping;
    float particleRadius;

    uint3 gridSize;
    unsigned int numCells;
    float3 worldMin, worldMax;
    float3 cellSize;

    unsigned int numBodies;
    unsigned int maxParticlesPerCell;

    float spring;
    float damping;
    float shear;
    float attraction;
    float boundaryDamping;
};

#endif
