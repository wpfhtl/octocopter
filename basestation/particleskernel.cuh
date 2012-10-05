#ifndef PARTICLES_KERNEL_H
#define PARTICLES_KERNEL_H

// This file exists to share the SimParams-Struct between .cpp and .cu-code,
// both particlesystem.h and particleskernel.cu include it.

#include "vector_types.h"

// simulation parameters
struct SimParams {
    uint3 gridSize;
    float particleRadius;
    float3 gravity;
    float3 worldMin, worldMax;

    unsigned int particleCount;

    float spring;
    float shear;
    float attraction;

    float dampingMotion;
    float velocityFactorCollisionParticle;
    float velocityFactorCollisionBoundary;
};

#endif
