#ifndef PARTICLES_KERNEL_H
#define PARTICLES_KERNEL_H

// This file exists to share the SimParams-Struct between .cpp and .cu-code,
// both particlesystem.h and particleskernel.cu include it.

#include "vector_types.h"

// simulation parameters
struct SimulationParameters {
    // gridSize is used for TWO grids: (we might split this up into two members later)
    // - one grid contains the particles, its used to find neighbors for collisions
    // - one grid contains the colliders, its used to find neighbors for keeping it sparse
    uint3 gridSize;

    // radius of the particles. The radius of colliders is 0 right now
    float particleRadius;
    float3 gravity;
    float3 worldMin, worldMax;

    unsigned int particleCount;
    unsigned int colliderCountMax;

    float spring;
    float shear;
    float attraction;

    float dampingMotion;
    float velocityFactorCollisionParticle;
    float velocityFactorCollisionBoundary;
};

#endif
