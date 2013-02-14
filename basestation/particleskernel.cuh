#ifndef PARTICLES_KERNEL_H
#define PARTICLES_KERNEL_H

// This file exists to share the SimParams-Struct between .cpp and .cu-code,
// both particlesystem.h and particleskernel.cu include it.

#include "vector_types.h"

// simulation parameters

struct Grid
{
    uint3 cells;
    float3 worldMin, worldMax;
};

struct SimulationParameters
{
    // gridSize is used for TWO grids: (we might split this up into two members later)
    // - one grid contains the particles, its used to find neighbors for collisions
    // - one grid contains the colliders, its used to find neighbors for keeping it sparse
    Grid particleSystem;
    Grid waypointPressure;

    uint3 particleSystemGridSize;
    float3 particleSystemWorldMin, particleSystemWorldMax;

    uint3 scanVolumeGridSize;
    float3 scanVolumeWorldMin, scanVolumeWorldMax;

    unsigned int particleCount;
    unsigned int colliderCountMax;

    float3 gravity;
    float particleRadius;
    float spring;
    float shear;
    float attraction;
    float dampingMotion;
    float velocityFactorCollisionParticle;
    float velocityFactorCollisionBoundary;
};

#endif
