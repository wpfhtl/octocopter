#ifndef SIMULATIONPARAMETERS_CUH
#define SIMULATIONPARAMETERS_CUH

#include "grid.cuh"

// This file exists to share the SimParams-Struct between .cpp and .cu-code,
// both particlesystem.h and particleskernel.cu include it.

struct ParametersParticleSystem
{
    // gridSize is used for TWO grids: (we might split this up into two members later)
    // - one grid contains the particles, its used to find neighbors for collisions
    // - one grid contains the colliders, its used to find neighbors for keeping it sparse
    Grid gridParticleSystem;

    Grid gridWaypointPressure;

    unsigned int particleCount;
    unsigned int colliderCountMax;

    float3 gravity;
    float timeStepInner;
    float timeStepOuter;
    float colliderRadius;
    float particleRadius;
    float spring;
    float shear;
    float attraction;
    float dampingMotion;
    float velocityFactorCollisionParticle;
    float velocityFactorCollisionCollider;
    float velocityFactorCollisionBoundary;

    void initialize()
    {
        timeStepInner = 0.01f;
        timeStepOuter = 0.06f;
        gridParticleSystem.worldMin = make_float3(-32.0f, -4.0f, -32.0f);
        gridParticleSystem.worldMax = make_float3(32.0f, 60.0f, 32.0f);
        gridParticleSystem.cells = make_uint3(64, 64, 64);

        gridWaypointPressure.worldMin = make_float3(128.0f, -4.0f, 128.0f);
        gridWaypointPressure.worldMax = make_float3(128.0f, 60.0f, 128.0f);
        gridWaypointPressure.cells = make_uint3(1024, 64, 1024);

        particleRadius = 0.5f;
        colliderRadius = 0.1f;
        particleCount = 32768;
        dampingMotion = 0.99f;      // used only for integration
        velocityFactorCollisionCollider = 0.1f;
        velocityFactorCollisionParticle = 0.03f;
        velocityFactorCollisionBoundary = -0.5f;
        gravity = make_float3(0.0f, -9.810f, 0.0f);  // paper
        spring = 0.5f;
        shear = 0.0f;
        attraction = 0.0f;
    }
};

#endif
