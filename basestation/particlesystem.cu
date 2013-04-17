// Fix for gcc 4.7
//#undef _GLIBCXX_ATOMIC_BUILTINS
//#undef _GLIBCXX_USE_INT128

#include "thrust/device_ptr.h"
//#include "thrust/device_vector.h"
#include "thrust/for_each.h"
#include "thrust/iterator/zip_iterator.h"
#include "thrust/sort.h"

#include "parametersparticlesystem.cuh"
#include "cuda.h"
#include "particlesystem.cuh"
#include "cudahelper.cuh"
#include "cudahelper.h"
#include "grid.cuh"
#include "helper_math.h"

#include <QDebug>

// simulation parameters in constant memory
__constant__ ParametersParticleSystem parametersParticleSystem;

void copyParametersToGpu(ParametersParticleSystem *hostParams)
{
    // Copy parameters to constant memory. This was synchronous once, I changed
    // it to be asynchronous. Shouldn't cause any harm, even if parameters were
    // applied one frame too late.
    cudaSafeCall(cudaMemcpyToSymbolAsync(parametersParticleSystem, hostParams, sizeof(ParametersParticleSystem)));
}

void getDeviceAddressOfParametersParticleSystem(ParametersParticleSystem** ptr)
{
    cudaSafeCall(cudaGetSymbolAddress((void**)ptr, parametersParticleSystem));
}

// Used to copy the waypoint pressure from a uint8_t vector to the w-components of the cell-position-float4-vector.
// When extracting waypoints form device vectors, this is done, then the cell-positions are sorted DESC according
// to the w component, then the N first waypoints are extracted.
struct copy_functor
{
    __host__ __device__
    copy_functor() {}

    template <typename Tuple>
    //__host__ otherwise we get warnings that params (global mem) cannot be read directly in a host function
    __device__
    void operator()(Tuple t)
    {
        volatile float pressure = thrust::get<0>(t);
        volatile float4 cellWorldPosition = thrust::get<1>(t);

        // store new position and velocity
        thrust::get<1>(t) = make_float4(cellWorldPosition.x, cellWorldPosition.y, cellWorldPosition.z, pressure);
    }
};


struct functorDecreaseWaypointPressure
{
  __device__
  uint8_t operator()(uint8_t x) const
  {
      return sqrtf(x);
  }
};

struct functorComputeWaypointBenefit
{
    float3 vehiclePosition;

    __host__ __device__
    functorComputeWaypointBenefit(float3 vehiclePosition) {this->vehiclePosition = vehiclePosition;}

    template <typename Tuple>
    __device__
    void operator()(Tuple t)
    {
        volatile uint8_t pressureSrc = thrust::get<0>(t);
        //volatile uint8_t pressureDst = thrust::get<1>(t);
        volatile unsigned int cellHash = thrust::get<2>(t);

        float3 cellPosition = parametersParticleSystem.gridWaypointPressure.getCellCenter(parametersParticleSystem.gridWaypointPressure.getCellCoordinate(cellHash));

        float distance = length(cellPosition - vehiclePosition);
        distance = max(1.0f, distance);

        // store waypoint benefit
        thrust::get<1>(t) = (pressureSrc * pressureSrc * pressureSrc) / sqrtf(distance);
    }
};


// Integrate particles, same as above. But for setting the waypointPressureMap, we need out-of-order access that thrust::Tuple cannot provide
__global__
void integrateSystemD(
        float4*         particlePositions,          // in/out: particle positions
        float4*         particleVelocities,         // in/out: particle velocities
        unsigned char*  gridWaypointPressure,       // in/out: grid containing quint8-cells with waypoint-pressure values (80-255)
        float4*         particleCollisionPositions, // input:  particle positions
        const ParametersParticleSystem* const params)                     // input:  the particle system's grid
{
    const unsigned int index = getThreadIndex1D();
    if(index >= params->particleCount) return;

    float3 pos = make_float3(particlePositions[index].x, particlePositions[index].y, particlePositions[index].z);
    float3 vel = make_float3(particleVelocities[index].x, particleVelocities[index].y, particleVelocities[index].z);

    vel += params->gravity * params->timeStepInner;
    vel *= params->dampingMotion;

    // If particle moves further than its radius in one iteration, it may slip through cracks that would be unpassable
    // in reality. To prevent this, do not move particles further than 0.9r in every timestemp
    float3 movement = vel * params->timeStepInner;
    float safeParticleRadius = params->particleRadius * 0.9f;
    float distance = length(movement);
    if(distance >= safeParticleRadius)
    {
        vel /= (distance / safeParticleRadius);
        movement = vel * params->timeStepInner;
    }

    // new position = old position + velocity * params.timeStep
    pos += movement;

#ifdef TRUE
    // particles bounce off the cube's inner sides
    if(pos.x > params->gridParticleSystem.worldMax.x - params->particleRadius) { pos.x = params->gridParticleSystem.worldMax.x - params->particleRadius; vel.x *= params->velocityFactorCollisionBoundary;}
    if(pos.x < params->gridParticleSystem.worldMin.x + params->particleRadius) { pos.x = params->gridParticleSystem.worldMin.x + params->particleRadius; vel.x *= params->velocityFactorCollisionBoundary;}
    if(pos.z > params->gridParticleSystem.worldMax.z - params->particleRadius) { pos.z = params->gridParticleSystem.worldMax.z - params->particleRadius; vel.z *= params->velocityFactorCollisionBoundary;}
    if(pos.z < params->gridParticleSystem.worldMin.z + params->particleRadius) { pos.z = params->gridParticleSystem.worldMin.z + params->particleRadius; vel.z *= params->velocityFactorCollisionBoundary;}
    if(pos.y > params->gridParticleSystem.worldMax.y - params->particleRadius) { pos.y = params->gridParticleSystem.worldMax.y - params->particleRadius; vel.y *= params->velocityFactorCollisionBoundary;}

//    if(pos.y < params.gridParticleSystem.worldMin.y + params.particleRadius) { pos.y = params.gridParticleSystem.worldMin.y + params.particleRadius; vel.y *= params.velocityFactorCollisionBoundary;}
#else
    // particles re-appear on the other end of the axis (pacman-like :)
    if(pos.x > params.gridParticleSystem.worldMax.x - params.particleRadius) { pos.x = params.gridParticleSystem.worldMin.x + params.particleRadius; vel.x *= -params.velocityFactorCollisionBoundary;}
    if(pos.x < params.gridParticleSystem.worldMin.x + params.particleRadius) { pos.x = params.gridParticleSystem.worldMax.x - params.particleRadius; vel.x *= -params.velocityFactorCollisionBoundary;}
    if(pos.z > params.gridParticleSystem.worldMax.z - params.particleRadius) { pos.z = params.gridParticleSystem.worldMin.z + params.particleRadius; vel.z *= -params.velocityFactorCollisionBoundary;}
    if(pos.z < params.gridParticleSystem.worldMin.z + params.particleRadius) { pos.z = params.gridParticleSystem.worldMax.z - params.particleRadius; vel.z *= -params.velocityFactorCollisionBoundary;}
    if(pos.y > params.gridParticleSystem.worldMax.y - params.particleRadius) { pos.y = params.gridParticleSystem.worldMax.y - params.particleRadius; vel.y *= -params.velocityFactorCollisionBoundary;}
#endif

    // The w-component of a particle is always 1.0. If it has hit a collider, it is set to 1.1 instead. When the particle is re-set, w is set to 1.0 again.
    float pos_w = particlePositions[index].w;

    // special case: hitting bottom plane of bounding box
    if(pos.y < params->gridParticleSystem.worldMin.y + params->particleRadius)
    {
        // put the particle back to the top, re-set velocity back to zero
        pos.y = params->gridParticleSystem.worldMax.y - params->particleRadius;
        pos_w = 1.0f;

        // pcpData is the ParticleCollisionPosition, so a non-zero value means this particle has hit a collider and now reached the bottom.
        // Record this in gwpData and re-set the pcpData to zero.
        float3 lastCollisionPosition = make_float3(particleCollisionPositions[index].x, particleCollisionPositions[index].y, particleCollisionPositions[index].z);

        // The w-component tells us whether this particle had a collision, set in collideParticlesWithParticlesAndCollidersD()
        if(particleCollisionPositions[index].w > 0.5f)
        {
            // Find out in what cell the collision occured
            uint hash = params->gridWaypointPressure.getCellHash(
                        params->gridWaypointPressure.getCellCoordinate(
                            lastCollisionPosition
                            )
                        );

            // Increase that cell's waypointpressure
            gridWaypointPressure[hash] = min(gridWaypointPressure[hash] + 1, 255);

            // Clear the particle's last position of collision
            particleCollisionPositions[index] = make_float4(0.0f);
        }
    }

    // store new position and velocity
    particlePositions[index] = make_float4(pos, /*posData.w*/pos_w);
    particleVelocities[index] = make_float4(vel, /*velData.w*/1.0f);
}

// collide two spheres using DEM method
__device__
float3 collideSpheres(
        float3 posParticle,
        float3 velParticle,
        float radiusParticle,
        float3 posCollider,
        float3 velCollider,
        float radiusCollider,
        float attraction,
        float velocityFactorCollisionDashpot,
        bool* particleCollidedWithCollider)
{
    // calculate relative position
    float3 relPos = posCollider - posParticle;

    float distance = length(relPos);
    float collisionDistance = radiusParticle + radiusCollider;

    float3 force = make_float3(0.0f);
    if(distance < collisionDistance)
    {
        if(particleCollidedWithCollider) *particleCollidedWithCollider = true;
        float3 normal = relPos / distance;

        // relative velocity
        float3 relVel = velCollider - velParticle;

        // The spring force. When a particle has penetrated its opponent, the term
        // (distance - collisionDistance) will be negative by the amount of penetration.
        // The normal is pointing from the particle to the collider and has a length of 1.
        // Thus, params.spring should be > 0 to cause the particle to bounce away.
        force = parametersParticleSystem.spring * (distance - collisionDistance) * normal;

        // The dashpot/damping force. relVel is the velocity of the impact, so when
        // velocityFactorCollisionDashpot is positive, it would actually increase the
        // speed towards the collider after the collision. Thus, it should be negative.
        force += velocityFactorCollisionDashpot * relVel;

        // The relative tangential velocity
        float3 tanVel = relVel - (dot(relVel, normal) * normal);
        // The tangential shear force
        force += parametersParticleSystem.shear * tanVel;

        // The attraction. When a particle hits eithe rparticle or collider, it can
        // either be attracted or repulsed. A positive value means attraction.
        force += attraction * relPos;
    }

    return force;
}


// collide a particle against all other particles and colliders in a given cell
__device__
float3 collideCell(
        float4* particleCollisionPositions, // output: storage for the particle's current position if it collides with a collider
        int3    gridCellToSearch,       // input: grid cell to search for particles that could collide
        uint    particleToCollideIndex, // input: index of particle that is being collided
        float3  particleToCollidePos,   // input: position of particle that is being collided
        float3  particleToCollideVel,   // input: velocity of particle that is being collided

        float4* particlePosSorted,      // input: sorted positions  of particles to collide with
        float4* particleVelSorted,      // input: sorted velocities of particles to collide with
        uint*   particleCellStart,      // input: cellStart[x] gives us the index of particle[Pos|Vel]Sorted in which the particles in cell x start
        uint*   particleCellEnd,        // input: cellEnd  [x] gives us the index of particle[Pos|Vel]Sorted in which the particles in cell x end

        float4* colliderPosSorted,      // input: sorted positions of colliders to collide with
        uint*   colliderCellStart,      // input: cellStart[x] gives us the index of colliderPosSorted in which the colliders in cell x start
        uint*   colliderCellEnd,         // input: cellEnd  [x] gives us the index of colliderPosSorted in which the colliders in cell x end
        bool*   particleCollidedWithCollider = 0
        )
{
    uint gridHash = parametersParticleSystem.gridParticleSystem.getCellHash(gridCellToSearch);

    float3 forceCollisionsAgainstParticles = make_float3(0.0f);

    // Collide against other particles. Get start of bucket for this cell
    uint particlesStartIndex = particleCellStart[gridHash];
    // cell is not empty
    if(particlesStartIndex != 0xffffffff)
    {
        // iterate over particles in this cell
        uint particlesEndIndex = particleCellEnd[gridHash];

        for(uint j=particlesStartIndex; j<particlesEndIndex; j++)
        {
            // check not colliding with self
            if (j != particleToCollideIndex)
            {
                float3 posToCollideAgainst = make_float3(particlePosSorted[j]);
                float3 velToCollideAgainst = make_float3(particleVelSorted[j]);

                // collide two spheres
                forceCollisionsAgainstParticles += collideSpheres(
                            particleToCollidePos,
                            particleToCollideVel,
                            parametersParticleSystem.particleRadius,
                            posToCollideAgainst,
                            velToCollideAgainst,
                            parametersParticleSystem.particleRadius,
                            parametersParticleSystem.attraction,
                            parametersParticleSystem.velocityFactorCollisionParticle,
                            0);
            }
        }
    }

    float3 forceCollisionsAgainstColliders = make_float3(0.0f);

    // Collide against other particles. Get start of bucket for this cell
    uint collidersStartIndex = colliderCellStart[gridHash];
    // cell is not empty
    if(collidersStartIndex != 0xffffffff)
    {
        // iterate over particles in this cell
        uint collidersEndIndex = colliderCellEnd[gridHash];

        for(uint j=collidersStartIndex; j<collidersEndIndex; j++)
        {
            float3 posToCollideAgainst = make_float3(colliderPosSorted[j]);

            // collide two spheres
            forceCollisionsAgainstColliders += collideSpheres(
                        particleToCollidePos,
                        particleToCollideVel,
                        parametersParticleSystem.particleRadius,
                        posToCollideAgainst,
                        make_float3(0.0f),
                        parametersParticleSystem.colliderRadius,
                        parametersParticleSystem.attraction,
                        parametersParticleSystem.velocityFactorCollisionCollider,
                        particleCollidedWithCollider);
        }
    }

    return forceCollisionsAgainstParticles + forceCollisionsAgainstColliders;
}


// Collide a single particle (given by thread-id through @index) against all other particles and colliders in own and neighboring cells
__global__
void collideParticlesWithParticlesAndCollidersD(
        float4* newVel,             // output: new velocities. This is actually mDeviceVel, so its the original velocity location
        float4* particlePosVbo,     // output: the w-component is set to 1.1 when the particle hits a collider. Used by the vertex-shader to color the particle
        float4* particleCollisionPositions,// output: Every particle's position of last collision, or 0.0/0.0/0.0 if none occurred.

        float4* particlePosSorted,  // input: particle positions sorted according to containing grid cell
        float4* particleVelSorted,  // input: particle velocities sorted according to containing grid cell
        uint*   particleMapIndex,   // input: particle indices sorted according to containing grid cell
        uint*   particleCellStart,  // input: cellStart[19] contains the index of gridParticleIndex in which particles in cell 19 start
        uint*   particleCellEnd,    // input: cellEnd[19] contains the index of gridParticleIndex in which particles in cell 19 end

        float4* colliderPosSorted,  // input: collider positions sorted according to containing grid cell
        uint*   colliderMapIndex,   // input: collider indices sorted according to containing grid cell
        uint*   colliderCellStart,  // input: cellStart[19] contains the index of gridColliderIndex in which colliders in cell 19 start
        uint*   colliderCellEnd,    // input: cellEnd[19] contains the index of gridColliderIndex in which colliders in cell 19 end

        uint    numParticles)       // input: number of total particles
{
    uint particleToCollideIndex = getThreadIndex1D();
    if (particleToCollideIndex >= numParticles) return;

    // read particle data from sorted arrays
    float3 particleToCollidePos = make_float3(particlePosSorted[particleToCollideIndex]);
    float3 particleToCollideVel = make_float3(particleVelSorted[particleToCollideIndex]);

    // get grid-cell of particle
    int3 particleToCollideGridCell = parametersParticleSystem.gridParticleSystem.getCellCoordinate(particleToCollidePos);

    // examine neighbouring cells
    float3 forceOnParticle = make_float3(0.0f);

    bool particleCollidedWithCollider = false;

    for(int z=-1; z<=1; z++)
    {
        for(int y=-1; y<=1; y++)
        {
            for(int x=-1; x<=1; x++)
            {
                int3 neighbourGridCell = particleToCollideGridCell + make_int3(x, y, z);

                // Collide against other particles and colliders in this cell
                forceOnParticle += collideCell(
                            particleCollisionPositions,
                            neighbourGridCell,
                            particleToCollideIndex,
                            particleToCollidePos,
                            particleToCollideVel,

                            particlePosSorted,
                            particleVelSorted,
                            particleCellStart,
                            particleCellEnd,

                            colliderPosSorted,
                            colliderCellStart,
                            colliderCellEnd,
                            &particleCollidedWithCollider);
            }
        }
    }

    // write new velocity back to original unsorted location
    uint originalIndex = particleMapIndex[particleToCollideIndex];
    newVel[originalIndex] = make_float4(particleToCollideVel + forceOnParticle, 0.0f);

    if(particleCollidedWithCollider)
    {
        // If the particle collided with a collider, we mark its position's w-component, so it can be colored red by the (vertex) shader.
        // The w-component is re-set to 1.0 when the particle reaches the bbox's bottom
        particlePosVbo[originalIndex].w = 1.1f;

        // Store the particle's last collision-position. When the particle reaches the bottom-plane,
        // integrateSystem() increments the value of the cell in which that last collision happened.
        // The w-component of 1.0 is used to detect that a collision took place, xyz mark its position.
        particleCollisionPositions[originalIndex] = make_float4(particleToCollidePos, 1.0f);
    }
}

__global__
void fillGridMapCellWorldPositionsD(
        float4* gridMapCellWorldPositions,
        uint numberOfCells)
{
    uint cellIndex = getThreadIndex1D();
    if(cellIndex >= numberOfCells) return;
/*
    float3 gridCellCoordinate = make_float3(
                floor(fmod((double)cellIndex, (double)(params.gridWaypointPressure.cells.x))),
                floor(fmod((double)cellIndex, (double)(params.gridParticleSystem.cells.x * params.gridParticleSystem.cells.y)) / params.gridParticleSystem.cells.x),
                floor(fmod((double)cellIndex, (double)(params.gridParticleSystem.cells.x * params.gridParticleSystem.cells.y * params.gridParticleSystem.cells.z)) / (params.gridParticleSystem.cells.x * params.gridParticleSystem.cells.y))
                );

    float3 cellSize;
    cellSize.x = (params.gridParticleSystem.worldMax.x - params.gridParticleSystem.worldMin.x) / params.gridParticleSystem.cells.x;
    cellSize.y = (params.gridParticleSystem.worldMax.y - params.gridParticleSystem.worldMin.y) / params.gridParticleSystem.cells.y;
    cellSize.z = (params.gridParticleSystem.worldMax.z - params.gridParticleSystem.worldMin.z) / params.gridParticleSystem.cells.z;


    gridMapCellWorldPositions[cellIndex] = make_float4(
                params.gridParticleSystem.worldMin.x + (cellSize.x * gridCellCoordinate.x) + (cellSize.x / 2.0),
                params.gridParticleSystem.worldMin.y + (cellSize.y * gridCellCoordinate.y) + (cellSize.y / 2.0),
                params.gridParticleSystem.worldMin.z + (cellSize.z * gridCellCoordinate.z) + (cellSize.z / 2.0),
                0.0f
                );
*/
    int3 cellCoordinate = parametersParticleSystem.gridWaypointPressure.getCellCoordinate(cellIndex);
    float3 cellCenter = parametersParticleSystem.gridWaypointPressure.getCellCenter(cellCoordinate);


    gridMapCellWorldPositions[cellIndex] = make_float4(cellCenter, 0.0f);
}

void integrateSystem(float *particlePositions, float *particleVelocities, uint8_t* gridWaypointPressure, float* particleCollisionPositions, const ParametersParticleSystem* const params, uint numParticles)
{
    if(numParticles == 0) return;

    uint numThreads, numBlocks;
    CudaHelper::computeExecutionKernelGrid(numParticles, KERNEL_LAUNCH_BLOCKSIZE, numBlocks, numThreads);

    // execute the kernel
    integrateSystemD<<< numBlocks, numThreads >>>(
                                                    (float4*)particlePositions,          // in/out: particle positions
                                                    (float4*)particleVelocities,         // in/out: particle velocities
                                                    gridWaypointPressure,       // in/out: grid containing quint8-cells with waypoint-pressure values (80-255)
                                                    (float4*)particleCollisionPositions, // input:  particle positions
                                                    params);

    // check if kernel invocation generated an error
    cudaCheckSuccess("integrateSystem");
}



void collideParticlesWithParticlesAndColliders(
        float* newVel,              // output: The particle velocities
        float *particlePosVbo,      // output: The w-component is changed whenever a particle has hit a collider. Used just for visualization.
        float* particleCollisionPositions,          // output: Every particle's position of last collision, or 0.0/0.0/0.0 if none occurred.

        float* particlePosSorted,   // input:  The particle positions, sorted by gridcell
        float* particleVelSorted,   // input:  The particle velocities, sorted by gridcell
        uint*  particleMapIndex,    // input:  The value-part of the particle gridcell->index map, sorted by gridcell
        uint*  particleCellStart,   // input:  At which index in mDeviceMapParticleIndex does cell X start?
        uint*  particleCellEnd,     // input:  At which index in mDeviceMapParticleIndex does cell X end?

        float* colliderSortedPos,   // input:  The collider positions, sorted by gridcell
        uint*  colliderMapIndex,    // input:  The value-part of the collider gridcell->index map, sorted by gridcell
        uint*  colliderCellStart,   // input:  At which index in mDeviceMapColliderIndex does cell X start?
        uint*  colliderCellEnd,     // input:  At which index in mDeviceMapColliderIndex does cell X end?

        uint   numParticles,        // input:  How many particles to collide against other particles (one thread per particle)
        uint   numCells             // input:  Number of grid cells
        )
{

    // thread per particle
    uint numThreads, numBlocks;
    CudaHelper::computeExecutionKernelGrid(numParticles, KERNEL_LAUNCH_BLOCKSIZE, numBlocks, numThreads);

    // execute the kernel
    collideParticlesWithParticlesAndCollidersD<<< numBlocks, numThreads >>>(
                                                                              (float4*)newVel,
                                                                              (float4*)particlePosVbo,
                                                                              (float4*)particleCollisionPositions,

                                                                              (float4*)particlePosSorted,
                                                                              (float4*)particleVelSorted,
                                                                              particleMapIndex,
                                                                              particleCellStart,
                                                                              particleCellEnd,

                                                                              (float4*)colliderSortedPos,
                                                                              colliderMapIndex,
                                                                              colliderCellStart,
                                                                              colliderCellEnd,

                                                                              numParticles);

    // check if kernel invocation generated an error
    cudaCheckSuccess("collideParticlesWithParticlesAndCollidersD");
}

void sortGridOccupancyMap(uint *dGridParticleHash, uint *dGridParticleIndex, uint numParticles)
{
    if(numParticles > 0)
        thrust::sort_by_key(thrust::device_ptr<uint>(dGridParticleHash),                // KeysBeginning
                            thrust::device_ptr<uint>(dGridParticleHash + numParticles), // KeysEnd
                            thrust::device_ptr<uint>(dGridParticleIndex));              // ValuesBeginning

    // check if kernel invocation generated an error
    cudaCheckSuccess("sortGridOccupancyMap");
}


// Fill mDeviceGridMapCellWorldPositions - this might be done only once and then copied lateron (just like the waypoint pressure above)
void fillGridMapCellWorldPositions(float* gridMapCellWorldPositions, uint numCells)
{
    // thread per cell
    uint numThreads, numBlocks;
    CudaHelper::computeExecutionKernelGrid(numCells, KERNEL_LAUNCH_BLOCKSIZE, numBlocks, numThreads);

    fillGridMapCellWorldPositionsD<<< numBlocks, numThreads >>>(
                                                                  (float4*)gridMapCellWorldPositions,
                                                                  numCells);
}

// Sort mDeviceGridMapWayPointPressureSorted => mDeviceGridMapCellWorldPositions according to the keys DESC
void sortGridMapWayPointPressure(float* gridMapWayPointPressureSorted, float* gridMapCellWorldPositions, uint numberOfCells, uint numWaypointsRequested)
{
    if(numberOfCells > 0)
    {
        thrust::sort_by_key(thrust::device_ptr<float>(gridMapWayPointPressureSorted),             // KeysBeginning
                            thrust::device_ptr<float>(gridMapWayPointPressureSorted + numberOfCells),  // KeysEnd
                            thrust::device_ptr<float4>((float4*)gridMapCellWorldPositions),         // ValuesBeginning
                            thrust::greater<float>());                                                // In descending order

        // Now we want to copy the waypointpressure-value for all requested waypoints from gridMapWayPointPressureSorted(quint8) to gridMapCellWorldPositions.w(float)
        thrust::device_ptr<float4>  d_cwp((float4*)gridMapCellWorldPositions);
        thrust::device_ptr<float> d_wpp((float*)gridMapWayPointPressureSorted);

        thrust::for_each(
                    thrust::make_zip_iterator(thrust::make_tuple(d_wpp, d_cwp)),
                    thrust::make_zip_iterator(thrust::make_tuple(d_wpp + numWaypointsRequested, d_cwp + numWaypointsRequested)),
                    copy_functor());
    }

    // check if kernel invocation generated an error
    cudaCheckSuccess("sortGridMapWayPointPressure");
}

uint8_t getMaximumWaypointPressure(uint8_t* gridMapOfWayPointPressure, unsigned int numberOfCells)
{
    if(numberOfCells > 0)
    {
        thrust::device_ptr<uint8_t> result = thrust::max_element(
                    thrust::device_ptr<uint8_t>(gridMapOfWayPointPressure),
                    thrust::device_ptr<uint8_t>(gridMapOfWayPointPressure + numberOfCells));

        // check if kernel invocation generated an error
        cudaCheckSuccess("getMaximumWaypointPressure");

        return *result;
    }

    return 0;
}

void decreaseWaypointPressure(uint8_t* gridMapOfWayPointPressure, unsigned int numberOfCells)
{
    if(numberOfCells > 0)
    {

        // in-place transformation
        thrust::transform(thrust::device_ptr<uint8_t>(gridMapOfWayPointPressure),
                          thrust::device_ptr<uint8_t>(gridMapOfWayPointPressure + numberOfCells),
                          thrust::device_ptr<uint8_t>(gridMapOfWayPointPressure),
                          functorDecreaseWaypointPressure());

    }

    // check if kernel invocation generated an error
    cudaCheckSuccess("decreaseWaypointPressure");
}

void computeWaypointBenefit(float* gridMapOfWayPointPressureDst, uint8_t* gridMapOfWayPointPressureSrc, float* vehiclePosition, unsigned int numberOfCells)
{
    if(numberOfCells > 0)
    {
        //    thrust::device_ptr<float4> d_pos4((float4*)pos);
        //    thrust::device_ptr<float4> d_vel4((float4*)vel);
        //    thrust::device_ptr<float4> d_pcp4((float4*)particleCollisionPositions);
        //    thrust::device_ptr<uint8_t> d_gwpp((uint8_t*)gridWaypointPressure);


        thrust::counting_iterator<unsigned int> cellHash(0);
        thrust::device_ptr<uint8_t> d_wpp_src(gridMapOfWayPointPressureSrc);
        thrust::device_ptr<float> d_wpp_dst(gridMapOfWayPointPressureDst);

        thrust::for_each(
            thrust::make_zip_iterator(thrust::make_tuple(d_wpp_src, d_wpp_dst, cellHash)),
            thrust::make_zip_iterator(thrust::make_tuple(d_wpp_src + numberOfCells, d_wpp_dst + numberOfCells, cellHash + numberOfCells)),
            functorComputeWaypointBenefit(*((float3*)vehiclePosition)));

        /*
        // in-place transformation
        thrust::transform(thrust::device_ptr<uint8_t>(gridMapOfWayPointPressureSrc),
                          thrust::device_ptr<uint8_t>(gridMapOfWayPointPressureSrc + numberOfCells),
                          thrust::device_ptr<uint8_t>(gridMapOfWayPointPressureDst),
                          functorComputeWaypointBenefit((float3*)vehiclePosition));
        */

    }

    // check if kernel invocation generated an error
    cudaCheckSuccess("computeWaypointBenefit");
}
