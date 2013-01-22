// Fix for gcc 4.7
#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#ifndef PARTICLES_KERNEL_H_
#define PARTICLES_KERNEL_H_

#include "helper_math.h"
#include "particleskernel.cuh"

#include "thrust/tuple.h"

// 0 seems to be 10% faster with 256k particles
/*#define USE_TEX 0

#if USE_TEX
#define FETCH(t, i) tex1Dfetch(t##Tex, i)
#else
#define FETCH(t, i) t[i]
#endif


#if USE_TEX
// textures for particle position and velocity
texture<float4, 1, cudaReadModeElementType> oldPosTex;
texture<float4, 1, cudaReadModeElementType> oldVelTex;

texture<uint, 1, cudaReadModeElementType> gridParticleHashTex;
texture<uint, 1, cudaReadModeElementType> cellStartTex;
texture<uint, 1, cudaReadModeElementType> cellEndTex;
#endif
*/
// simulation parameters in constant memory
__constant__ CollisionParameters params;

// Calculate's a particle's containing cell in the uniform grid
__device__ int3 getGridCellCoordinate(float3 worldPos)
{
    float3 cellSize;
    cellSize.x = (params.worldMax.x - params.worldMin.x) / params.gridSize.x;
    cellSize.y = (params.worldMax.y - params.worldMin.y) / params.gridSize.y;
    cellSize.z = (params.worldMax.z - params.worldMin.z) / params.gridSize.z;

    int3 gridPos;
    gridPos.x = floor((worldPos.x - params.worldMin.x) / cellSize.x);
    gridPos.y = floor((worldPos.y - params.worldMin.y) / cellSize.y);
    gridPos.z = floor((worldPos.z - params.worldMin.z) / cellSize.z);
    return gridPos;
}

// Calculate a particle's hash value (=address in grid) from its containing cell (clamping to edges)
__device__ uint getGridCellHash(int3 gridPos)
{
    gridPos.x = gridPos.x & (params.gridSize.x-1);  // wrap grid, assumes size is power of 2
    gridPos.y = gridPos.y & (params.gridSize.y-1);
    gridPos.z = gridPos.z & (params.gridSize.z-1);
    return __umul24(__umul24(gridPos.z, params.gridSize.y), params.gridSize.x) + __umul24(gridPos.y, params.gridSize.x) + gridPos.x;
}

// deprecated, buggy, unfixable. see non-thrust version below.
struct integrate_functor
{
    float deltaTime;

    __host__ __device__
    integrate_functor(float delta_time) : deltaTime(delta_time) {}

    template <typename Tuple>
    //__host__ otherwise we get warnings that params (global mem) cannot be read directly in a host function
    __device__
    void operator()(Tuple t)
    {
        volatile float4 posData = thrust::get<0>(t);
        volatile float4 velData = thrust::get<1>(t);
        volatile float4 pcpData = thrust::get<2>(t);
        volatile uint8_t gwpData = thrust::get<3>(t);

        float3 pos = make_float3(posData.x, posData.y, posData.z);
        float3 vel = make_float3(velData.x, velData.y, velData.z);

        vel += params.gravity * deltaTime;
        vel *= params.dampingMotion;

        // new position = old position + velocity * deltaTime
        pos += vel * deltaTime;

        // collisions with cube sides
        if (pos.x > params.worldMax.x - params.particleRadius) { pos.x = params.worldMax.x - params.particleRadius; vel.x *= params.velocityFactorCollisionBoundary;}
        if (pos.x < params.worldMin.x + params.particleRadius) { pos.x = params.worldMin.x + params.particleRadius; vel.x *= params.velocityFactorCollisionBoundary;}
        if (pos.z > params.worldMax.z - params.particleRadius) { pos.z = params.worldMax.z - params.particleRadius; vel.z *= params.velocityFactorCollisionBoundary;}
        if (pos.z < params.worldMin.z + params.particleRadius) { pos.z = params.worldMin.z + params.particleRadius; vel.z *= params.velocityFactorCollisionBoundary;}
        if (pos.y > params.worldMax.y - params.particleRadius) { pos.y = params.worldMax.y - params.particleRadius; vel.y *= params.velocityFactorCollisionBoundary;}

        // special case: hitting bottom plane of bounding box
        if (pos.y < params.worldMin.y + params.particleRadius)
        {
            // put the particle back to the top, re-set velocity back to zero
            pos.y = params.worldMax.y - params.particleRadius;
            vel.y = 0.0f;

            // pcpData is the ParticleCollisionPosition, so a non-zero value means this particle has hit a collider and now reached the bottom.
            // Record this in gwpData and re-set the pcpData to zero.
            if(pcpData.x != 0.0f || pcpData.y != 0.0f || pcpData.z != 0.0f || pcpData.w != 0.0f)
            {
                thrust::get<2>(t) = make_float4(0.0f); // clear the particle's last position of collision
                thrust::get<3>(t) = min(gwpData + 1, 255);
            }
        }

        // store new position and velocity
        thrust::get<0>(t) = make_float4(pos, posData.w);
        thrust::get<1>(t) = make_float4(vel, velData.w);
    }
};


// Integrate particles, same as above. But for setting the waypointPressureMap, we need out-of-order access that thrust::Tuple cannot provide
__global__
void integrateSystemD(
        float4*         particlePositions,          // in/out: particle positions
        float4*         particleVelocities,         // in/out: particle velocities
        unsigned char*  gridWaypointPressure,       // in/out: grid containing quint8-cells with waypoint-pressure values (80-255)
        float4*         particleCollisionPositions, // input:  particle positions
        float           deltaTime,
        uint            numParticles)
{
    uint index = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
    if(index >= numParticles) return;

    float3 pos = make_float3(particlePositions[index].x, particlePositions[index].y, particlePositions[index].z);
    float3 vel = make_float3(particleVelocities[index].x, particleVelocities[index].y, particleVelocities[index].z);

    vel += params.gravity * deltaTime;
    vel *= params.dampingMotion;

    // If particle moves further than its radius in one iteration, it may slip through cracks that would be unpassable
    // in reality. To prevent this, do no tmove particles further than r in every timestemp
    float3 movement = vel * deltaTime;
    float factor = length(movement) / params.particleRadius;
    if(factor > 1.0 && false asdasdasd)
    {
        movement /= factor;
    }

    // new position = old position + velocity * deltaTime
    pos += movement;

    // collisions with cube sides
    if (pos.x > params.worldMax.x - params.particleRadius) { pos.x = params.worldMax.x - params.particleRadius; vel.x *= params.velocityFactorCollisionBoundary;}
    if (pos.x < params.worldMin.x + params.particleRadius) { pos.x = params.worldMin.x + params.particleRadius; vel.x *= params.velocityFactorCollisionBoundary;}
    if (pos.z > params.worldMax.z - params.particleRadius) { pos.z = params.worldMax.z - params.particleRadius; vel.z *= params.velocityFactorCollisionBoundary;}
    if (pos.z < params.worldMin.z + params.particleRadius) { pos.z = params.worldMin.z + params.particleRadius; vel.z *= params.velocityFactorCollisionBoundary;}
    if (pos.y > params.worldMax.y - params.particleRadius) { pos.y = params.worldMax.y - params.particleRadius; vel.y *= params.velocityFactorCollisionBoundary;}

    // special case: hitting bottom plane of bounding box
    if (pos.y < params.worldMin.y + params.particleRadius)
    {
        // put the particle back to the top, re-set velocity back to zero
        pos.y = params.worldMax.y - params.particleRadius;
        vel.y = 0.0f;

        // pcpData is the ParticleCollisionPosition, so a non-zero value means this particle has hit a collider and now reached the bottom.
        // Record this in gwpData and re-set the pcpData to zero.
        float3 lastCollisionPosition = make_float3(particleCollisionPositions[index].x, particleCollisionPositions[index].y, particleCollisionPositions[index].z);

        if(lastCollisionPosition.x != 0.0f || lastCollisionPosition.y != 0.0f || lastCollisionPosition.z != 0.0f)
        {
            // Find out in what cell the collision occured
            uint hash = getGridCellHash(getGridCellCoordinate(lastCollisionPosition));

            gridWaypointPressure[hash] = min(gridWaypointPressure[hash] + 1, 255);

            // Clear the particle's last position of collision
            particleCollisionPositions[index] = make_float4(0.0f);
        }
    }

    // store new position and velocity
    particlePositions[index] = make_float4(pos, /*posData.w*/1.0);
    particleVelocities[index] = make_float4(vel, /*velData.w*/1.0);
}

// Calculate grid hash value for each particle
__global__
void computeMappingFromGridCellToParticleD(
        uint*   gridParticleHash,  // output
        uint*   gridParticleIndex, // output
        float4* pos,               // input: particle positions
        uint    numParticles)
{
    uint index = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
    if(index >= numParticles) return;

    volatile float4 p = pos[index];

    // In which grid cell does the particle live?
    int3 gridPos = getGridCellCoordinate(make_float3(p.x, p.y, p.z));

    // Calculate the particle's hash from the grid-cell. This means particles in the same cell have the same hash
    uint hash = getGridCellHash(gridPos);

    // This array is the key-part of the map, mapping cellId (=hash) to particleIndex. The term "map" is not
    // exactly correct, because there can be multiple keys (because one cell can store many particles)
    gridParticleHash[index] = hash;

    // It seems stupid to fill an array like "array[x]=x". But this array is the value-part of a map and will get sorted according to the keys (=gridParticleHash)
    gridParticleIndex[index] = index;
}

// rearrange particle data into sorted order (sorted according to containing grid cell), and find the start of each cell in the sorted hash array
__global__
void sortPosAndVelAccordingToGridCellAndFillCellStartAndEndArraysD(
        uint*   cellStart,         // output: cell start index
        uint*   cellEnd,           // output: cell end index
        float4* posSorted,         // output: sorted positions, sorted according to the containing gridcell
        float4* velSorted,         // output: sorted velocities, sorted according to the containing gridcell
        uint*   gridParticleHash,  // input:  sorted grid hashes
        uint*   gridParticleIndex, // input:  sorted particle indices
        float4* posUnsorted,       // input:  unsorted position array
        float4* velUnsorted,       // input:  unsorted velocity array
        uint    numParticles       // input:  number of particles/colliders
        )
{
    uint threadIndex = __umul24(blockIdx.x,blockDim.x) + threadIdx.x;

    // This resides in shared memory space of the threadBlock, lives as
    // long as the block and is accessible from all threads in the block.
    // Its size (in bytes) is defined at runtime through the Ns parameter
    // in the <<Dg, Db, Ns, S>> expression of the caller.
    // Here, its set to ((ThreadsInBlock + 1) elements)
    extern __shared__ uint sharedHash[];

    uint hash;

    // When particleCount is smaller than a multiple of the block size, the remaining threads do nothing.
    if(threadIndex < numParticles)
    {
        hash = gridParticleHash[threadIndex];

        // Load hash data into shared memory so that we can look at neighboring
        // particle's hash value without loading two hash values per thread
        sharedHash[threadIdx.x+1] = hash; // => key of the sorted map

        if(threadIndex > 0 && threadIdx.x == 0)
        {
            // first thread in block must load neighbor particle hash
            sharedHash[0] = gridParticleHash[threadIndex-1];
        }
    }

    __syncthreads();

    if (threadIndex < numParticles)
    {
        // If this particle has a different cell index to the previous particle then it must be the
        // first particle in the cell, so store the index of this particle in the cell. As it isn't
        // the first particle, it must also be the cell end of the previous particle's cell
        if(threadIndex == 0 || hash != sharedHash[threadIdx.x])
        {
            cellStart[hash] = threadIndex;
            if (threadIndex > 0)
                cellEnd[sharedHash[threadIdx.x]] = threadIndex;
        }

        if(threadIndex == numParticles - 1)
        {
            cellEnd[hash] = threadIndex + 1;
        }

        // Now use the sorted index to reorder the pos and vel data
        uint sortedIndex = gridParticleIndex[threadIndex]; // => value of the sorted map
        float4 pos, vel;

        // Only use vel if passed vels are non-zero. This way, we can use this method also for colliders, which have no velocities.
        pos = posUnsorted[sortedIndex];
        if(velUnsorted && velSorted)
            vel = velUnsorted[sortedIndex];

        // ben: hier if() beenden, dann syncthreads() und dann nicht in sortedPos schreiben, sondern in oldPos? Bräuchte ich dann noch zwei pos/vel container?
        posSorted[threadIndex] = pos;
        if(velUnsorted && velSorted) velSorted[threadIndex] = vel;
    }
}

// collide two spheres using DEM method
__device__
float3 collideSpheres(
        float3 posA,
        float3 velA,
        float radiusA,
        float3 posB,
        float3 velB,
        float radiusB,
        float attraction)
{
    // calculate relative position
    float3 relPos = posB - posA;

    float dist = length(relPos);
    float collideDist = radiusA + radiusB;

    float3 force = make_float3(0.0f);
    if (dist < collideDist)
    {
        float3 norm = relPos / dist;

        // relative velocity
        float3 relVel = velB - velA;

        // relative tangential velocity
        float3 tanVel = relVel - (dot(relVel, norm) * norm);

        // spring force
        force = -params.spring*(collideDist - dist) * norm;
        // dashpot (damping) force
        force += params.velocityFactorCollisionParticle*relVel;
        // tangential shear force
        force += params.shear*tanVel;
        // attraction
        force += attraction*relPos;
    }

    return force;
}


// collide a particle against all other particles in a given cell
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
        uint*   colliderCellEnd         // input: cellEnd  [x] gives us the index of colliderPosSorted in which the colliders in cell x end
        )
{
    uint gridHash = getGridCellHash(gridCellToSearch);

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
                            params.particleRadius,
                            posToCollideAgainst,
                            velToCollideAgainst,
                            params.particleRadius,
                            params.attraction);
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
                        params.particleRadius,
                        posToCollideAgainst,
                        make_float3(0.0f),
                        0.1f,
                        params.attraction);
        }
    }

    // If the particle collided with a collider, store its current position
    if(forceCollisionsAgainstColliders.x != 0.0f || forceCollisionsAgainstColliders.y != 0.0f || forceCollisionsAgainstColliders.z != 0.0f)
    {
        // Store the particle's last collision-position. When the particle reaches the bottom-plane,
        // integrateSystem() increments the value of the cell that last collision appeared in.
        particleCollisionPositions[particleToCollideIndex] = make_float4(particleToCollidePos, 0.0);
    }

    return forceCollisionsAgainstParticles + forceCollisionsAgainstColliders;
}


// Collide a single particle (given by thread-id through @index) against all other particles and colliders in own and neighboring cells
__global__
void collideParticlesWithParticlesAndCollidersD(
        float4* newVel,             // output: new velocities. This is actually mDeviceVel, so its the original velocity location
        float4* particleCollisionPositions,          // output: Every particle's position of last collision, or 0.0/0.0/0.0 if none occurred.

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
    uint particleToCollideIndex = __mul24(blockIdx.x,blockDim.x) + threadIdx.x;
    if (particleToCollideIndex >= numParticles) return;

    // read particle data from sorted arrays
    float3 particleToCollidePos = make_float3(particlePosSorted[particleToCollideIndex]);
    float3 particleToCollideVel = make_float3(particleVelSorted[particleToCollideIndex]);

    // get grid-cell of particle
    int3 particleToCollideGridCell = getGridCellCoordinate(particleToCollidePos);

    // examine neighbouring cells
    float3 forceOnParticle = make_float3(0.0f);

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
                            colliderCellEnd);
            }
        }
    }

    // write new velocity back to original unsorted location
    uint originalIndex = particleMapIndex[particleToCollideIndex];
    newVel[originalIndex] = make_float4(particleToCollideVel + forceOnParticle, 0.0f);
}

#endif
