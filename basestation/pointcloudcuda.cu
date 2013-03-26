// Fix for gcc 4.7
//#undef _GLIBCXX_ATOMIC_BUILTINS
//#undef _GLIBCXX_USE_INT128

#include "thrust/sort.h"
#include "thrust/unique.h"
#include <thrust/remove.h>

#include <QDebug>
#include "cuda.h"
#include "cudahelper.cuh"
#include "cudahelper.h"
#include "pointcloudcuda.cuh"
#include "helper_math.h"

#include "grid.cuh"

// pointcloud parameters in constant memory
__constant__ ParametersPointCloud paramsPointCloud;

inline __host__ __device__ bool operator!=(float3 &a, float3 &b)
{
    return !(a.x == b.x && a.y == b.y && a.z == b.z);
}

inline __host__ __device__ bool operator!=(float4 &a, float4 &b)
{
    return !(a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w);
}

// rearrange particle data into sorted order (sorted according to containing grid cell), and find the start of each cell in the sorted hash array

void getDeviceAddressOfParametersPointCloud(ParametersPointCloud** ptr)
{
    cudaSafeCall(cudaGetSymbolAddress((void**)ptr, paramsPointCloud));
}

void copyParametersToGpu(ParametersPointCloud *hostParams)
{
    // copy parameters to constant memory
    cudaSafeCall(cudaMemcpyToSymbol(paramsPointCloud, hostParams, sizeof(ParametersPointCloud)));
}



 /*


__global__
void sortPosAccordingToGridCellAndFillCellStartAndStoppArraysD(
        float4* posUnsorted,             // input: UNsorted position array
        float4* posSorted,          // output: sorted positions, sorted according to the containing gridcell
        uint*   pointCellStart,          // output: cell start index
        uint*   pointCellStopp,            // output: cell end index
        uint *  gridCellIndex,      // input: sorted grid hashes
        uint *  gridParticleIndex,  // input: sorted particle indices
        uint    numParticles)
{
    uint threadIndex = (blockIdx.x * blockDim.x) + threadIdx.x;

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
        hash = gridCellIndex[threadIndex];

        // Load hash data into shared memory so that we can look at neighboring
        // particle's hash value without loading two hash values per thread
        sharedHash[threadIdx.x+1] = hash; // => key of the sorted map

        if(threadIndex > 0 && threadIdx.x == 0)
        {
            // first thread in block must load neighbor particle hash
            sharedHash[0] = gridCellIndex[threadIndex-1];
        }
    }

    __syncthreads();

    float4 pos;

    if (threadIndex < numParticles)
    {
        // If this particle has a different cell index to the previous particle then it must be the
        // first particle in the cell, so store the index of this particle in the cell. As it isn't
        // the first particle, it must also be the cell end of the previous particle's cell
        if(threadIndex == 0 || hash != sharedHash[threadIdx.x])
        {
            pointCellStart[hash] = threadIndex;

            // When pointCellStopp[5] == 20, then the last particle of cell 5 is in index 19, not 20!
            // This is ok, because we do "for(uint j=startIndex; j **  <  ** endIndex; j++)" below.
            if(threadIndex > 0)
                pointCellStopp[sharedHash[threadIdx.x]] = threadIndex;
        }

        if(threadIndex == numParticles - 1)
        {
            pointCellStopp[hash] = threadIndex + 1;
        }

        // Now use the sorted index to reorder the pos and vel data
        uint sortedIndex = gridParticleIndex[threadIndex]; // => value of the sorted map
        pos = posUnsorted[sortedIndex];

        // I once thought that I could __syncthreads() here to make sure that all threads have fetched the unsorted positions. Then, I would
        // simply write to the sorted indices in the same array. This in-place sort would save me a second array of the same size.
        // Unfortunately, when we read from e.g. posUnsorted[123], __syncthreads() and write to posUnsorted[789], then its unlikely that
        // posUnsorted[789] will have been read by another thread after __syncthreads(). This is because the thread reading posUnsorted[789]
        // is scheduled much later, in another thread block. So, this method can only be used when the number of points is less than the
        // number of points/particles whose indices we're trying to sort. This makes it unpracticable. We need a second positions array!
        posSorted[threadIndex] = pos;
    }
}


void sortPosAccordingToGridCellAndFillCellStartAndStoppArrays(
        float* posUnsorted,
        float* posSorted,
        uint*  pointCellStart,
        uint*  pointCellStopp,
        uint*  gridCellIndex,
        uint*  gridPointIndex,
        uint   numPoints,
        uint   numCells)
{
    uint numThreads, numBlocks;
    computeExecutionKernelGrid(numPoints, 256, numBlocks, numThreads);

    // set all cells to empty
    cudaSafeCall(cudaMemset(pointCellStart, 0xffffffff, numCells*sizeof(uint)));

    // Number of bytes in shared memory that is allocated for each (thread)block.
    uint smemSize = sizeof(uint)*(numThreads+1);

    sortPosAccordingToGridCellAndFillCellStartAndStoppArraysD<<< numBlocks, numThreads, smemSize>>>(
                                                                                                      (float4*) posUnsorted,
                                                                                                      (float4*) posSorted,
                                                                                                      pointCellStart,
                                                                                                      pointCellStopp,
                                                                                                      gridCellIndex,
                                                                                                      gridPointIndex,
                                                                                                      numPoints);

    cudaCheckSuccess("sortPosAccordingToGridCellAndFillCellStartAndEndArrays()");
}

*/




/*
// collide a particle against all other particles in a given cell
__device__
bool checkCellForNeighborsD(
        float4* points,                     // a pointer to all points on the GPU
        int3    coordinateOfCellToSearch,   // grid cell to search for particles that could collide
        uint    pointToBeReducedIndex,    // index of particle that is being collided
        float4  pointToBeReducedPos,                // position of particle that is being collided
        uint*   pointCellStart,
        uint*   pointCellStopp)
{
    uint gridHash = paramsPointCloud.grid.getCellHash(coordinateOfCellToSearch);

    // get start of bucket for this cell
    uint startIndex = pointCellStart[gridHash];

    // cell is not empty
    if(startIndex != 0xffffffff)
    {
        // iterate over particles in this cell
        uint endIndex = pointCellStopp[gridHash];
        for(uint j=startIndex; j<endIndex; j++)
        {
            // check not colliding with self
            if (j != pointToBeReducedIndex)
            {
                float4 posOther = points[j];

                float4 posRelative = pointToBeReducedPos - posOther;

                float dist = length(posRelative);

                // If they collide AND we're checking the point that was further from the scanner, THEN reduce it!
                if(dist < paramsPointCloud.minimumDistance && pointToBeReducedPos.w > posOther.w && posOther.w != 0.0)
                {
                    return true;
                }
            }
        }
    }

    return false;
}

// Collide a single particle (given by thread-id through @index) against all spheres in own and neighboring cells
__global__
void clearClosePointsPreferringSmallerWComponentD(
        float4* points,     // output: new positions, same or zeroed. This is actually mDevicePointPos, so its the original position location
        uint*   gridPointIndex,  // input: particle indices sorted according to containing grid cell
        uint*   pointCellStart,       // input: pointCellStart[19] contains the index of gridParticleIndex in which cell 19 starts
        uint*   pointCellStopp,         // input: pointCellStopp[19] contains the index of gridParticleIndex in which cell 19 ends
        uint    offsetOfPointsToBeReduced,
        uint    numberOfPointsToBeReduced)       // input: number of total particles
{
    uint index = (blockIdx.x * blockDim.x) + threadIdx.x;
    if(index >= numberOfPointsToBeReduced) return;

    unsigned int indexOfPointToBeReduced = offsetOfPointsToBeReduced + index;

    float4 positionOfPointToBeReduced = points[indexOfPointToBeReduced];

    // get address of point in grid
    int3 gridPos = paramsPointCloud.grid.getCellCoordinate(make_float3(positionOfPointToBeReduced));

    // examine neighbouring cells
    for(int z=-1; z<=1; z++)
    {
        for(int y=-1; y<=1; y++)
        {
            for(int x=-1; x<=1; x++)
            {
                int3 coordinateOfCellToSearch = gridPos + make_int3(x, y, z);
                bool thisPointShouldBeReduced = checkCellForNeighborsD(
                            points,
                            coordinateOfCellToSearch,
                            indexOfPointToBeReduced,
                            positionOfPointToBeReduced,
                            pointCellStart,
                            pointCellStopp);

                if(thisPointShouldBeReduced)
                {
                    points[indexOfPointToBeReduced] = make_float4(0.0);
                    return;
                }
            }
        }
    }
}


void clearClosePointsPreferringSmallerWComponent(
        float*          points,
        unsigned int*   cellStart,
        unsigned int*   cellStopp,
        unsigned int*   gridCellIndex,
        unsigned int*   gridPointIndex,
        unsigned int    offsetOfPointsToBeReduced,
        unsigned int    numberOfPointsToBeReduced,
        unsigned int    numberOfCells)
{
    // thread per particle
    uint numThreads, numBlocks;
    computeExecutionKernelGrid(numberOfPointsToBeReduced, 64, numBlocks, numThreads);

    // execute the kernel
    clearClosePointsPreferringSmallerWComponentD<<< numBlocks, numThreads >>>(
                                                                                (float4*)points,
                                                                                //gridCellIndex,
                                                                                gridPointIndex,
                                                                                cellStart,
                                                                                cellStopp,
                                                                                offsetOfPointsToBeReduced,
                                                                                numberOfPointsToBeReduced
                                                                                //numberOfCells
                                                                                );

    // check if kernel invocation generated an error
    cudaCheckSuccess("markCollidingPoints");
}

*/






// collide a particle against all other particles in a given cell
__device__
bool checkCellForNeighborsBenD(
        int3    gridPos,     // grid cell to search for particles that could collide
        uint    index,       // index of particle that is being collided
        float4  pos,         // position of particle that is being collided
        float4* posSorted,
        uint*   pointCellStart,
        uint*   pointCellStopp)
{
    uint gridHash = paramsPointCloud.grid.getCellHash(gridPos);

    // get start of bucket for this cell
    uint startIndex = pointCellStart[gridHash];

    // cell is not empty
    if(startIndex != 0xffffffff)
    {
        // iterate over particles in this cell
        uint endIndex = pointCellStopp[gridHash];
        for(uint j=startIndex; j<endIndex; j++)
        {
            // check not colliding with self
            if (j != index)
            {
                float4 posOther = posSorted[j];

                float4 relPos = pos - posOther;

                float distSquared = lengthSquared(make_float3(relPos));

                // If they collide AND we're checking the point that was further from the scanner, THEN reduce it!
                if(distSquared < paramsPointCloud.minimumDistance * paramsPointCloud.minimumDistance && pos.w > posOther.w/* && posOther.w != 0.0*/)
                {
                    return true;
                }
            }
        }
    }
    return false;
}


// Collide a single particle (given by thread-id through @index) against all spheres in own and neighboring cells
__global__
void markCollidingPointsD(
        float4* posOriginal,     // output: new positions, same or zeroed. This is actually mDevicePointPos, so its the original position location
        float4* posSorted,          // input: positions sorted according to containing grid cell
        uint*   gridPointIndex,  // input: particle indices sorted according to containing grid cell
        uint*   pointCellStart,       // input: pointCellStart[19] contains the index of gridParticleIndex in which cell 19 starts
        uint*   pointCellStopp,         // input: pointCellStopp[19] contains the index of gridParticleIndex in which cell 19 ends
        uint    numPoints)       // input: number of total particles
{
    uint index = __mul24(blockIdx.x,blockDim.x) + threadIdx.x;
    if(index >= numPoints) return;

    // read particle data from sorted arrays
    float4 pos = posSorted[index];
//    float3 vel = make_float3(FETCH(oldVel, index));

    // get address of particle in grid
    int3 gridPos = paramsPointCloud.grid.getCellCoordinate(make_float3(pos));

    uint originalIndex = gridPointIndex[index];
    // examine neighbouring cells
    for(int z=-1; z<=1; z++)
    {
        for(int y=-1; y<=1; y++)
        {
            for(int x=-1; x<=1; x++)
            {
                int3 neighbourPos = gridPos + make_int3(x, y, z);
                if(
                        checkCellForNeighborsBenD(neighbourPos, index, pos, posSorted, pointCellStart, pointCellStopp)
                        &&
                        originalIndex % 2 == 0)
                {
                    // There is a neighboring point AND this point's index is even. Mark it for removal by zeroing it out!
                    posOriginal[originalIndex] = make_float4(0.0, 0.0, 0.0, 0.0);
                    return;
                }
            }
        }
    }

    // This point does not collide with any other. Do not change its values, it will be kept.
}


void markCollidingPoints(
        float* posOriginal,
        float* posSorted,
        unsigned int*  gridPointIndex,
        unsigned int*  pointCellStart,
        unsigned int*  pointCellStopp,
        unsigned int   numPoints)
{
    if(numPoints == 0) return;

    // thread per particle
    uint numThreads, numBlocks;
    computeExecutionKernelGrid(numPoints, 64, numBlocks, numThreads);

    // execute the kernel
    markCollidingPointsD<<< numBlocks, numThreads >>>(
                                               (float4*)posOriginal,
                                               (float4*)posSorted,
                                               gridPointIndex,
                                               pointCellStart,
                                               pointCellStopp,
                                               numPoints
                                               );

}


/*



// Calculate grid hash value for each particle
__global__
void computeMappingFromGridCellToParticlePcdD(
        uint*   gridParticleHash,  // output
        uint*   gridParticleIndex, // output
        float4* pos,               // input: particle positions
        uint    numParticles)
{
    const unsigned int index = getThreadIndex1D();
    if(index >= numParticles) return;

    volatile float4 p = pos[index];

    // In which grid cell does the particle live?
    int3 gridPos = paramsPointCloud.grid.getCellCoordinate(make_float3(p.x, p.y, p.z));

    // Calculate the particle's hash from the grid-cell. This means particles in the same cell have the same hash
    uint hash = paramsPointCloud.grid.getCellHash(gridPos);

    // This array is the key-part of the map, mapping cellId (=hash) to particleIndex. The term "map" is not
    // exactly correct, because there can be multiple keys (because one cell can store many particles)
    gridParticleHash[index] = hash;

    // It seems stupid to fill an array like "array[x]=x". But this array is the value-part of a map and will get sorted according to the keys (=gridParticleHash)
    gridParticleIndex[index] = index;
}


// Calculates a hash for each particle. The hash value is ("based on") its cell id.
void computeMappingFromPointToGridCellPcd(
        uint*  gridParticleHash,
        uint*  gridParticleIndex,
        float* pos,
        int    numParticles)
{
    if(numParticles == 0) return;

    uint numThreads, numBlocks;
    computeExecutionKernelGrid(numParticles, 64, numBlocks, numThreads);

//    Grid ben;
//    cudaMemcpy(&ben, grid, sizeof(Grid), cudaMemcpyDeviceToHost);
//    qDebug() << "grid on device:"
//             << ben.worldMin.x << ben.worldMin.y << ben.worldMin.z
//             << ben.worldMax.x << ben.worldMax.y << ben.worldMax.z
//             << ben.cells.x << ben.cells.y << ben.cells.z;

    // execute the kernel
    computeMappingFromGridCellToParticlePcdD<<< numBlocks, numThreads >>>(
                                                                         gridParticleHash,
                                                                         gridParticleIndex,
                                                                         (float4 *) pos,
                                                                         numParticles
                                                                         );

    // check if kernel invocation generated an error
    cudaCheckSuccess("computeMappingFromPointToGridCell");
}





*/












// bounding box type
typedef thrust::pair<float4, float4> bbox;

// reduce a pair of bounding boxes (a,b) to a bounding box containing a and b
struct bbox_reduction : public thrust::binary_function<bbox,bbox,bbox>
{
    __host__ __device__
    bbox operator()(bbox a, bbox b)
    {
        // min corner
        float4 min = make_float4(thrust::min(a.first.x, b.first.x), thrust::min(a.first.y, b.first.y), thrust::min(a.first.z, b.first.z), 0);

        // max corner
        float4 max = make_float4(thrust::max(a.second.x, b.second.x), thrust::max(a.second.y, b.second.y), thrust::max(a.second.z, b.second.z), 0);

        return bbox(min, max);
    }
};

// convert a point to a bbox containing that point, (point) -> (point, point)
struct bbox_transformation : public thrust::unary_function<float4,bbox>
{
    __host__ __device__
    bbox operator()(float4 point)
    {
        return bbox(point, point);
    }
};

void getBoundingBox(float *dPoints, uint numPoints, float3& min, float3& max)
{
    float4* points = (float4*)dPoints;

    // wrap raw pointer with a device_ptr
    thrust::device_ptr<float4> dev_ptr = thrust::device_pointer_cast(points);

    bbox init = bbox(dev_ptr[0], dev_ptr[0]);

    // initial bounding box contains first point - does this execute on host? If yes, how can dPoints[0] work?
//    bbox init = bbox(points[0], points[0]);
//    bbox init = bbox(thrust::device_ptr<float4>(dPoints)[0], thrust::device_ptr<float4>(dPoints)[0]);

    // transformation operation
    bbox_transformation opConvertPointToBoundingBox;

    // binary reduction operation
    bbox_reduction opUnifyBoundingBoxes;

    // compute the bounding box for the point set
    bbox result = thrust::transform_reduce(
                thrust::device_ptr<float4>(points),
                thrust::device_ptr<float4>(points + numPoints),
                opConvertPointToBoundingBox,
                init,
                opUnifyBoundingBoxes);

    min = make_float3(result.first);
    max = make_float3(result.second);
}

void sortMapAccordingToKeys(uint *dGridCellIndex, uint *dGridPointIndex, uint numPoints)
{
    if(numPoints == 0) return;

    thrust::sort_by_key(thrust::device_ptr<uint>(dGridCellIndex),                // KeysBeginning
                        thrust::device_ptr<uint>(dGridCellIndex + numPoints),    // KeysEnd
                        thrust::device_ptr<uint>(dGridPointIndex));              // ValuesBeginning

    cudaCheckSuccess("sortMapAccordingToKeys");
}

inline __host__ __device__ bool operator==(float4 a, float4 b)
{
    return
            a.x == b.x &&
            a.y == b.y &&
            a.z == b.z &&
            a.w == b.w;
}

unsigned int removeClearedPoints(float *devicePoints, unsigned int numberOfPoints)
{
    float4* points = (float4*)devicePoints;

    const thrust::device_ptr<float4> newEnd = thrust::remove(
                thrust::device_ptr<float4>(points),
                thrust::device_ptr<float4>(points + numberOfPoints),
                make_float4(0.0f)
                );

    cudaCheckSuccess("removeZeroPoints");

    unsigned int numberOfPointsLeft = newEnd.get() - points;

    return numberOfPointsLeft;
}


// ##########################################
// This is a test to use thrust::unqiue on the point list instead of all the magic we use above
// ##########################################

struct SnapToGridOp
{
    const float3 mMinDist;

    // if params.minimumDistance is e.g. 0.02 = 2cm, factor should be 50.
    // if params.minimumDistance is e.g. 0.10 = 10cm, factor should be 10.
    SnapToGridOp(float3 minDist) : mMinDist(1.0f / minDist) { }

    __host__ __device__
    float4 operator()(float4 a)
    {
        return make_float4(
                    round(a.x * mMinDist.x) / mMinDist.x,
                    round(a.y * mMinDist.y) / mMinDist.y,
                    round(a.z * mMinDist.z) / mMinDist.z,
                    1.0f
                    );
    }

    __host__ __device__
    float round(float r)
    {
        return (r > 0.0f) ? floor(r + 0.5f) : ceil(r - 0.5f);
    }
};

struct CloseToEachOther
{
    float distance;

    __host__ __device__
    CloseToEachOther(float dist = 0.01f) { distance = dist;}

    __host__ __device__
    bool operator()(float4 a, float4 b)
    {
        return (fabs(b.x - a.x) + fabs(b.y - a.y) + fabs(b.z - a.z)) < distance;
    }
};

struct ComparePoints {
    enum SortOrder
    {
        XYZ,
        XZY,
        YXZ,
        YZX,
        ZXY,
        ZYX
    };

    SortOrder sortOrder;

    __host__ __device__
    ComparePoints(SortOrder so)
    {
        sortOrder = so;
    }

    __host__ __device__
    bool operator()(const float4& p1, const float4& p2)
    {
        if(sortOrder == ComparePoints::XYZ)
        {
            if(p1.x < p2.x) return true;
            if(p1.x > p2.x) return false;
            if(p1.y < p2.y) return true;
            if(p1.y > p2.y) return false;
            if(p1.z < p2.z) return true;
            if(p1.z > p2.z) return false;
        }
        if(sortOrder == ComparePoints::XZY)
        {
            if(p1.x < p2.x) return true;
            if(p1.x > p2.x) return false;
            if(p1.z < p2.z) return true;
            if(p1.z > p2.z) return false;
            if(p1.y < p2.y) return true;
            if(p1.y > p2.y) return false;
        }
        if(sortOrder == ComparePoints::YXZ)
        {
            if(p1.y < p2.y) return true;
            if(p1.y > p2.y) return false;
            if(p1.x < p2.x) return true;
            if(p1.x > p2.x) return false;
            if(p1.z < p2.z) return true;
            if(p1.z > p2.z) return false;
        }

        if(sortOrder == ComparePoints::YZX)
        {
            if(p1.y < p2.y) return true;
            if(p1.y > p2.y) return false;
            if(p1.z < p2.z) return true;
            if(p1.z > p2.z) return false;
            if(p1.x < p2.x) return true;
            if(p1.x > p2.x) return false;
        }

        if(sortOrder == ComparePoints::ZXY)
        {
            if(p1.z < p2.z) return true;
            if(p1.z > p2.z) return false;
            if(p1.x < p2.x) return true;
            if(p1.x > p2.x) return false;
            if(p1.y < p2.y) return true;
            if(p1.y > p2.y) return false;
        }

        if(sortOrder == ComparePoints::ZYX)
        {
            if(p1.z < p2.z) return true;
            if(p1.z > p2.z) return false;
            if(p1.y < p2.y) return true;
            if(p1.y > p2.y) return false;
            if(p1.x < p2.x) return true;
            if(p1.x > p2.x) return false;
        }

        return false; // when points are completely equal.
    }
};

unsigned int snapToGridAndMakeUnique(float *devicePoints, unsigned int numPoints, float minimumDistance)
{
    float4* points = (float4*)devicePoints;

    thrust::device_ptr<float4> begin = thrust::device_ptr<float4>(points);
    thrust::device_ptr<float4> end = thrust::device_ptr<float4>(points + numPoints);

    thrust::transform(
                begin,
                end,
                begin,
                SnapToGridOp(make_float3(minimumDistance, minimumDistance, minimumDistance))
                );

    cudaCheckSuccess("snapToGridAndMakeUnique0");



    thrust::sort(begin, end, ComparePoints(ComparePoints::XYZ));
    end = thrust::unique(begin, end, CloseToEachOther());

    cudaCheckSuccess("snapToGridAndMakeUnique1");

    thrust::sort(begin, end, ComparePoints(ComparePoints::XZY));
    end = thrust::unique(begin, end, CloseToEachOther());

    cudaCheckSuccess("snapToGridAndMakeUnique2");

    thrust::sort(begin, end, ComparePoints(ComparePoints::YXZ));
    end = thrust::unique(begin, end, CloseToEachOther());

    cudaCheckSuccess("snapToGridAndMakeUnique3");

    thrust::sort(begin, end, ComparePoints(ComparePoints::YZX));
    end = thrust::unique(begin, end, CloseToEachOther());

    cudaCheckSuccess("snapToGridAndMakeUnique4");

    thrust::sort(begin, end, ComparePoints(ComparePoints::ZXY));
    end = thrust::unique(begin, end, CloseToEachOther());

    cudaCheckSuccess("snapToGridAndMakeUnique5");

    thrust::sort(begin, end, ComparePoints(ComparePoints::ZYX));
    end = thrust::unique(begin, end, CloseToEachOther());

    cudaCheckSuccess("snapToGridAndMakeUnique");

    return end.get() - points;
}

/*
__global__ void replaceCellPointsByMeanValueD(
        float4* devicePoints,
        float4* devicePointsSorted,
        uint* pointCellStart,
        uint* pointCellStopp,
        uint* gridCellIndex,
        uint* gridPointIndex,
        uint numPoints,
        uint numCells)
{
    uint cellIndex = __mul24(blockIdx.x,blockDim.x) + threadIdx.x;

    // get start of bucket for this cell
    uint startIndex = pointCellStart[cellIndex];

    float4 pointAverage = make_float4(0.0f);

    // cell is not empty
    if(startIndex != 0xffffffff)
    {
        // iterate over particles in this cell
        uint endIndex = pointCellStopp[cellIndex];
        for(uint j=startIndex; j<endIndex; j++)
        {
            pointAverage += devicePointsSorted[j];
        }
        pointAverage /= endIndex-startIndex;
    }

    devicePoints[cellIndex] = pointAverage;
}


unsigned int replaceCellPointsByMeanValue(float *devicePoints, float* devicePointsSorted, unsigned int *pointCellStart, unsigned int *pointCellStopp, unsigned int *gridCellIndex, unsigned int *gridPointIndex, unsigned int numPoints, unsigned int numCells)
{
    // The points are already sorted according to the containing cell, cellStart and cellStopp are up to date
    // Start a thread for every CELL, reading all its points, creating a mean-value. Write this value into devicePoints[threadid]

    uint numThreads, numBlocks;
    computeExecutionKernelGrid(numCells, 256, numBlocks, numThreads);

    qDebug() << "replaceCellPointsByMeanValue(): using" << numCells << "threads to create as many averaged points";

    replaceCellPointsByMeanValueD<<< numBlocks, numThreads>>>(
                                                                (float4*)devicePoints,
                                                                (float4*)devicePointsSorted,
                                                                pointCellStart,
                                                                pointCellStopp,
                                                                gridCellIndex,
                                                                gridPointIndex,
                                                                numPoints,
                                                                numCells);

    cudaCheckSuccess("replaceCellPointsByMeanValue");

    // Every cell created a point, empty cells create zero-points. Delete those.
    return removeZeroPoints(devicePoints, numCells);
}
*/

struct IsOutsideBoundingBoxOp
{
    const float3 mBBoxMin, mBBoxMax;

    IsOutsideBoundingBoxOp(const float3& boxMin, const float3& boxMax) :
        mBBoxMin(boxMin),
        mBBoxMax(boxMax)
    { }

    __host__ __device__
    bool operator()(const float4 point)
    {
        return
                mBBoxMin.x > point.x ||
                mBBoxMin.y > point.y ||
                mBBoxMin.z > point.z ||
                mBBoxMax.x < point.x ||
                mBBoxMax.y < point.y ||
                mBBoxMax.z < point.z;
    }
};

unsigned int clearPointsOutsideBoundingBox(float* points, unsigned int numberOfPoints, ParametersPointCloud* params)
{
    // move all points in bbox to beginning of devicePointsBase and return number of points left
    IsOutsideBoundingBoxOp op(params->grid.worldMin, params->grid.worldMax);

    float4* pointsf4 = (float4*)points;

    const thrust::device_ptr<float4> newEnd = thrust::remove_if(
                thrust::device_ptr<float4>(pointsf4),
                thrust::device_ptr<float4>(pointsf4 + numberOfPoints),
                op);

    cudaCheckSuccess("clearPointsOutsideBoundingBox");

    unsigned int numberOfPointsRemaining = newEnd.get() - pointsf4;

    return numberOfPointsRemaining;
}


unsigned int copyPoints(float* devicePointsBaseDst, float* devicePointsBaseSrc, unsigned int numberOfPointsToCopy)
{
    float4* pointsSrc = (float4*)devicePointsBaseSrc;
    float4* pointsDst = (float4*)devicePointsBaseDst;

    const thrust::device_ptr<float4> newEnd = thrust::copy(
                thrust::device_ptr<float4>(pointsSrc),
                thrust::device_ptr<float4>(pointsSrc + numberOfPointsToCopy),
                thrust::device_ptr<float4>(pointsDst));

    cudaCheckSuccess("copyPoints");

    const unsigned int numberOfPointsCopied = newEnd.get() - pointsDst;
    return numberOfPointsCopied;
}


struct IsInsideBoundingBoxOp
{
    const float3 mBBoxMin, mBBoxMax;

    IsInsideBoundingBoxOp(const float3& boxMin, const float3& boxMax) :
        mBBoxMin(boxMin),
        mBBoxMax(boxMax)
    { }

    __host__ __device__
    bool operator()(const float4 point)
    {
        return
                mBBoxMin.x < point.x &&
                mBBoxMin.y < point.y &&
                mBBoxMin.z < point.z &&
                mBBoxMax.x > point.x &&
                mBBoxMax.y > point.y &&
                mBBoxMax.z > point.z;
    }
};


unsigned int copyPointsInBoundingBox(float* devicePointsBaseDst, float* devicePointsBaseSrc, float3 &bBoxMin, float3 &bBoxMax, unsigned int numberOfPointsToCopy)
{
    float4* pointsSrc = (float4*)devicePointsBaseSrc;
    float4* pointsDst = (float4*)devicePointsBaseDst;

    IsInsideBoundingBoxOp op(bBoxMin, bBoxMax);

    const thrust::device_ptr<float4> newEnd = thrust::copy_if(
                thrust::device_ptr<float4>(pointsSrc),
                thrust::device_ptr<float4>(pointsSrc + numberOfPointsToCopy),
                thrust::device_ptr<float4>(pointsDst),
                op);

    cudaCheckSuccess("copyPointsInBoundingBox");

    const unsigned int numberOfPointsCopied = newEnd.get() - pointsDst;
    return numberOfPointsCopied;
}
























struct ComparisonOperatorPointCellHash {

    Grid mGrid;

    __host__ __device__
    ComparisonOperatorPointCellHash(Grid grid)
    {
        mGrid = grid;
    }

    __host__ __device__
    bool operator()(const float4& p1, const float4& p2)
    {
        int3 gc1 = mGrid.getCellCoordinate(make_float3(p1));
        int3 gc2 = mGrid.getCellCoordinate(make_float3(p2));

        if(mGrid.getCellHash(gc1) < mGrid.getCellHash(gc2))
            return true;
        else
            return false;
    }
};
/*
void sortPointsToGridCellOrder(float* devicePoints, const Grid& grid, unsigned int numberOfPoints)
{
    float4* points = (float4*)devicePoints;

    ComparisonOperatorPointCellHash op(grid);

    thrust::sort(
                thrust::device_ptr<float4>(points),
                thrust::device_ptr<float4>(points + numberOfPoints),
                op);
}
*/
/*
void buildGridOccupancyMap(float* devicePoints, unsigned int* mDeviceMapGridCell, unsigned int numberOfPoints)
{
    // Launch one kernel per point, but have all kernels for the 32 bits of an unsigned int in a warp.
    // This way, they can quickly write to one unsigned integer value.

    uint numThreads, numBlocks;
    computeExecutionKernelGrid(numberOfPoints, 256, numBlocks, numThreads);

    cudaCheckSuccess("buildGridOccupancyMapD");
}*/

//unsigned int convertOccupiedCellsToPoints(float* devicePoints, unsigned int* mDeviceMapGridCell, unsigned int numberOfPoints)
//{
//return 0;
//}
