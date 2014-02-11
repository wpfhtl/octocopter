// Fix for gcc 4.7
//#undef _GLIBCXX_ATOMIC_BUILTINS
//#undef _GLIBCXX_USE_INT128

#include "thrust/sort.h"
#include "thrust/unique.h"
#include <thrust/remove.h>
#include <thrust/count.h>
#include <thrust/device_ptr.h>

#include "cuda.h"
#include "cudahelper.cuh"
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

// Returns the presence of neighbors of @pos within paramsPointCloud.minimumDistance in @gridCell
// Caller must ensure:
//  - pos is in grid
//  - gridCell is valid (less than grid.cells)
__device__ void checkCellForNeighbors(
        int3    gridCell,       // grid cell to search for particles that could collide
        uint    index,          // index of particle that is being collided
        float4  pos,            // position of particle that is being collided
        float4* posSorted,
        uint*   pointCellStart,
        uint*   pointCellStopp,
        float4& clusterPosition,
        uint&   numberofNeighborsFound)
{
    const uint gridHash = paramsPointCloud.grid.getCellHash(gridCell);

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
            if(j != index)
            {
                const float4 posOther = posSorted[j];
                const float4 relPos = pos - posOther;
                float distSquared = lengthSquared(make_float3(relPos));

                // If they collide AND we're checking the point that was further from the scanner, THEN reduce it!
                if(distSquared < paramsPointCloud.minimumDistance * paramsPointCloud.minimumDistance)
                {
                    clusterPosition += posOther;
                    numberofNeighborsFound ++;
                }
            }
        }
    }
}

// not really random, but should be good enough.
__device__ float randomNumber(uint seed)
{
    uint a = threadIdx.x * seed;
    uint b = blockIdx.x * blockDim.x;

    b = 36969 * (b & 65535) + (b >> 16);
    a = 18000 * (a & 65535) + (a >> 16);

    return ((b << 16) + a) / 4294967295.0;
}

// Collide a single point (given by thread-id through @index) against all points in own and neighboring cells
__global__
void markCollidingPointsD(
        float4* posOriginal,        // output: new positions, same or zeroed. This is actually mDevicePointPos, so its the original position location
        float4* positionsSorted,    // input: positions sorted according to containing grid cell
        uint*   gridPointIndex,     // input: particle indices sorted according to containing grid cell
        uint*   pointCellStart,     // input: pointCellStart[19] contains the index of gridParticleIndex in which cell 19 starts
        uint*   pointCellStopp,     // input: pointCellStopp[19] contains the index of gridParticleIndex in which cell 19 ends
        uint    numPoints)          // input: number of total particles
{
    uint threadIndex = getThreadIndex1D();
    if(threadIndex >= numPoints) return;

    // read particle data from sorted arrays
    const float4 worldPos = positionsSorted[threadIndex];

    // get address of particle in grid
    const int3 gridCellCoordinate = paramsPointCloud.grid.getCellCoordinate(make_float3(worldPos));

    // Do not process points that are not in the defined grid!
    if(gridCellCoordinate.x == -1)
    {
        printf("got a point not in grid, ouch!\n");
        return;
    }

    const uint originalIndex = gridPointIndex[threadIndex];

    float4 clusterPosition = make_float4(0.0);
    unsigned int numberOfCollisionsInOwnCell = 0;
    unsigned int numberOfCollisionsInNeighborCells = 0;

    // This code tries to optimize, thinking: If we already find many neighbors in our own cell, there's really not much
    // use in looking in other cells, too. We could even go so far as to test for the minimumDistance vs cellSize: If the
    // mindist is 20 cm, the cell contains 100 points and the cellsize is 20cm, then we know we have lots of neighbors
    // without even looking at them. But we don't know their .w component values!
    checkCellForNeighbors(
                gridCellCoordinate,
                threadIndex,
                worldPos,
                positionsSorted,
                pointCellStart,
                pointCellStopp,
                clusterPosition,
                numberOfCollisionsInOwnCell);

    // examine neighbouring cells
    for(int z=-1; z<=1 && numberOfCollisionsInOwnCell < 5; z++)
    {
        for(int y=-1; y<=1; y++)
        {
            for(int x=-1; x<=1; x++)
            {
                const int3 neighbourGridCoordinate = gridCellCoordinate + make_int3(x, y, z);
                if(x == 0 && y == 0 && z == 0) continue;

                checkCellForNeighbors(
                            neighbourGridCoordinate,
                            threadIndex,
                            worldPos,
                            positionsSorted,
                            pointCellStart,
                            pointCellStopp,
                            clusterPosition,
                            numberOfCollisionsInNeighborCells);
            }
        }
    }

    const float numberOfCollisionsTotal = numberOfCollisionsInOwnCell + numberOfCollisionsInNeighborCells;
    if(numberOfCollisionsTotal > 0.0)
    {
        const float averageNeighborScanDistance = clusterPosition.w / numberOfCollisionsTotal;
        if(averageNeighborScanDistance > worldPos.w /*&& randomNumber(numPoints + threadIndex) * numberOfCollisionsTotal > 1.0*/)
        {
            // If the other neighbors are of better quality, delete ourselves
            posOriginal[originalIndex] = make_float4(0.0);
        }
        else
        {
            // If we're better, move us into the center of our neighborhood
            posOriginal[originalIndex] = clusterPosition / numberOfCollisionsTotal;
        }
    }

    //posOriginal[originalIndex] = make_float4(worldPos.x, worldPos.y, worldPos.z, numberOfCollisionsTotal);
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
    computeExecutionKernelGrid(numPoints, 128, numBlocks, numThreads);

    //std::cout << "markCollidingPoints(): we have " << numPoints << " points, " << numThreads << " threads and " << numBlocks << " blocks" << std::endl;

    // execute the kernel
    // TODO: test optimization: Write back not into posOriginal, but into posSorted (should be faster), then memcpy posSorted into posOriginal.
    // If writing back into posSorted is faster, we should be able to just switch buffers after every reduction.
    markCollidingPointsD<<< numBlocks, numThreads >>>(
                                               (float4*)posOriginal,
                                               (float4*)posSorted,
                                               gridPointIndex,
                                               pointCellStart,
                                               pointCellStopp,
                                               numPoints
                                               );

    cudaDeviceSynchronize();

    cudaCheckSuccess("markCollidingPoints");
}

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

    thrust::device_ptr<float4> newEnd;

    try
    {
        // Just for debugging!
        int result = thrust::count(thrust::device_ptr<float4>(points),
                                   thrust::device_ptr<float4>(points + numberOfPoints),
                                   make_float4(0.0f));
        std::cerr << __PRETTY_FUNCTION__ << " removing zero points: " << result << " of " << numberOfPoints << std::endl;

        newEnd = thrust::remove(
                    thrust::device_ptr<float4>(points),
                    thrust::device_ptr<float4>(points + numberOfPoints),
                    make_float4(0.0f)
                    );
    }
    catch(thrust::system_error &e)
    {
      // output an error message and exit
      std::cerr << "Error accessing vector element: " << e.what() << std::endl;
      exit(-1);
    }

    cudaCheckSuccess("removeZeroPoints");

    unsigned int numberOfPointsLeft = newEnd.get() - points;

    return numberOfPointsLeft;
}

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

unsigned int removePointsOutsideBoundingBox(float* points, unsigned int numberOfPoints, Grid* grid)
{
    printf("removePointsOutsideBoundingBox(): clearing %d points outside %.2f %.2f %.2f and %.2f %.2f %.2f\n", numberOfPoints, grid->worldMin.x, grid->worldMin.y, grid->worldMin.z, grid->worldMax.x, grid->worldMax.y, grid->worldMax.z);
    // move all points in bbox to beginning of devicePointsBase and return number of points left
    IsOutsideBoundingBoxOp op(grid->worldMin, grid->worldMax);

    float4* pointsf4 = (float4*)points;

    const thrust::device_ptr<float4> newEnd = thrust::remove_if(
                thrust::device_ptr<float4>(pointsf4),
                thrust::device_ptr<float4>(pointsf4 + numberOfPoints),
                op);

    cudaCheckSuccess("removePointsOutsideBoundingBox");

    unsigned int numberOfPointsRemaining = newEnd.get() - pointsf4;

    printf("removePointsOutsideBoundingBox(): done.\n");

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

// Requires dst and src to live in device memory space
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
