#include "cuda.h"

void checkCudaSuccess(const char *errorMessage)
{
  cudaError_t err = cudaGetLastError();
  if( cudaSuccess != err) {
    printf("CUDA error %s: %s.\n", errorMessage, cudaGetErrorString(err) );
    exit(-1);
  }
}

void *mapGLBufferObject(struct cudaGraphicsResource **cuda_vbo_resource)
{
    void *ptr;
    cudaGraphicsMapResources(1, cuda_vbo_resource, 0);
    size_t num_bytes;
    cudaGraphicsResourceGetMappedPointer((void **)&ptr, &num_bytes, *cuda_vbo_resource);
    return ptr;
}

void copyArrayFromDevice(void* host, const void* device, struct cudaGraphicsResource **cuda_vbo_resource, int size)
{
    if (cuda_vbo_resource) device = mapGLBufferObject(cuda_vbo_resource);

    cudaMemcpy(host, device, size, cudaMemcpyDeviceToHost);

    if (cuda_vbo_resource)
    {
        //unmapGLBufferObject(*cuda_vbo_resource);
        cudaGraphicsUnmapResources(1, cuda_vbo_resource, 0);
    }
}

//Round a / b to nearest higher integer value
uint iDivUp(uint a, uint b)
{
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

// compute grid and thread block size for a given number of elements
void computeGridSize(uint n, uint blockSize, uint &numBlocks, uint &numThreads)
{
    numThreads = std::min(blockSize, n);
    numBlocks = iDivUp(n, numThreads);
}
