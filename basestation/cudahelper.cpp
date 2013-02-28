#include "cudahelper.h"

void *mapGLBufferObject(struct cudaGraphicsResource **cuda_vbo_resource)
{
    void *ptr;
    cudaSafeCall(cudaGraphicsMapResources(1, cuda_vbo_resource, 0));
    size_t num_bytes;
    cudaSafeCall(cudaGraphicsResourceGetMappedPointer((void **)&ptr, &num_bytes, *cuda_vbo_resource));
    return ptr;
}

void copyArrayFromDevice(void* host, const void* device, struct cudaGraphicsResource **cuda_vbo_resource, int size)
{
    if(cuda_vbo_resource) device = mapGLBufferObject(cuda_vbo_resource);

    cudaSafeCall(cudaMemcpy(host, device, size, cudaMemcpyDeviceToHost));

    if(cuda_vbo_resource) cudaSafeCall(cudaGraphicsUnmapResources(1, cuda_vbo_resource, 0));
}

// Round a / b to nearest higher integer value
uint iDivUp(uint a, uint b)
{
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

// Compute grid and thread block size for a given number of elements
void computeExecutionKernelGrid(uint n, uint blockSize, uint &numBlocks, uint &numThreads)
{
    numThreads = std::min(blockSize, n);
    numBlocks = iDivUp(n, numThreads);
}

QVector3D cudaConvert(const float3& p) { return QVector3D(p.x, p.y, p.z); }
float3 cudaConvert(const QVector3D& p) { return make_float3(p.x(), p.y(), p.z()); }
