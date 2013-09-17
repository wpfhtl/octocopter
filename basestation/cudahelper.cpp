#include "cudahelper.h"
#include "cudahelper.cuh"
#include <QDebug>

bool CudaHelper::isDeviceSupported = false;

bool CudaHelper::initializeCuda()
{
    if(isDeviceSupported) return true; // already initialized

    // Initialize CUDA
    int numberOfCudaDevices;
    cudaSafeCall(cudaGetDeviceCount(&numberOfCudaDevices));

    if(numberOfCudaDevices < 1)
    {
        qDebug() << "CudaHelper::initializeCuda(): No CUDA devices found!";
        return false;
    }

    int activeCudaDevice;
    cudaSafeCall(cudaGetDevice(&activeCudaDevice));

    // Necessary for OpenGL graphics interop: GlWidget and CUDA-based FlightPlanners have a close relationship because cudaGlSetGlDevice() needs to be called in GL context and before any other CUDA calls.
    cudaSafeCall(cudaGLSetGLDevice(activeCudaDevice));

    cudaSafeCall(cudaSetDeviceFlags(cudaDeviceMapHost));// in order for the cudaHostAllocMapped flag to have any effect

    cudaDeviceProp deviceProps;
    cudaGetDeviceProperties(&deviceProps, activeCudaDevice);

    size_t memTotal, memFree;
    cudaMemGetInfo(&memFree, &memTotal);

    qDebug() << "CudaHelper::initializeCuda(): device" << deviceProps.name << "has compute capability" << deviceProps.major << deviceProps.minor << "and"
             << memFree / 1048576 << "of" << memTotal / 1048576 << "mb free, has"
             << deviceProps.multiProcessorCount << "multiprocessors,"
             << (deviceProps.integrated ? "is" : "is NOT" ) << "integrated,"
             << (deviceProps.canMapHostMemory ? "can" : "can NOT") << "map host mem, has"
             << deviceProps.memoryClockRate / 1000 << "Mhz mem clock, a"
             << deviceProps.memoryBusWidth << "bit mem bus and max"
             << deviceProps.maxTexture1DLinear / 1048576 << "mb of 1d texture bound to linear memory.";

    // We need compute capability 3.0 and some free memory
    if(deviceProps.major >= 3 && memFree/1048576 > 512)
    {
        CudaHelper::isDeviceSupported = true;
        qDebug() << "CudaHelper::initializeCuda(): device is supported!";
        return true;
    }
    else
    {
        CudaHelper::isDeviceSupported = false;
        qDebug() << "CudaHelper::initializeCuda(): device is NOT supported!";
        return false;
    }
}

void* CudaHelper::mapGLBufferObject(struct cudaGraphicsResource **cuda_vbo_resource)
{
    void *ptr;
    cudaSafeCall(cudaGraphicsMapResources(1, cuda_vbo_resource, 0));
    size_t num_bytes;
    cudaSafeCall(cudaGraphicsResourceGetMappedPointer((void **)&ptr, &num_bytes, *cuda_vbo_resource));
    return ptr;
}

void CudaHelper::copyArrayFromDevice(void* host, const void* device, struct cudaGraphicsResource **cuda_vbo_resource, int size)
{
    if(cuda_vbo_resource) device = mapGLBufferObject(cuda_vbo_resource);

    cudaSafeCall(cudaMemcpy(host, device, size, cudaMemcpyDeviceToHost));

    if(cuda_vbo_resource) cudaSafeCall(cudaGraphicsUnmapResources(1, cuda_vbo_resource, 0));
}

QVector3D CudaHelper::convert(const float3& p) { return QVector3D(p.x, p.y, p.z); }
float3 CudaHelper::convert(const QVector3D& p) { return make_float3(p.x(), p.y(), p.z()); }
