#include "cudahelper.h"

bool CudaHelper::mDeviceSupported = false;

bool CudaHelper::initializeCuda()
{
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
        CudaHelper::mDeviceSupported = true;
        qDebug() << "CudaHelper::initializeCuda(): device is supported!";
        return true;
    }
    else
    {
        CudaHelper::mDeviceSupported = false;
        qDebug() << "CudaHelper::initializeCuda(): device is NOT supported!";
        return false;
    }
}

bool CudaHelper::isDeviceSupported()
{
    return CudaHelper::mDeviceSupported;
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

// Round a / b to nearest higher integer value
uint CudaHelper::iDivUp(uint a, uint b)
{
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

// Compute grid and thread block size for a given number of elements
void CudaHelper::computeExecutionKernelGrid(uint n, uint blockSize, uint &numBlocks, uint &numThreads)
{
    numThreads = std::min(blockSize, n);
    numBlocks = iDivUp(n, numThreads);
}

QVector3D CudaHelper::cudaConvert(const float3& p) { return QVector3D(p.x, p.y, p.z); }
float3 CudaHelper::cudaConvert(const QVector3D& p) { return make_float3(p.x(), p.y(), p.z()); }
