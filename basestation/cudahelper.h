#ifndef CUDA_H
#define CUDA_H

// Fix for CUDA and gcc 4.7
#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
//#include  <QVector3D>
#include  <iostream>


#define CUDA_ERROR_CHECK
#define cudaSafeCall(err) __cudaSafeCall( err, __FILE__, __LINE__ )
#define cudaCheckSuccess(src)  __cudaCheckSuccess( src, __FILE__, __LINE__ )

inline void __cudaSafeCall(cudaError err, const char *file, const int line)
{
#ifdef CUDA_ERROR_CHECK
    if(cudaSuccess != err)
    {
        cout << "cudaSafeCall(): failed at " << file << ":" << line << ": " << cudaGetErrorString(err);
        fflush(stdout);
        abort();
    }
#endif
    return;
}

inline void __cudaCheckSuccess(const char *errorSource, const char *file, const int line)
{
#ifdef CUDA_ERROR_CHECK
    cudaError err = cudaGetLastError();
    if ( cudaSuccess != err )
    {
        cout << "cudaCheckError(): " << errorSource << " failed at " << file << ":" << line << ": " << cudaGetErrorString(err);
        fflush(stdout);
        abort();
    }

    // More careful checking, using sync (will slow down performance)
    err = cudaDeviceSynchronize();
    if(cudaSuccess != err)
    {
        cout << "cudaCheckError(): with sync failed at " << file << ":" << line << ": " << cudaGetErrorString(err);
        fflush(stdout);
        abort();
    }
#endif

    return;
}

class CudaHelper
{
private:
    CudaHelper() {} // We don't want to have this instantiated

public:
    static bool isDeviceSupported;
    static bool initializeCuda();

    static void *mapGLBufferObject(struct cudaGraphicsResource **cuda_vbo_resource);

    static void copyArrayFromDevice(void* host, const void* device, struct cudaGraphicsResource **cuda_vbo_resource, int size);

    //Round a / b to nearest higher integer value
    static uint iDivUp(uint a, uint b);

    // compute grid and thread block size for a given number of elements
    static void computeExecutionKernelGrid(uint n, uint blockSize, uint &numBlocks, uint &numThreads);

//    static QVector3D cudaConvert(const float3& p);
//    static float3 cudaConvert(const QVector3D& p);
};

#endif
