#ifndef CUDA_H
#define CUDA_H

// Fix for CUDA and gcc 4.7
#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include  <QVector3D>

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
//    static uint iDivUp(uint a, uint b);

    static QVector3D cudaConvert(const float3& p);
    static float3 cudaConvert(const QVector3D& p);
};

#endif
