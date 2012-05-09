// Fix for gcc 4.7
#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#include <cuda_runtime.h>
#include <QDebug>
#include <QVector3D>

// Kernel definition
extern "C" void computeFreeColumns(
    unsigned char* hostGridPointer,
    unsigned char* pixmap,
    float4* vertexPositions,
    int resX, int resY, int resZ,
    const QVector3D& boundingBoxMin,
    const QVector3D& boundingBoxMax);

__global__ void kernelComputeFreeColumns(
    unsigned char* gridPointer,
    unsigned char* pixmap,
    float4* vertexPositions,
    int resX,
    int resY,
    int resZ,
    int minX,
    int minY,
    int minZ,
    int maxX,
    int maxY,
    int maxZ)
{
    int i = threadIdx.x;
    int j = threadIdx.y;

    //pixmap[i] = i*j;//gridPointer[i];
    //pixmap[i] = i*j;//gridPointer[i];

    i = blockDim.x * blockIdx.x + threadIdx.x;

    //gridPointer[i] = i;
    pixmap[i] = gridPointer[i];
    vertexPositions[i] = make_float4(i*15,i*15,threadIdx.x,80);
}















void computeFreeColumns(
    unsigned char* hostGridPointer,
    unsigned char* pixmap,
    float4* vertexPositions,
    int resX, int resY, int resZ,
    const QVector3D& boundingBoxMin,
    const QVector3D& boundingBoxMax)
{
    cudaError_t mCudaError;

    //    int numBlocks = 1;
    //    dim3 threadsPerBlock(N, N);
    //    MatAdd<<<numBlocks, threadsPerBlock>>>(A, B, C);

    // gridPointer points to the host's grid. We need the pointer to the device's version of that data.
    quint8* deviceVolumeDataBasePointer = 0;
    mCudaError = cudaHostGetDevicePointer(&deviceVolumeDataBasePointer, hostGridPointer, 0);
    if(deviceVolumeDataBasePointer == 0) qDebug("couldn't get device pointer for volume data");
    if(mCudaError != cudaSuccess) qDebug("couldn't get device pointer for volume data: %s", cudaGetErrorString(mCudaError));

    unsigned char *pixmapDevice;
    cudaMalloc((void**)&pixmapDevice, resX*resZ);

    qDebug("executing kernel!");
    kernelComputeFreeColumns<<<64, 64>>>(
                                           deviceVolumeDataBasePointer,
                                           pixmapDevice,
                                           vertexPositions,
                                           resX,
                                           resY,
                                           resZ,
                                           boundingBoxMin.x(), boundingBoxMin.y(), boundingBoxMin.z(),
                                           boundingBoxMax.x(), boundingBoxMax.y(), boundingBoxMax.z());

    cudaMemcpy(pixmap, pixmapDevice, resX*resZ, cudaMemcpyDeviceToHost);

    cudaDeviceSynchronize();
    cudaThreadSynchronize();

    qDebug() << "grid:";
    for(int i=0;i<(resX*resY*resZ)/8;i++) printf("%d ", hostGridPointer[i]);
    fflush(stdout);

    qDebug() << "\npixmap:";
    for(int i=0;i<resX*resZ;i++) printf("%d ", pixmap[i]);
    fflush(stdout);

    cudaFree(pixmapDevice);
}



