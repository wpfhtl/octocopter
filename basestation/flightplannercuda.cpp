#include "flightplannercuda.h"

FlightPlannerCuda::FlightPlannerCuda(QWidget* widget, Octree* pointCloud) : FlightPlannerInterface(widget, pointCloud)
{
    // Initialize CUDA
    int numberOfCudaDevices;
    cudaGetDeviceCount(&numberOfCudaDevices);
    Q_ASSERT(numberOfCudaDevices && "FlightPlannerCuda::FlightPlannerCuda(): No CUDA devices found, exiting.");

    int activeCudaDevice;
    cudaGetDevice(&activeCudaDevice);

    cudaDeviceProp deviceProps;
    cudaGetDeviceProperties(&deviceProps, activeCudaDevice);

    size_t memTotal, memFree;
    cudaMemGetInfo(&memFree, &memTotal);

    qDebug() << "FlightPlannerCuda::FlightPlannerCuda(): device has"
             << deviceProps.totalGlobalMem << "bytes mem,"
             << deviceProps.multiProcessorCount << "multiprocessors,"
             << "is integrated:" << deviceProps.integrated
             << "can map host memory:" << deviceProps.canMapHostMemory
             << ", has" << deviceProps.memoryClockRate << "hz memory clock"
             << deviceProps.memoryBusWidth << "memory bus width and"
             << memFree / 1000000 << "of" << memTotal << "mb memory free";

    cudaSetDeviceFlags(cudaDeviceMapHost);// in order for the cudaHostAllocMapped flag to have any effect

    mVoxelManager = new VoxelManager(256, 16, 256);

    mHostColumnOccupancyPixmapData = new unsigned char[mVoxelManager->getGroundPlanePixelCount()];

    // Allocate memory for volume data on device
    cudaMalloc((void**)&mDeviceVolumeData, mVoxelManager->getVolumeDataSize());
    cudaMalloc((void**)&mDeviceColumnOccupancyPixmapData, mVoxelManager->getGroundPlanePixelCount());

    cudaMemGetInfo(&memFree, &memTotal);

    qDebug() << "FlightPlannerCuda::FlightPlannerCuda(): after allocating memory, device has" << memFree / 1000000 << "of" << memTotal << "mb memory free";

    // Create the GL buffer for vertices with a color (3 floats = 12 bytes and 4 bytes rgba color)
//    glGenBuffers(1,&mVertexArray);
//    glBindBuffer(GL_ARRAY_BUFFER, mVertexArray);
//    glBufferData(GL_ARRAY_BUFFER, mNumVoxels * 16, NULL, GL_DYNAMIC_COPY);
//    cudaGLRegisterBufferObject(mVertexArray);
}

void FlightPlannerCuda::slotCreateSafePathToNextWayPoint()
{

}

FlightPlannerCuda::~FlightPlannerCuda()
{
    delete mVoxelManager;

    // Shutdown CUDA
    cudaFree(mDeviceVolumeData);
}

void FlightPlannerCuda::insertPoint(LidarPoint* const point)
{
    mVoxelManager->setVoxelValue(point->position, true);
}

void FlightPlannerCuda::slotGenerateWaypoints()
{
    cudaMemcpyAsync(mDeviceVolumeData, mVoxelManager->getBasePointer(), mVoxelManager->getVolumeDataSize(), cudaMemcpyHostToDevice, 0);

    // Start kernel to check reachability, return pixmap
    //multiplyNumbersGPU<<<blockGridRows, threadBlockRows>>>(d_dataA, d_dataB, d_resultC);
    computeFreeColumns(mDeviceVolumeData, mHostColumnOccupancyPixmapData);

//    int numBlocks = 1;
//    dim3 threadsPerBlock(N, N);
//    MatAdd<<<numBlocks, threadsPerBlock>>>(A, B, C);

    // Now read the mem and show the picture!

}

// NOT in a glBegin()/glEnd() pair.
void FlightPlannerCuda::slotVisualize() const
{
    // Use CUDA?!
//    void* vertexPointer;
    // Map the buffer to CUDA
//    cudaGLMapBufferObject(&vertexPointer, mVertexArray);
    // Run a kernel to create/manipulate the data
//    MakeVerticiesKernel<<<gridSz,blockSz>>>(vertexPointer, mNumVoxels);
    // Unmap the buffer
//    cudaGLUnmapbufferObject(mVertexArray);


    // Bind the Buffer
//    glBindBuffer( GL_ARRAY_BUFFER, mVertexArray );
    // Enable Vertex and Color arrays
//    glEnableClientState(GL_VERTEX_ARRAY);
//    glEnableClientState(GL_COLOR_ARRAY);
    // Set the pointers to the vertices and colors
//    glVertexPointer(3,GL_FLOAT,16,0);
//    glColorPointer(4,GL_UNSIGNED_BYTE,16,12);

//    glDrawArrays(GL_POINTS,0, mNumVoxels);
}

void FlightPlannerCuda::slotSetScanVolume(const QVector3D min, const QVector3D max)
{
    FlightPlannerInterface::slotSetScanVolume(min, max);
    mVoxelManager->slotSetScanVolume(min, max);

    // TODO: re-fill VoxelManager's data from basestations octree.
}

void FlightPlannerCuda::slotProcessPhysics(bool process)
{
}

void FlightPlannerCuda::slotSubmitGeneratedWayPoints()
{
    mWayPointsAhead->append(mWayPointsGenerated);
    mWayPointsGenerated.clear();
    sortToShortestPath(*mWayPointsAhead, mVehiclePoses.last().position);
    emit wayPointsSetOnRover(*mWayPointsAhead);
    emit wayPoints(*mWayPointsAhead);

    emit suggestVisualization();
}

void FlightPlannerCuda::slotDeleteGeneratedWayPoints()
{
    mWayPointsGenerated.clear();

    emit suggestVisualization();
}

void FlightPlannerCuda::slotWayPointReached(const WayPoint wpt)
{
    FlightPlannerInterface::slotWayPointReached(wpt);

//    slotCreateSafePathToNextWayPoint();
}

void FlightPlannerCuda::slotVehiclePoseChanged(const Pose& pose)
{
    FlightPlannerInterface::slotVehiclePoseChanged(pose);

    // check for collisions?!
}
