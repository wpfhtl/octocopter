#include "flightplannercuda.h"

FlightPlannerCuda::FlightPlannerCuda(QWidget* widget, Octree* pointCloud) : FlightPlannerInterface(widget, pointCloud)
{
    // Initialize CUDA
    int numberOfCudaDevices;
    cudaGetDeviceCount(&numberOfCudaDevices);
    Q_ASSERT(numberOfCudaDevices && "FlightPlannerCuda::FlightPlannerCuda(): No CUDA devices found, exiting.");

    int activeCudaDevice;
    mCudaError = cudaGetDevice(&activeCudaDevice);
    if(mCudaError != cudaSuccess) qFatal("FlightPlannerCuda::FlightPlannerCuda(): couldn't get device: code %d: %s, exiting.", mCudaError, cudaGetErrorString(mCudaError));

    mCudaError = cudaSetDeviceFlags(cudaDeviceMapHost);// in order for the cudaHostAllocMapped flag to have any effect
    if(mCudaError != cudaSuccess) qFatal("FlightPlannerCuda::FlightPlannerCuda(): couldn't set device flag: code %d: %s, exiting.", mCudaError, cudaGetErrorString(mCudaError));

    cudaDeviceProp deviceProps;
    cudaGetDeviceProperties(&deviceProps, activeCudaDevice);



    // Necessary for OpenGL graphics interop
    mCudaError = cudaGLSetGLDevice(0);
    if(mCudaError != cudaSuccess) qFatal("FlightPlannerCuda::FlightPlannerCuda(): couldn't set device to GL interop mode: code %d: %s, exiting.", mCudaError, cudaGetErrorString(mCudaError));

    size_t memTotal, memFree;
    cudaMemGetInfo(&memFree, &memTotal);

    qDebug() << "FlightPlannerCuda::FlightPlannerCuda(): device" << deviceProps.name << "has compute capability" << deviceProps.major << deviceProps.minor << "and"
             << memFree / 1048576 << "of" << memTotal / 1048576 << "mb free, has"
             << deviceProps.multiProcessorCount << "multiprocessors,"
             << (deviceProps.integrated ? "is" : "is NOT" ) << "integrated,"
             << (deviceProps.canMapHostMemory ? "can" : "can NOT") << "map host mem, has"
             << deviceProps.memoryClockRate / 1000 << "Mhz mem clock and a"
             << deviceProps.memoryBusWidth << "bit mem bus";

    mVoxelManager = new VoxelManager(64, 64, 64);

    // Allocate data on host and device for the volume data
    mCudaError = cudaHostAlloc(mVoxelManager->getVolumeDataBasePointer(), mVoxelManager->getVolumeDataSize(), cudaHostAllocMapped | cudaHostAllocWriteCombined);
    if(mCudaError != cudaSuccess) qFatal("FlightPlannerCuda::FlightPlannerCuda(): couldn't allocate %llu bytes of pinned memory: code %d: %s, exiting.", mVoxelManager->getVolumeDataSize(), mCudaError, cudaGetErrorString(mCudaError));

    // Clear the voloume data
    memset(*mVoxelManager->getVolumeDataBasePointer(), 0, mVoxelManager->getVolumeDataSize());

    mHostColumnOccupancyPixmapData = new unsigned char[mVoxelManager->getGroundPlanePixelCount()];

    cudaMemGetInfo(&memFree, &memTotal);
    qDebug() << "FlightPlannerCuda::FlightPlannerCuda(): after allocating pinned memory, device has" << memFree / 1048576 << "of" << memTotal / 1048576 << "mb memory free";

    // Create the GL buffer for vertices with a color (3 floats = 12 bytes and 4 bytes rgba color)
//    glGenBuffers(1,&mVertexArray);
//    glBindBuffer(GL_ARRAY_BUFFER, mVertexArray);
//    glBufferData(GL_ARRAY_BUFFER, mNumVoxels * 16, NULL, GL_DYNAMIC_COPY);
//    cudaGLRegisterBufferObject(mVertexArray);
}


void FlightPlannerCuda::slotGenerateWaypoints()
{
    qDebug() << "FlightPlannerCuda::slotGenerateWaypoints(): total occupancy is" << mVoxelManager->getTotalOccupancy();

    // Start kernel to check reachability, return pixmap
    computeFreeColumns(
                *mVoxelManager->getVolumeDataBasePointer(),
                mHostColumnOccupancyPixmapData,
                mVoxelManager->getResolutionX(),
                mVoxelManager->getResolutionY(),
                mVoxelManager->getResolutionZ());
}


void FlightPlannerCuda::slotCreateSafePathToNextWayPoint()
{

}

FlightPlannerCuda::~FlightPlannerCuda()
{
    delete mVoxelManager;

    delete mHostColumnOccupancyPixmapData;

    // Shutdown CUDA
    cudaFreeHost(*mVoxelManager->getVolumeDataBasePointer());

    cudaDeviceReset();
}

void FlightPlannerCuda::insertPoint(LidarPoint* const point)
{
    mVoxelManager->setVoxelValue(point->position, true);
}

// NOT in a glBegin()/glEnd() pair.
void FlightPlannerCuda::slotVisualize() const
{
    FlightPlannerInterface::slotVisualize();

    qDebug() << "FlightPlannerCuda::slotVisualize(): total occupancy:" << mVoxelManager->getTotalOccupancy();
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

    // Re-fill VoxelManager's data from basestations octree.
    if(mOctree) insertPointsFromNode(mOctree->root());
}

bool FlightPlannerCuda::insertPointsFromNode(const Node* node)
{
    if(node->isLeaf())
    {
        for(int i=0;i<node->data.size();i++)
            insertPoint(node->data.at(i));
    }
    else
    {
        // invoke recursively for childnodes/leafs
        const QList<const Node*> childNodes = node->getAllChildLeafs();
        foreach(const Node* childNode, childNodes)
            if(!insertPointsFromNode(childNode))
                return false;
    }

    return true;
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
