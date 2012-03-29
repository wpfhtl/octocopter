#include "flightplannercuda.h"

FlightPlannerCuda::FlightPlannerCuda(QWidget* widget, Octree* pointCloud) : FlightPlannerInterface(widget, pointCloud)
{
    mVoxelManager = new VoxelManager(64, 64, 64);
}

void FlightPlannerCuda::slotInitialize()
{
    // Initialize CUDA
    int numberOfCudaDevices;
    cudaGetDeviceCount(&numberOfCudaDevices);
    Q_ASSERT(numberOfCudaDevices && "FlightPlannerCuda::slotInitialize(): No CUDA devices found, exiting.");

    int activeCudaDevice;
    mCudaError = cudaGetDevice(&activeCudaDevice);
    if(mCudaError != cudaSuccess) qFatal("FlightPlannerCuda::slotInitialize(): couldn't get device: code %d: %s, exiting.", mCudaError, cudaGetErrorString(mCudaError));

    // Necessary for OpenGL graphics interop
    mCudaError = cudaGLSetGLDevice(activeCudaDevice);
    if(mCudaError != cudaSuccess) qFatal("FlightPlannerCuda::slotInitialize(): couldn't set device to GL interop mode: code %d: %s, exiting.", mCudaError, cudaGetErrorString(mCudaError));

    mCudaError = cudaSetDeviceFlags(cudaDeviceMapHost);// in order for the cudaHostAllocMapped flag to have any effect
    if(mCudaError != cudaSuccess) qFatal("FlightPlannerCuda::slotInitialize(): couldn't set device flag: code %d: %s, exiting.", mCudaError, cudaGetErrorString(mCudaError));

    cudaDeviceProp deviceProps;
    cudaGetDeviceProperties(&deviceProps, activeCudaDevice);

    size_t memTotal, memFree;
    cudaMemGetInfo(&memFree, &memTotal);

    qDebug() << "FlightPlannerCuda::FlightPlannerCuda(): device" << deviceProps.name << "has compute capability" << deviceProps.major << deviceProps.minor << "and"
             << memFree / 1048576 << "of" << memTotal / 1048576 << "mb free, has"
             << deviceProps.multiProcessorCount << "multiprocessors,"
             << (deviceProps.integrated ? "is" : "is NOT" ) << "integrated,"
             << (deviceProps.canMapHostMemory ? "can" : "can NOT") << "map host mem, has"
             << deviceProps.memoryClockRate / 1000 << "Mhz mem clock and a"
             << deviceProps.memoryBusWidth << "bit mem bus";


    // Allocate data on host and device for the volume data
    mCudaError = cudaHostAlloc(mVoxelManager->getVolumeDataBasePointer(), mVoxelManager->getVolumeDataSize(), cudaHostAllocMapped | cudaHostAllocWriteCombined);
    if(mCudaError != cudaSuccess) qFatal("FlightPlannerCuda::slotInitialize(): couldn't allocate %llu bytes of pinned memory: code %d: %s, exiting.", mVoxelManager->getVolumeDataSize(), mCudaError, cudaGetErrorString(mCudaError));

    // Clear the voloume data
    memset(*mVoxelManager->getVolumeDataBasePointer(), 0, mVoxelManager->getVolumeDataSize());

    mHostColumnOccupancyImage = new QImage(mVoxelManager->getResolutionX(), mVoxelManager->getResolutionZ(), QImage::Format_RGB888);

    cudaMemGetInfo(&memFree, &memTotal);
    qDebug() << "FlightPlannerCuda::slotInitialize(): after allocating pinned memory, device has" << memFree / 1048576 << "of" << memTotal / 1048576 << "mb memory free";

    // Create a GL texture for column occupancy results
    /* Writing to textures is "surface write" in CUDA and requires compute capability 2.0 - we have 1.X :(
    {
        glGenTextures( 1, &mColumnOccupancyTexture );
        glBindTexture( GL_TEXTURE_2D, mColumnOccupancyTexture );

        // set basic parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

        // Create texture data (4-component unsigned byte)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, mVoxelManager->getResolutionX(), mVoxelManager->getResolutionZ(), 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);

        // Unbind the texture
        glBindTexture(GL_TEXTURE_2D, 0);

        // Register it with CUDA
        mCudaError = cudaGraphicsGLRegisterImage(
          &mColumnOccupancyTextureCudaResource,
          mColumnOccupancyTexture,
          GL_TEXTURE_2D, // GL_TEXTURE_2D, GL_TEXTURE_RECTANGLE, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_3D, or GL_TEXTURE_2D_ARRAY ???
          cudaGraphicsRegisterFlagsNone // or cudaGraphicsRegisterFlagsWriteDiscard if we only write to it??
        );
        if(mCudaError != cudaSuccess) qFatal("FlightPlannerCuda::slotInitialize(): couldn't register GL texture for CUDA usage: code %d: %s, exiting.", mCudaError, cudaGetErrorString(mCudaError));
    }*/

    {
        // Create the GL buffer for vertices with a color (3 floats = 12 bytes and 4 bytes rgba color)
        glGenBuffers(1,&mSampleVertexArray);
        glBindBuffer(GL_ARRAY_BUFFER, mSampleVertexArray);

        // initialize buffer object
        const unsigned int size = mVoxelManager->getResolutionX() * mVoxelManager->getResolutionZ() * 4 * sizeof(float);
        unsigned char* daten = new unsigned char[size];
//        memset(daten, 80, size); keep random

        glBufferData(GL_ARRAY_BUFFER, size, (void*)daten, GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        // register this buffer object with CUDA
        mCudaError = cudaGraphicsGLRegisterBuffer(&mSampleVertexArrayCudaResource, mSampleVertexArray, cudaGraphicsMapFlagsWriteDiscard);
        if(mCudaError != cudaSuccess) qFatal("FlightPlannerCuda::slotInitialize(): couldn't register GL VBO for CUDA usage: code %d: %s, exiting.", mCudaError, cudaGetErrorString(mCudaError));
    }
}

void FlightPlannerCuda::slotGenerateWaypoints()
{
    qDebug() << "FlightPlannerCuda::slotGenerateWaypoints(): total occupancy is" << mVoxelManager->getTotalOccupancy();

    /*
    // Lock the texture for CUDA usage - DISABLED, needs compute capability 2.0
    mCudaError = cudaGraphicsMapResources(1, &mColumnOccupancyTextureCudaResource, 0);
    if(mCudaError != cudaSuccess) qFatal("FlightPlannerCuda::slotGenerateWaypoints(): couldn't map/lock texture for CUDA usage: code %d: %s, exiting.", mCudaError, cudaGetErrorString(mCudaError));
    */

    // Map buffer object for writing from CUDA
    float4* vertexPositions;
    mCudaError = cudaGraphicsMapResources(1, &mSampleVertexArrayCudaResource, 0);
    if(mCudaError != cudaSuccess) qFatal("FlightPlannerCuda::slotGenerateWaypoints(): couldn't map/lock VBO for CUDA usage: code %d: %s, exiting.", mCudaError, cudaGetErrorString(mCudaError));

    // Get the pointer for the device code.
    size_t num_bytes;
    mCudaError = cudaGraphicsResourceGetMappedPointer((void**)&vertexPositions, &num_bytes, mSampleVertexArrayCudaResource);
    if(mCudaError != cudaSuccess) qFatal("FlightPlannerCuda::slotGenerateWaypoints(): couldn't get pointer to VBO for CUDA usage: code %d: %s, exiting.", mCudaError, cudaGetErrorString(mCudaError));

    // Start kernel to check reachability, give it pointer to VBO positions and let it write to pixmap
    computeFreeColumns(
                *mVoxelManager->getVolumeDataBasePointer(),
                mHostColumnOccupancyImage->bits(),
                vertexPositions,
                mVoxelManager->getResolutionX(),
                mVoxelManager->getResolutionY(),
                mVoxelManager->getResolutionZ(),
                mScanVolumeMin, mScanVolumeMax);

    mCudaError = cudaGraphicsUnmapResources(1, &mSampleVertexArrayCudaResource, 0);
    if(mCudaError != cudaSuccess) qFatal("FlightPlannerCuda::slotGenerateWaypoints(): couldn't unmap/unlock VBO from CUDA usage: code %d: %s, exiting.", mCudaError, cudaGetErrorString(mCudaError));
}


void FlightPlannerCuda::slotCreateSafePathToNextWayPoint()
{

}

FlightPlannerCuda::~FlightPlannerCuda()
{
    delete mVoxelManager;

    delete mHostColumnOccupancyImage;

    // disabled, needs compute capability 2.0
    //glDeleteTextures(1, &mColumnOccupancyTexture);

    // delete VBO
    cudaGraphicsUnregisterResource(mSampleVertexArrayCudaResource);
    glDeleteBuffers(1, &mSampleVertexArray);

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

    qDebug() << "FlightPlannerCuda::slotVisualize(): total occupancy:" << mVoxelManager->getTotalOccupancy() << "rendering" << mVoxelManager->getResolutionX() * mVoxelManager->getResolutionZ() << "points";


    // Use CUDA?!
//    void* vertexPointer;
    // Map the buffer to CUDA
//    cudaGLMapBufferObject(&vertexPointer, mVertexArray);
    // Run a kernel to create/manipulate the data
//    MakeVerticiesKernel<<<gridSz,blockSz>>>(vertexPointer, mNumVoxels);
    // Unmap the buffer
//    cudaGLUnmapbufferObject(mVertexArray);

    // Render the VBO!
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPointSize(3);
    glColor3f(0.5, 0.5, 0.5);
    glBindBuffer(GL_ARRAY_BUFFER, mSampleVertexArray);
    glVertexPointer(4, GL_FLOAT, 0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_POINTS,0, mVoxelManager->getResolutionX() * mVoxelManager->getResolutionZ() * 2);
    glDisableClientState(GL_VERTEX_ARRAY);

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
    sortToShortestPath(*mWayPointsAhead, mVehiclePoses.last().getPosition());
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
