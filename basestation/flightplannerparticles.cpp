#include "flightplannerparticles.h"

FlightPlannerParticles::FlightPlannerParticles(QWidget* widget, Octree* pointCloud) : FlightPlannerInterface(widget, pointCloud)
{
    mParticleSystem = 0;
    mParticleRenderer = 0;
}

void FlightPlannerParticles::slotInitialize()
{
    qDebug() << "FlightPlannerParticles::slotInitialize()";

    // Initialize CUDA
    int numberOfCudaDevices;
    cudaGetDeviceCount(&numberOfCudaDevices);
    Q_ASSERT(numberOfCudaDevices && "FlightPlannerParticles::slotInitialize(): No CUDA devices found, exiting.");

    int activeCudaDevice;
    mCudaError = cudaGetDevice(&activeCudaDevice);
    if(mCudaError != cudaSuccess) qFatal("FlightPlannerParticles::slotInitialize(): couldn't get device: code %d: %s, exiting.", mCudaError, cudaGetErrorString(mCudaError));

    // Necessary for OpenGL graphics interop
    mCudaError = cudaGLSetGLDevice(activeCudaDevice);
    if(mCudaError != cudaSuccess) qFatal("FlightPlannerParticles::slotInitialize(): couldn't set device to GL interop mode: code %d: %s, exiting.", mCudaError, cudaGetErrorString(mCudaError));

    mCudaError = cudaSetDeviceFlags(cudaDeviceMapHost);// in order for the cudaHostAllocMapped flag to have any effect
    if(mCudaError != cudaSuccess) qFatal("FlightPlannerParticles::slotInitialize(): couldn't set device flag: code %d: %s, exiting.", mCudaError, cudaGetErrorString(mCudaError));

    cudaDeviceProp deviceProps;
    cudaGetDeviceProperties(&deviceProps, activeCudaDevice);

    size_t memTotal, memFree;
    cudaMemGetInfo(&memFree, &memTotal);

    qDebug() << "FlightPlannerParticles::FlightPlannerParticles(): device" << deviceProps.name << "has compute capability" << deviceProps.major << deviceProps.minor << "and"
             << memFree / 1048576 << "of" << memTotal / 1048576 << "mb free, has"
             << deviceProps.multiProcessorCount << "multiprocessors,"
             << (deviceProps.integrated ? "is" : "is NOT" ) << "integrated,"
             << (deviceProps.canMapHostMemory ? "can" : "can NOT") << "map host mem, has"
             << deviceProps.memoryClockRate / 1000 << "Mhz mem clock and a"
             << deviceProps.memoryBusWidth << "bit mem bus";

    uint numParticles = 256;
    uint3 gridSize;

    // simulation parameters
    damping = 1.0f;
    gravity = 0.0003f;
    ballr = 10;
    mode = 0;

    collideSpring = 0.5f;
    collideDamping = 0.02f;
    collideShear = 0.1f;
    collideAttraction = 0.0f;

    gridSize.x = gridSize.y = gridSize.z = 64;
    printf("grid: %d x %d x %d = %d cells\n", gridSize.x, gridSize.y, gridSize.z, gridSize.x*gridSize.y*gridSize.z);
    printf("particles: %d\n", numParticles);

    // unnecessary, just std opengl stuff                initGL(&argc, argv);
    //cudaGLInit(argc, argv); // simply does cudaGLSetGLDevice( cutGetMaxGflopsDeviceId() );, which is done above already

    mParticleSystem = new ParticleSystem(numParticles, gridSize);
    mParticleSystem->setVolume(mScanVolumeMin, mScanVolumeMax);
    mParticleSystem->reset(ParticleSystem::CONFIG_RANDOM);


    mParticleRenderer = new ParticleRenderer;
    connect(mParticleSystem, SIGNAL(particleRadiusChanged(float)), mParticleRenderer, SLOT(slotSetParticleRadius()));
    connect(mGlWidget, SIGNAL(fovChanged(float)), mParticleRenderer, SLOT(slotSetFovVertical(float)));
    mParticleSystem->slotSetParticleRadius(3.3f);
    mParticleRenderer->setColorBuffer(mParticleSystem->getColorBuffer());

    /*
    // create a new parameter list
    params = new ParamListGL("misc");
    params->AddParam(new Param<float>("time step", timestep, 0.0f, 1.0f, 0.01f, &timestep));
    params->AddParam(new Param<float>("damping"  , damping , 0.0f, 1.0f, 0.001f, &damping));
    params->AddParam(new Param<float>("gravity"  , gravity , 0.0f, 0.001f, 0.0001f, &gravity));
    params->AddParam(new Param<int>  ("ball radius", ballr , 1, 20, 1, &ballr));

    params->AddParam(new Param<float>("collide spring" , collideSpring , 0.0f, 1.0f, 0.001f, &collideSpring));
    params->AddParam(new Param<float>("collide damping", collideDamping, 0.0f, 0.1f, 0.001f, &collideDamping));
    params->AddParam(new Param<float>("collide shear"  , collideShear  , 0.0f, 0.1f, 0.001f, &collideShear));
    params->AddParam(new Param<float>("collide attract", collideAttraction, 0.0f, 0.1f, 0.001f, &collideAttraction));
*/

}

void FlightPlannerParticles::slotGenerateWaypoints()
{
    qDebug() << "FlightPlannerParticles::slotGenerateWaypoints()";

}


void FlightPlannerParticles::slotCreateSafePathToNextWayPoint()
{

}

FlightPlannerParticles::~FlightPlannerParticles()
{

    cudaDeviceReset();
}

void FlightPlannerParticles::insertPoint(LidarPoint* const point)
{
    //mVoxelManager->setVoxelValue(point->position, true);
}

// NOT in a glBegin()/glEnd() pair.
void FlightPlannerParticles::slotVisualize() const
{
    FlightPlannerInterface::slotVisualize();

    qDebug() << "FlightPlannerParticles::slotVisualize()";

    // update the simulation
    mParticleSystem->setDamping(damping);
    mParticleSystem->setGravity(-gravity);
    mParticleSystem->setCollideSpring(collideSpring);
    mParticleSystem->setCollideDamping(collideDamping);
    mParticleSystem->setCollideShear(collideShear);
    mParticleSystem->setCollideAttraction(collideAttraction);

    mParticleSystem->update(0.5f);
    mParticleRenderer->setWindowSize(mGlWidget->size());
    mParticleRenderer->setVertexBuffer(mParticleSystem->getCurrentReadBuffer(), mParticleSystem->getNumParticles());
    mParticleRenderer->display(ParticleRenderer::PARTICLE_SPHERES); // make this spheres. And make that work!
}

void FlightPlannerParticles::slotSetScanVolume(const QVector3D min, const QVector3D max)
{
    FlightPlannerInterface::slotSetScanVolume(min, max);

    if(mParticleSystem) mParticleSystem->setVolume(mScanVolumeMin, mScanVolumeMax);

    // Re-fill VoxelManager's data from basestations octree.
    if(mOctree) insertPointsFromNode(mOctree->root());
}

bool FlightPlannerParticles::insertPointsFromNode(const Node* node)
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


void FlightPlannerParticles::slotProcessPhysics(bool process)
{
}

void FlightPlannerParticles::slotSubmitGeneratedWayPoints()
{
    mWayPointsAhead->append(mWayPointsGenerated);
    mWayPointsGenerated.clear();
    sortToShortestPath(*mWayPointsAhead, mVehiclePoses.last().getPosition());
    emit wayPointsSetOnRover(*mWayPointsAhead);
    emit wayPoints(*mWayPointsAhead);

    emit suggestVisualization();
}

void FlightPlannerParticles::slotDeleteGeneratedWayPoints()
{
    mWayPointsGenerated.clear();

    emit suggestVisualization();
}

void FlightPlannerParticles::slotWayPointReached(const WayPoint wpt)
{
    FlightPlannerInterface::slotWayPointReached(wpt);

    //    slotCreateSafePathToNextWayPoint();
}

void FlightPlannerParticles::slotVehiclePoseChanged(const Pose& pose)
{
    FlightPlannerInterface::slotVehiclePoseChanged(pose);

    // check for collisions?!
}
