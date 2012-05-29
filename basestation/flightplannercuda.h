#ifndef FLIGHTPLANNERCUDA_H
#define FLIGHTPLANNERCUDA_H

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include "flightplannerinterface.h"
#include "node.h"
#include "lidarpoint.h"
#include "voxelmanager.h"
#include <waypoint.h>
#include "openglutilities.h"

extern "C" void computeFreeColumns(
    unsigned char* hostGridPointer,
    unsigned char* pixmap,
    float4* vertexPositions,
    int resX, int resY, int resZ,
    const QVector3D& bBoxMin,
    const QVector3D& bBoxMax);

class FlightPlannerCuda : public FlightPlannerInterface
{
    Q_OBJECT
public:
    FlightPlannerCuda(QWidget* widget, Octree* pointCloud);
    ~FlightPlannerCuda();

    void insertPoint(LidarPoint* const point);

private:

    cudaError_t mCudaError;

    VoxelManager* mVoxelManager;

    GLuint mSampleVertexArray; // VBO with vertices to be written from CUDA kernel.
    cudaGraphicsResource* mSampleVertexArrayCudaResource; // the same VBO, represented as a CUDA graphics resource

    QList<WayPoint> mWayPointsGenerated, mWayPointsDetour;

    GLuint mColumnOccupancyTexture; // the texture to whch we'll write with CUDA
    cudaGraphicsResource* mColumnOccupancyTextureCudaResource; // the same texture, represented as a CUDA graphics resource

    QImage* mHostColumnOccupancyImage; // the pointer to the host's copy of the kernel's result (the grayscale map of column traversability)
    unsigned char* mDeviceColumnOccupancyPixmapData; // the pointer to the device's copy of the kernel's result (the grayscale map of column traversability)
//    unsigned char* mDeviceVolumeData; // the pointer to the device#s copy of the volume data. UNUSED, we use mapped memory

    // To re-fill our datastructure when the boundingbox has changed.
    bool insertPointsFromNode(const Node* node);

signals:

private slots:
    void slotGenerateWaypoints();
    void slotProcessPhysics(bool);

public slots:
    void slotSetScanVolume(const QVector3D min, const QVector3D max);

    // Inserts detour-waypoints between vehicle position and next waypoint if necessary.
    // Returns true if path was found, else false.
    void slotCreateSafePathToNextWayPoint();

    void slotInitialize();

    // Overridden from base, create safe path to next waypoint whenever current one was reached.
    void slotWayPointReached(const WayPoint);

    // Overridden from base to move vehicle physics object and check for collisions
    void slotVehiclePoseChanged(const Pose& pose);

    void slotVisualize();

};

#endif // FLIGHTPLANNERCUDA_H
