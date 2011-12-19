#ifndef FLIGHTPLANNERCUDA_H
#define FLIGHTPLANNERCUDA_H

//#include <GL/glew.h>
//#include <GL/freeglut.h>
#include <cuda_runtime.h>

#include "flightplannerinterface.h"
#include "node.h"
#include "lidarpoint.h"
#include "voxelmanager.h"
#include <waypoint.h>
#include "openglutilities.h"

extern "C" void computeFreeColumns(unsigned char* gridPointer, unsigned char* pixmap);

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

    GLuint mVertexArray;

    QList<WayPoint> mWayPointsGenerated, mWayPointsDetour;

    unsigned char* mHostColumnOccupancyPixmapData; // the pointer to the host's copy of the kernel's result (the grayscale map of column traversability)
    unsigned char* mDeviceColumnOccupancyPixmapData; // the pointer to the device's copy of the kernel's result (the grayscale map of column traversability)
//    unsigned char* mDeviceVolumeData; // the pointer to the device#s copy of the volume data. UNUSED, we use mapped memory

signals:

private slots:
    void slotGenerateWaypoints();
    void slotProcessPhysics(bool);
    void slotSubmitGeneratedWayPoints();
    void slotDeleteGeneratedWayPoints();

public slots:
    void slotSetScanVolume(const QVector3D min, const QVector3D max);

    // Inserts detour-waypoints between vehicle position and next waypoint if necessary.
    // Returns true if path was found, else false.
    void slotCreateSafePathToNextWayPoint();

    // Overridden from base, create safe path to next waypoint whenever current one was reached.
    void slotWayPointReached(const WayPoint);

    // Overridden from base to move vehicle physics object and check for collisions
    void slotVehiclePoseChanged(const Pose& pose);

    void slotVisualize() const;

};

#endif // FLIGHTPLANNERCUDA_H
