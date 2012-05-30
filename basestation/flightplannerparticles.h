#ifndef FlightPlannerParticles_H
#define FlightPlannerParticles_H

#include "flightplannerinterface.h"
#include "particlerenderer.h"
#include "particlesystem.h"
#include "node.h"
#include "lidarpoint.h"
#include "glwidget.h"
#include "voxelmanager.h"
#include <waypoint.h>
#include "openglutilities.h"

#define MAX_EPSILON_ERROR 5.00f
#define THRESHOLD         0.30f
//#define GRID_SIZE       64
//#define NUM_PARTICLES   16384

extern "C" void cudaGLInit(int argc, char **argv);

class FlightPlannerParticles : public FlightPlannerInterface
{
    Q_OBJECT
public:
    FlightPlannerParticles(QWidget* glWidget, Octree* pointCloud);
    ~FlightPlannerParticles();

    void insertPoint(LidarPoint* point);

private:

    QList<WayPoint> mWayPointsGenerated, mWayPointsDetour;

    int mode;

    enum { M_VIEW = 0, M_MOVE };

    uint numParticles;
    uint3 gridSize;

    // simulation parameters
    float timestep;
    float damping;
    float gravity;
    int ballr;

    float collideSpring;
    float collideDamping;
    float collideShear;
    float collideAttraction;


    ParticleSystem* mParticleSystem;
    ParticleRenderer* mParticleRenderer;

    cudaError_t mCudaError;

    // To re-fill our datastructure when the boundingbox has changed.
    bool insertPointsFromNode(const Node* node);

signals:

private slots:
    void slotGenerateWaypoints();
    void slotProcessPhysics(bool);
//    void slotSubmitGeneratedWayPoints();
//    void slotDeleteGeneratedWayPoints();

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

#endif // FlightPlannerParticles_H
