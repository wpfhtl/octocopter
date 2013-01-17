#ifndef FLIGHTPLANNERPARTICLES_H
#define FLIGHTPLANNERPARTICLES_H

#include "flightplannerinterface.h"
#include "particlerenderer.h"
#include "particlesystem.h"
#include "lidarpoint.h"
#include "glwidget.h"
#include <waypoint.h>
#include "openglutilities.h"

class PointCloudCuda;

class FlightPlannerParticles : public FlightPlannerInterface
{
    Q_OBJECT
public:
    FlightPlannerParticles(QWidget* parentWidget, GlWidget* glWidget, PointCloud* pointcloud);
    ~FlightPlannerParticles();

    void keyPressEvent(QKeyEvent *event);

private:
    bool mUpdateParticleSystem;

    PointCloudCuda* mPointCloudColliders;
    // cursor?

    QList<WayPoint> mWayPointsGenerated, mWayPointsDetour;

    ParticleSystem* mParticleSystem;
    ParticleRenderer* mParticleRenderer;

    cudaError_t mCudaError;

    ShaderProgram* mShaderProgramGridLines; // for drawing the grid
    unsigned int mVboGridLines;

    // To re-fill our datastructure when the boundingbox has changed.
    bool insertPointsFromNode(const Node* node);

signals:

private slots:
    void slotGenerateWaypoints();
    void slotProcessPhysics(bool);

    // Our octree found @point to be alone enough to be stored. So this method inserts it into the particle system.
//    void slotPointAcceptedIntoOctree(const LidarPoint*point);

public slots:
    void slotSetScanVolume(const QVector3D min, const QVector3D max);

    void slotNewScanData(const QVector<QVector3D>* const pointList, const QVector3D* const scannerPosition);

    // Inserts detour-waypoints between vehicle position and next waypoint if necessary.
    // Returns true if path was found, else false.
    void slotCreateSafePathToNextWayPoint();

    void slotInitialize();

    // Overridden from base, create safe path to next waypoint whenever current one was reached.
    void slotWayPointReached(const WayPoint);

    // Overridden from base to move vehicle physics object and check for collisions
    void slotVehiclePoseChanged(const Pose *const pose);

    void slotVisualize();

};

#endif // FlightPlannerParticles_H
