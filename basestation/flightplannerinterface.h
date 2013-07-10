#ifndef FLIGHTPLANNERINTERFACE_H
#define FLIGHTPLANNERINTERFACE_H

#include <QMap>
#include <QOpenGLFunctions_4_3_Core>
#include "pointcloud.h"
#include "openglutilities.h"
#include "shaderprogram.h"
#include "waypointlist.h"
#include <pose.h>

class Pose;
class GlWindow;
class BaseStation;

class FlightPlannerInterface : public QObject, protected QOpenGLFunctions_4_3_Core
{
    Q_OBJECT
public:

protected:
    QVector3D mScanVolumeMin, mScanVolumeMax;
    QVector<Pose> mVehiclePoses;
    GlWindow* mGlWindow;
    BaseStation* mBaseStation;
    PointCloud* mPointCloudDense;

    bool mRenderGlobalBoundingBox, mRenderLocalBoundingBox, mRenderWayPoints;

    //QList<WayPoint>* mWayPointsAhead, *mWayPointsPassed;
    // A map, mapping from name to waypointlist. Names are e.g. waypoints_ahead, waypoints_passed etc.
    QMap<QString, WayPointList*> mWaypointListMap;

    unsigned int mVboBoundingBox, mVboWayPointConnections;
    //QVector<float> mBoundingBoxVertices;

    ShaderProgram *mShaderProgramDefault, *mShaderProgramSpheres;

    static void sortToShortestPath(QList<WayPoint> &wayPointsSetOnRover, const QVector3D &currentVehiclePosition);

public:
    // Here, basestation passes its own pointcloud. Its up to the implementation to use it.
    FlightPlannerInterface(BaseStation* basestation, GlWindow *glWidget, PointCloud* pointcloud);
    virtual ~FlightPlannerInterface();

//    void setGlWidget(GlWidget* glWidget) {mGlWidget = glWidget;}

//    const QVector<Pose>& getVehiclePoses() { return mVehiclePoses; }

    const Pose getLastKnownVehiclePose(void) const;
//    const QVector3D getCurrentVehicleVelocity() const;

    const QList<WayPoint> *const getWayPoints();

//    void getScanVolume(QVector3D& min, QVector3D& max);

    QVector3D getScanVolumeSize() const
    {
        return QVector3D(
                    mScanVolumeMax.x() - mScanVolumeMin.x(),
                    mScanVolumeMax.y() - mScanVolumeMin.y(),
                    mScanVolumeMax.z() - mScanVolumeMin.z());
    }

    QVector3D getScanVolumeCenter() const
    {
        return QVector3D(
                    mScanVolumeMin.x(),
                    mScanVolumeMin.y(),
                    mScanVolumeMin.z()
                    ) + getScanVolumeSize() / 2.0f;
    }


    virtual void keyPressEvent(QKeyEvent* event) = 0;

public slots:
    // Called by LogPlayer or RoverConnection when new scanData arrives. @points must be float4!
    virtual void slotNewScanFused(const float* const points, const quint32& count, const QVector3D* const scannerPosition) = 0;

    // Called by UI to clear the drawn trajectory
    void slotClearVehicleTrajectory();

    // Used in CUDA flightplanner, might be useful to others. Will be called
    // soon after construction, but maybe after the first call to slotVisualize().
    virtual void slotInitialize() = 0;

    // Called to set the waypoint-list. There are at least 3 entities reading and writing those lists:
    //
    // - ControlWidget
    // - FlightPlanner
    // - Rover
    //
    // When slotSetWayPoints(...) is called, the FlightPlanner-waypoint-list will be set and wayPoints(...)
    // will be emitted. So, if e.g. ControlWidget makes a change and calls FlightPlanner::slotSetWayPoints(),
    // the resulting wayPoints(...) signal will unnecessarily update the ControlWidget again. Same for Rover.
    //
    // To prevent this, the caller of this method should indicate the source of the list (ControlWidget would
    // use ControlWidget, Rover would use Rover, you get the idea). Then, the wayPoints()-signal is emitted
    // with this same source again and ControlWidget can use that to ignore the list. Sweet.
    void slotSetWayPoints(const QList<WayPoint>* const wayPointList, const WayPointListSource source/* = WayPointListSource::WayPointListSourceUnknown*/);

    virtual void slotWayPointReached(const WayPoint&);

    virtual void slotSetScanVolume(const QVector3D min, const QVector3D max);
    virtual void slotGenerateWaypoints(quint32 numberOfWaypointsToGenerate = 1) = 0;
    virtual void slotShowUserInterface() {}
    virtual void slotVisualize();

    void slotVehiclePoseChanged(const Pose *const pose);


    void slotSetRenderLocalBoundingBox(const bool enable) { mRenderLocalBoundingBox = enable; }
    void slotSetRenderGlobalBoundingBox(const bool enable) { mRenderGlobalBoundingBox = enable; }
    void slotSetRenderWayPoints(const bool enable) { mRenderWayPoints = enable; }


signals:
    void message(const LogImportance& importance, const QString& source, const QString& message);

    // This is emitted whenever FlightPlanner changes waypoints
    void wayPoints(const QList<WayPoint>* const, const WayPointListSource);

    void suggestVisualization();
};

#endif // FLIGHTPLANNERINTERFACE_H
