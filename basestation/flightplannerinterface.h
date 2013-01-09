#ifndef FLIGHTPLANNERINTERFACE_H
#define FLIGHTPLANNERINTERFACE_H

#include <QGLWidget>
#include <QMap>
#include "pointcloud.h"
#include "openglutilities.h"
#include "shaderprogram.h"
#include "waypointlist.h"
#include <pose.h>

class Pose;
class GlWidget;

class FlightPlannerInterface : public QObject
{
    Q_OBJECT
public:

protected:
    QVector3D mScanVolumeMin, mScanVolumeMax;
    QVector<Pose> mVehiclePoses;
    GlWidget* mGlWidget;
    QWidget* mParentWidget;
    PointCloud* mPointCloudDense;

    //QList<WayPoint>* mWayPointsAhead, *mWayPointsPassed;
    // A map, mapping from name to waypointlist. Names are e.g. waypoints_ahead, waypoints_passed etc.
    QMap<QString, WayPointList*> mWaypointListMap;

    unsigned int mBoundingBoxVbo;
    QVector<float> mBoundingBoxVertices;

    ShaderProgram *mShaderProgramDefault, *mShaderProgramSpheres;

    static void sortToShortestPath(QList<WayPoint> &wayPointsSetOnRover, const QVector3D &currentVehiclePosition);
    void setVboBoundingBox();

public:
    // Here, basestation passes its own pointcloud. Its up to the implementation to use it.
    FlightPlannerInterface(QWidget* widget, GlWidget *glWidget, PointCloud* pointcloud);
    virtual ~FlightPlannerInterface();

//    void setGlWidget(GlWidget* glWidget) {mGlWidget = glWidget;}

    const QVector<Pose>& getVehiclePoses() { return mVehiclePoses; }

    const Pose getLastKnownVehiclePose(void) const;
    const QVector3D getCurrentVehicleVelocity() const;

    const QList<WayPoint> *const getWayPoints();

    void getScanVolume(QVector3D& min, QVector3D& max);

private slots:
    void slotDeleteGeneratedWayPoints();
    void slotSubmitGeneratedWayPoints();

public slots:
    void slotWayPointDelete(const quint16& index);
    void slotWayPointInsert(const quint16& index, const WayPoint& wpt);
    void slotWayPointInsertedByRover(const quint16& index, const WayPoint& wpt);
    void slotWayPointSwap(const quint16& i, const quint16& j);
    void slotWayPointsClear();

    // Called by LogPlayer or RoverConnection when new scanData arrives.
    virtual void slotNewScanData(const QVector<QVector3D>* const pointList, const QVector3D* const scannerPosition) = 0;

    // Called by UI to clear the drawn trajectory
    void slotClearVehicleTrajectory();

    void slotCheckWayPointsHashFromRover(const QString& hash);

    // Used in CUDA flightplanner, might be useful to others. Will be called
    // soon after construction, but maybe after the first call to slotVisualize().
    virtual void slotInitialize() = 0;

    virtual void slotWayPointReached(const WayPoint&);

    virtual void slotSetScanVolume(const QVector3D min, const QVector3D max);
    virtual void slotGenerateWaypoints() = 0;
    virtual void slotVisualize();

    void slotVehiclePoseChanged(const Pose *const pose);

signals:
    void message(const LogImportance& importance, const QString& source, const QString& message);

    // Emitted to tell other classes that waypoint @wpt was inserted at index @index
    void wayPointInserted(const quint16& index, const WayPoint& wpt);

    // Emitted to tell the rover that it should insert waypoint @wpt at index @index
    void wayPointInsertOnRover(const quint16& index, const WayPoint& wpt);

    // Emitted to tell other classes that waypoint @index was deleted. We do not care how or by whom
    void wayPointDeleted(const quint16& index);
    // Emitted to tell the rover that it should delete waypoint @index
    void wayPointDeleteOnRover(const quint16& index);

    void wayPointsSetOnRover(const QList<WayPoint>* const);
    void wayPoints(const QList<WayPoint>* const);

    void suggestVisualization();
};

#endif // FLIGHTPLANNERINTERFACE_H
