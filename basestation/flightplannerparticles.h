#ifndef FLIGHTPLANNERPARTICLES_H
#define FLIGHTPLANNERPARTICLES_H

#include "particlerenderer.h"
#include "particlesystem.h"
#include "pathplanner.h"
#include "lidarpoint.h"
#include "glwindow.h"
#include <waypoint.h>
#include "waypointlist.h"
#include "openglutilities.h"

#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLFunctions_4_3_Core>

class FlightPlannerParticlesDialog;

class PointCloudCuda;
class BaseStation;

class FlightPlannerParticles : public QObject, protected OPENGL_FUNCTIONS_CLASS
{
    Q_OBJECT
public:
    FlightPlannerParticles(BaseStation* baseStation, GlWindow* glWidget, PointCloud* pointcloud);
    ~FlightPlannerParticles();

    const Pose getLastKnownVehiclePose(void) const;

    const QList<WayPoint> *const getWayPoints();

    const Box3D* getScanVolume() const
    {
        return &mScanVolume;
    }

private:
    Box3D mScanVolume;
    QVector<Pose> mVehiclePoses;
    GlWindow* mGlWindow;
    BaseStation* mBaseStation;
    PointCloud* mPointCloudDense;

    bool mRenderScanVolume, mRenderDetectionVolume, mRenderWayPoints;

    WayPointList mWayPointsAhead, mWayPointsPassed;
    // A map, mapping from name to waypointlist. Names are e.g. waypoints_ahead, waypoints_passed etc.
//    QMap<QString, WayPointList*> mWaypointListMap;

    unsigned int mVboBoundingBox, mVboWayPointConnections;

    ShaderProgram *mShaderProgramDefault, *mShaderProgramWaypoint;

    static void sortToShortestPath(QList<WayPoint> &wayPointsSetOnRover, const QVector3D &currentVehiclePosition);

    FlightPlannerParticlesDialog* mDialog;
    PointCloudCuda* mPointCloudColliders;
    PathPlanner* mPathPlanner;
    QVector3D mLastParticleSystemPositionToFollowVehicle;
    QTimer mTimerProcessInformationGain;
    QList<WayPoint> mWayPointsGenerated, mWayPointsDetour;
    ParticleSystem* mParticleSystem;
    ParticleRenderer* mParticleRenderer;
    cudaError_t mCudaError;
    ShaderProgram* mShaderProgramGridLines; // for drawing the grid
    qint32 mActiveWayPointVisualizationIndex;

    // To re-fill our datastructure when the boundingbox has changed.
    bool insertPointsFromNode(const Node* node);

    void showInformationGain();

    ParametersParticleSystem mSimulationParameters;

    // There is a gridmap of waypoint pressure on the GPU. To be useful for processing on the host, we first create a
    // list of QVector4D that corresponds to the cell's positions. Then, we sort the latter list using thrust::sort_by_key
    // (the key being the waypoint pressure) and receive a ranking of QVector4Ds. Hah!
    float* mDeviceGridInformationGainCellWorldPositions;

    // A gridmap (same grid as always) containing values from 0 to 255. 0 means no waypoint candidates within, 255 means maximum waypoint pressure.
    unsigned int mVboGridMapOfInformationGainBytes;
    unsigned int mVboGridMapOfInformationGainFloats;
    // Waypoint-pressure-values are stored in a VBO as 8bit-unsigned-ints
    struct cudaGraphicsResource *mCudaVboResourceGridMapOfInformationGainBytes; // handles OpenGL-CUDA exchange
    struct cudaGraphicsResource *mCudaVboResourceGridMapOfInformationGainFloats; // handles OpenGL-CUDA exchange

private slots:
    void slotDenseCloudInsertedPoints(PointCloud*const pointCloudSource, const quint32& firstPointToReadFromSrc, quint32 numberOfPointsToCopy);

    // checks waypoint pressure and if higher than threshold, cals slotGenerateWaypoints();
    void slotProcessInformationGain(const quint8 threshold = 5);

public slots:
    void slotShowUserInterface();
    void slotSetScanVolume(const Box3D scanVolume);
    void slotSetActiveWayPoint(qint32 index) {mActiveWayPointVisualizationIndex = index; suggestVisualization();}

    void slotNewScanFused(const float* const points, const quint32& count, const QVector3D* const scannerPosition);

    void slotClearGridWayPointPressure();

    // Used in CUDA flightplanner, might be useful to others. Will be called
    // soon after construction, but maybe after the first call to slotVisualize().
    void slotInitialize();

    // Overridden from base to move vehicle physics object and check for collisions
    void slotVehiclePoseChanged(const Pose *const pose);

    // Called by UI to clear the drawn trajectory
    void slotClearVehicleTrajectory();


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

    void slotWayPointReached(const WayPoint&);

    void slotGenerateWaypoints(quint32 numberOfWaypointsToGenerate = 10);
    void slotVisualize();

    void slotSetRenderDetectionVolume(const bool enable) { mRenderDetectionVolume = enable; }
    void slotSetRenderScanVolume(const bool enable) { mRenderScanVolume = enable; }
    void slotSetRenderWayPoints(const bool enable) { mRenderWayPoints = enable; }


signals:
    void message(const LogImportance& importance, const QString& source, const QString& message);

    void vboInfoGridInformationGain(quint32 vboPressure, Box3D gridBoundingBox, Vector3i grid);

    // This is emitted whenever FlightPlanner changes waypoints
    void wayPoints(const QList<WayPoint>* const, const WayPointListSource);

    void suggestVisualization();
};

#endif // FLIGHTPLANNERPARTICLES_H
