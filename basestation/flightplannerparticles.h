#ifndef FLIGHTPLANNERPARTICLES_H
#define FLIGHTPLANNERPARTICLES_H

#include "glscene.h"
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
    FlightPlannerParticles(BaseStation* baseStation, GlWindow* glWidget, PointCloudCuda *pointcloud);
    ~FlightPlannerParticles();

    const Pose getLastKnownVehiclePose(void) const;

    const QList<WayPoint> *const getWayPoints();

    PointCloudCuda* getPointCloudColliders() {return mPointCloudColliders;}
    PathPlanner* getPathPlanner() {return mPathPlanner;}

private:
    Box3D mVolumeLocal, mVolumeGlobal;
    QVector<Pose> mVehiclePoses;
    GlWindow* mGlWindow;
    BaseStation* mBaseStation;
    PointCloudCuda* mPointCloudDense;

    WayPointList mWayPointsAhead, mWayPointsPassed;

    static void sortToShortestPath(QList<WayPoint> &wayPointsSetOnRover, const QVector3D &currentVehiclePosition);

    FlightPlannerParticlesDialog* mDialog;
    PointCloudCuda* mPointCloudColliders;
    PathPlanner* mPathPlanner;
    QVector3D mLastParticleSystemPositionToFollowVehicle;
    QTimer mTimerProcessInformationGain;
    QTimer mTimerStepSimulation;
    QTime mTimeOfLastDenseCloudReduction;
    // QTimer mTimerCheckWayPointSafety; do whenever dense inserted points (if checked)
    //put into glscene! QTimer mTimerHideVisualizationAfterWayPointsComputed;
    ParticleSystem* mParticleSystem;
    cudaError_t mCudaError;

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
    void slotDenseCloudInsertedPoints(PointCloudCuda * const pointCloudSource, const quint32& firstPointToReadFromSrc, quint32 numberOfPointsToCopy);

    void slotSetProcessingStateToIdle();

    // checks waypoint pressure and if higher than threshold, cals slotGenerateWaypoints();
    void slotProcessInformationGain();

    void slotStepSimulation();

    void slotStartWayPointGeneration();

public slots:
    void slotShowUserInterface();
    void slotSetVolumeGlobal(const Box3D volume);
    void slotSetVolumeLocal(const Box3D volume);

    void slotNewScanFused(const float* const points, const quint32& count, const QVector3D* const scannerPosition);

    void slotClearGridOfInformationGain();

    // Used in CUDA flightplanner, might be useful to others. Will be called
    // soon after construction, but maybe after the first call to slotVisualize().
    void slotInitialize();

    // Overridden from base to move vehicle physics object and check for collisions
    void slotVehiclePoseChanged(const Pose *const pose);

    // Called by UI to clear the drawn trajectory
    void slotClearVehicleTrajectory();

    void slotClearWayPointsPassed() {mWayPointsPassed.clear();}


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

    void slotGenerateWaypoints();


signals:
    void message(const LogImportance& importance, const QString& source, const QString& message);
    void particleOpacity(float);

    void processingState(GlScene::FlightPlannerProcessingState);

    void vboInfoGridInformationGain(quint32 vboPressure, Box3D gridBoundingBox, Vector3<quint16> grid);
    void vboInfoParticles(quint32 vboPositions, quint32 particleCount, float particleRadius, Box3D particleSystemBoundingBox);

    // Forwarded from PathPlanner
    void vboInfoGridOccupancy(quint32 vboPressure, Box3D gridBoundingBox, Vector3<quint16> grid);
    void vboInfoGridPathPlanner(quint32 vboPressure, Box3D gridBoundingBox, Vector3<quint16> grid);

    void renderParticles(bool);
    void renderInformationGain(bool);
    void renderOccupancyGrid(bool);
    void renderPathPlannerGrid(bool);

    void volumeLocal(const Box3D*);
    void volumeGlobal(const Box3D*);
    void wayPointListAhead(WayPointList *wpl);
    void wayPointListPassed(WayPointList *wpl);

    // This is emitted whenever FlightPlanner changes waypoints
    void wayPoints(const QList<WayPoint>* const, const WayPointListSource);

    void cameraRotation(float);
    void suggestVisualization();
};

#endif // FLIGHTPLANNERPARTICLES_H
