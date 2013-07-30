#ifndef FLIGHTPLANNERINTERFACE_H
#define FLIGHTPLANNERINTERFACE_H

#include <QMap>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLFunctions_4_3_Core>
#include "pointcloud.h"
#include "openglutilities.h"
#include "shaderprogram.h"
#include "waypointlist.h"
#include <pose.h>

class Pose;
class GlWindow;
class BaseStation;

class FlightPlannerInterface : public QObject, protected OPENGL_FUNCTIONS_CLASS
{
    Q_OBJECT
public:

protected:

public:
    // Here, basestation passes its own pointcloud. Its up to the implementation to use it.
    FlightPlannerInterface(BaseStation* basestation, GlWindow *glWidget, PointCloud* pointcloud);
    virtual ~FlightPlannerInterface();

//    void setGlWidget(GlWidget* glWidget) {mGlWidget = glWidget;}

//    const QVector<Pose>& getVehiclePoses() { return mVehiclePoses; }



//    virtual void keyPressEvent(QKeyEvent* event) = 0;

public slots:
    // Called by LogPlayer or RoverConnection when new scanData arrives. @points must be float4!

};

#endif // FLIGHTPLANNERINTERFACE_H
