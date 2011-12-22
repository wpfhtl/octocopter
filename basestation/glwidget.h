#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "flightplannerinterface.h"

#include <QtGui>
#include <QColor>
#include <QtOpenGL>
#include <QGLWidget>
#include <QVector3D>

#include <cuda_gl_interop.h>

#include "openglutilities.h"
#include "octree.h"
#include "pose.h"
//#include "basestation.h"

class FlightPlannerInterface;
//class BaseStation;

class GlWidget : public QGLWidget
{
    Q_OBJECT

//    GLfloat rotQuad;
    Octree *mOctree;
    FlightPlannerInterface *mFlightPlanner;
//    BaseStation *mBaseStation;
//    void setShaders();

    QVector3D mCamLookAt;

    // Timer
    GLint timerId;
    GLfloat t;

    // Mouse Rotations
    QPoint      lastPos;
    QVector3D   camPos;
    GLfloat     rotX, rotY, rotZ;

    // Wheel Scaling
    GLdouble    currentScaling;
    GLdouble    mZoomFactor;

    void mouseDoubleClickEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void zoom(double zoomFactor);
    void drawAxes(const GLfloat& x, const GLfloat& y, const GLfloat& z, const GLfloat& red, const GLfloat& green, const GLfloat& blue) const;
    void drawVehicleVelocity() const;
    QVector3D convertMouseToWorldPosition(const QPoint&);

public:
    GlWidget(QWidget* parent, Octree* octree, FlightPlannerInterface* flightPlanner);
    void moveCamera(const QVector3D &pos);

    // Being called by the octree (as a callback) to visualize contents
//    static void drawPoint(const QVector3D &point);

protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    void timerEvent ( QTimerEvent * event );

signals:
    void initializingInGlContext();
    void visualizeNow();
    void mouseClickedAtWorldPos(Qt::MouseButton, QVector3D);

public slots:
    void slotViewFromTop();
    void slotViewFromSide();
    void slotSaveImage();
};

#endif
