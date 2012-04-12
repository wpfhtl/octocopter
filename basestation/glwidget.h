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

class FlightPlannerInterface;


class GlWidget : public QGLWidget
{
    Q_OBJECT

    Octree *mOctree;
    FlightPlannerInterface *mFlightPlanner;

    QVector3D mCamLookAtOffset;

    // Timer
    int mTimerIdZoom, mTimerIdRotate;
    QDateTime mTimeOfLastExternalUpdate;

    // Mouse Rotations
    QPoint      lastPos;
    QVector3D   camPos;
    GLfloat     rotX, rotY, rotZ;

    // Wheel Zooming. For smooth zooming, mZoomFactorCurrent converges toward mZoomFactorTarget
    GLdouble    mZoomFactorTarget, mZoomFactorCurrent;

    // small helper
    static void glVertexVector(QVector3D a) { glVertex3f(a.x(), a.y(), a.z()); }

    //void mouseDoubleClickEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void zoom(double zoomFactor);
    void drawAxes(const GLfloat& x, const GLfloat& y, const GLfloat& z, const GLfloat& red, const GLfloat& green, const GLfloat& blue) const;
    void drawVehicleVelocity() const;
    void drawVehiclePath() const;
    void drawVehicle() const;
    QVector3D convertMouseToWorldPosition(const QPoint&);

public:
    GlWidget(QWidget* parent, Octree* octree, FlightPlannerInterface* flightPlanner);
    void moveCamera(const QVector3D &pos);

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
    // When this is called, we take note of the time of last external update. Because when zooming/rotating
    // is also active, the redraws caused by those actions overlap with the external redraws, causing
    // superfluous and slow redrawing. So, by only redrawing for rotation/zoom when there hasn't been an
    // external redraw in a while, we can save CPU cycles.
    void slotUpdateView();

    void slotEnableTimerRotation(const bool& enable);
    void slotViewFromTop();
    void slotViewFromSide();
    void slotSaveImage();
};

#endif
