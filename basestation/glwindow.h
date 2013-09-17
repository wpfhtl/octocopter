#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QtGui>
#include <QColor>
#include <QWindow>
#include <QVector>
#include <QVector3D>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLFunctions_4_3_Core>
#include <QOpenGLDebugLogger>

#include <cuda_gl_interop.h>

#include "glscene.h"
#include "shaderprogram.h"

class PointCloud;

class FlightPlannerInterface;


class GlWindow : public QWindow, protected OPENGL_FUNCTIONS_CLASS
{
    Q_OBJECT

    QOpenGLContext* mOpenGlContext;
    GlScene* mGlScene;
    QTime mTimeOfLastRender;
    QTimer* mTimerUpdate;

    // Wheel Zooming. For smooth zooming, mZoomFactorCurrent converges toward mZoomFactorTarget
    GLdouble    mZoomFactorTarget, mZoomFactorCurrent;
    float mRotationPerFrame;
    bool mViewZooming;
    QPoint      mLastMousePosition;
    QOpenGLDebugLogger* mOpenGlDebugLogger;
    quint32 mFramesRenderedThisSecond;

    void zoom(double zoomFactor);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void exposeEvent(QExposeEvent *event);
    void resizeEvent(QResizeEvent *event);

private slots:
    void slotOpenGlDebugMessage(const QOpenGLDebugMessage message);

public:
    GlWindow(GlScene *glScene, QWindow *parent = 0);
    ~GlWindow();

    float mBackgroundBrightness;

signals:
    void initializingInGlContext();
    void message(const LogImportance& importance, const QString& source, const QString& message);

public slots:
    // This will init opengl and emit initializingInGlContext()
    void slotInitialize();

    void slotSetCameraRotation(const float rotation);

    // When this is called, we take note of the time of last external update. Because when zooming/rotating
    // is also active, the redraws caused by those actions overlap with the external redraws, causing
    // superfluous and slow redrawing. So, by only redrawing for rotation/zoom when there hasn't been an
    // external redraw in a while, we can save CPU cycles.
    void slotRenderLater();
    void slotRenderNow();

    void slotViewFromTop();
    void slotViewFromSide();
};

#endif
