#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include "pointcloud.h"
#include "glwindow.h"
#include "cudahelper.h"

GlWindow::GlWindow(GlScene* glScene, QWindow* parent) :
    QWindow(parent)
{
    mIsInitialized = false;

    // Tell Qt we will use OpenGL for this window
    setSurfaceType(QWindow::OpenGLSurface);

    // Specify the format and create platform-specific surface
    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setMajorVersion(OPENGL_VERSION_MAJOR);
    format.setMinorVersion(OPENGL_VERSION_MINOR);
    format.setSamples(4);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setOption(QSurfaceFormat::DebugContext);
//    format.setProfile(QSurfaceFormat::CompatibilityProfile);
    setFormat(format);
    create();

    // Create an OpenGL context
    mOpenGlContext = new QOpenGLContext;
    mOpenGlContext->setFormat(format);
    mOpenGlContext->create();

    mGlScene = glScene;
    connect(mGlScene, &GlScene::suggestVisualization, this, &GlWindow::slotRenderLater);

    mBackgroundBrightness = 0.2f;
    mFramesRenderedThisSecond = 0;
    mZoomFactorCurrent = 0.5;
    mZoomFactorTarget = 0.5;

    // Timed Animation
    mRotationPerFrame = 0.0f;
    mViewZooming = false;

    mTimerUpdate = new QTimer(this);
    mTimerUpdate->setInterval(1000 / 50);
    connect(mTimerUpdate, &QTimer::timeout, this, &GlWindow::slotRenderLater);
}

GlWindow::~GlWindow()
{
    mOpenGlContext->deleteLater();
    mOpenGlDebugLogger->deleteLater();
    mTimerUpdate->deleteLater();
}

void GlWindow::slotInitialize()
{
    if(mIsInitialized) return;

    qDebug() << __PRETTY_FUNCTION__;

    mIsInitialized = true;

    mOpenGlContext->makeCurrent(this);

    mOpenGlDebugLogger = new QOpenGLDebugLogger(this);
    connect(mOpenGlDebugLogger, SIGNAL(messageLogged(QOpenGLDebugMessage)), this, SLOT(slotOpenGlDebugMessage(QOpenGLDebugMessage)), Qt::DirectConnection);
    if(mOpenGlDebugLogger->initialize())
    {
        //mOpenGlDebugLogger->startLogging(QOpenGLDebugLogger::SynchronousLogging);
        mOpenGlDebugLogger->startLogging(QOpenGLDebugLogger::AsynchronousLogging);
        mOpenGlDebugLogger->enableMessages();
    }

    if(!initializeOpenGLFunctions())
    {
        qDebug() << __PRETTY_FUNCTION__ << "couldn't initialize OpenGL Core context, quitting.";
        exit(1);
    }

    CudaHelper::initializeCuda();

    OpenGlUtilities::initialize();

    glClearColor(mBackgroundBrightness, mBackgroundBrightness, mBackgroundBrightness, 0.0f);

    mGlScene->initialize();

    emit initializingInGlContext();
}

void GlWindow::slotRenderNow()
{
    if(!isExposed()) return;

    //qDebug() << __PRETTY_FUNCTION__;

    mOpenGlContext->makeCurrent(this);
    mFramesRenderedThisSecond++;

    const QTime currentTime = QTime::currentTime();
    if(mTimeOfLastRender.second() != currentTime.second()) mFramesRenderedThisSecond = 0;
    mTimeOfLastRender = currentTime;

    if(fabs(mRotationPerFrame) > 0.00001) mGlScene->rotateCamera(mRotationPerFrame, 0.0f, 0.0f);

    // Here we make mZoomFactorCurrent converge to mZoomFactorTarget for smooth zooming
    float step = 0.0f;
    if(mZoomFactorTarget > (mZoomFactorCurrent + 0.0001f))
        step = (mZoomFactorTarget - mZoomFactorCurrent) / 10.0f;
    else if((mZoomFactorTarget + 0.0001f) < mZoomFactorCurrent)
        step = -(mZoomFactorCurrent - mZoomFactorTarget) / 10.0f;

    if(mViewZooming && fabs(step) > 0.00001f)
    {
        mZoomFactorCurrent += step;
        mViewZooming = true;
        mGlScene->setCameraZoom(mZoomFactorCurrent);
        mGlScene->slotUpdateMatrixCameraToClip(width(), height());
    }
    else
    {
        // Stop zooming, lower framerate
        mTimerUpdate->setInterval(1000 / 50);
        mViewZooming = false;
    }

    // Clear color buffer and depth buffer
    glClearColor(mBackgroundBrightness, mBackgroundBrightness, mBackgroundBrightness, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    mGlScene->render();

    mOpenGlContext->swapBuffers(this);
}

void GlWindow::slotSetCameraRotation(const float rotation)
{
    mRotationPerFrame = rotation;

    // Enable the timer if we want to rotate and its not running already
    if(fabs(mRotationPerFrame) > 0.00001 && !mViewZooming)
        mTimerUpdate->start();
}

void GlWindow::mousePressEvent(QMouseEvent *event)
{
    mLastMousePosition = event->pos();
}

void GlWindow::mouseMoveEvent(QMouseEvent *event)
{
    //qDebug() << __PRETTY_FUNCTION__;

    const float deltaX = -float(event->x()-mLastMousePosition.x())/width();
    const float deltaY = -float(event->y()-mLastMousePosition.y())/height();

    if(event->buttons() & Qt::LeftButton)
    {
        mGlScene->rotateCamera(180.0f * deltaX, 180 * deltaY, 0.0f);
    }
    else if(event->buttons() & Qt::MiddleButton)
    {
        mGlScene->moveCameraTarget(180.0f * deltaY, 0.0f, 180.0f * deltaX);
    }

    mLastMousePosition = event->pos();
    slotRenderLater();
}

void GlWindow::wheelEvent(QWheelEvent *event)
{
    //qDebug() << __PRETTY_FUNCTION__;
    event->delta() > 0 ? mZoomFactorTarget *= 1.5f : mZoomFactorTarget *= 0.5f;
    mZoomFactorTarget = qBound(0.002f, (float)mZoomFactorTarget, 1.0f);
    mViewZooming = true;
    mTimerUpdate->setInterval(1000 / 50);
    mTimerUpdate->start();
    slotRenderLater();
}

void GlWindow::slotRenderLater()
{
    if(mTimeOfLastRender.msecsTo(QTime::currentTime()) > mTimerUpdate->interval())
    {
        slotRenderNow();

        if(!fabs(mRotationPerFrame) > 0.00001 && !mViewZooming)
            mTimerUpdate->stop();
    }
    else
    {
        if(!mTimerUpdate->isActive())
            mTimerUpdate->start();
    }
}

void GlWindow::exposeEvent(QExposeEvent *event)
{
    //qDebug() << __PRETTY_FUNCTION__;
    Q_UNUSED(event);
    if(isExposed()) slotRenderNow();
}

void GlWindow::resizeEvent(QResizeEvent *event)
{
    Q_UNUSED(event);

    if(!mIsInitialized) slotInitialize();

    const int w = width();
    const int h = height();

    //qDebug() << __PRETTY_FUNCTION__ << "size:" << w << h;

    // setup viewport, projection etc.
    mOpenGlContext->makeCurrent(this);
    glViewport(0, 0, w, h);
    mGlScene->slotUpdateMatrixCameraToClip(w, h);

    if(isExposed()) slotRenderNow();
}

void GlWindow::slotViewFromTop()
{
    mGlScene->setCameraRotation(50, -17.71, 0.0f);
    mZoomFactorCurrent = 0.6;
    mZoomFactorTarget = 0.6;
    mGlScene->setCameraZoom(mZoomFactorCurrent);
    slotRenderNow();
}

void GlWindow::slotViewFromSide()
{
    mGlScene->setCameraRotation(-23, 3.5, 0.0f);
    mZoomFactorCurrent = 0.6;
    mZoomFactorTarget = 0.6;
    mGlScene->setCameraZoom(mZoomFactorCurrent);
    slotRenderNow();
}

void GlWindow::slotOpenGlDebugMessage(const QOpenGLDebugMessage message)
{
    if(message.severity() != QOpenGLDebugMessage::Severity::LowSeverity)
    {
        qDebug() << message;
    }
}
