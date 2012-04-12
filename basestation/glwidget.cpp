#include <GL/glew.h>
#include "glwidget.h"

//extern "C" {

//#include <GL/gl.h>
//#include <GL/glext.h>
//}


GlWidget::GlWidget(QWidget* parent, Octree* octree, FlightPlannerInterface* flightPlanner) :
    QGLWidget(parent),
    mOctree(octree),
    mFlightPlanner(flightPlanner)
{
    QGLFormat fmt;
    fmt.setSamples(2);
    fmt.setSampleBuffers(true);
    QGLFormat::setDefaultFormat(fmt);
    setFormat(fmt);

    mCamLookAtOffset = QVector3D(0.0, 0.0, 0.0);

    mZoomFactorCurrent = 0.5;
    mZoomFactorTarget = 0.5;

    //Mouse Move Rotations
    rotX = 0;
    rotY = 0;
    rotZ = 0;

    //Timer Animation
    mTimerIdZoom = 0;
    mTimerIdRotate = 0;

    mTimeOfLastExternalUpdate = QDateTime::currentDateTime();

    setMinimumSize(320, 240);
}

void GlWidget::initializeGL()
{
    glewInit();

    // Give e.g. FlightPlannerCuda a chance to initialize CUDA in a GL context
    emit initializingInGlContext();

    // needed to convert mous epos to world pos, see
    // http://stackoverflow.com/questions/3089271/converting-mouse-position-to-world-position-opengl
    glEnable(GL_DEPTH);

    glShadeModel(GL_SMOOTH);						// Enable Smooth Shading
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);					// Black Background
    glClearColor(1.0f, 1.0f, 1.0f, 0.0f);					// White Background
    glClearColor(0.3f, 0.3f, 0.3f, 0.0f);					// Gray  Background
    glClearDepth(1.0f);							// Depth Buffer Setup
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);					// Set Line Antialiasing

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

//    glEnable(GL_BLEND);							// Enable Blending
//    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);			// Type Of Blending To Use
//    glBlendFunc( GL_SRC_ALPHA, GL_ONE );
    glBlendFunc( GL_ZERO, GL_ONE_MINUS_SRC_ALPHA );
//    glBlendFunc( GL_SRC_ALPHA_SATURATE, GL_ONE );
//    glBlendFunc(GL_ONE_MINUS_DST_ALPHA,GL_DST_ALPHA);
    glEnable(GL_DEPTH_TEST); // also needed for CUDA particles
}

void GlWidget::resizeGL(int w, int h)
{
//    qDebug() << "GlWidget::resizeGL(): resizing gl viewport to" << w << h;
    // setup viewport, projection etc.
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
//    gluPerspective(50.0*mZoomFactorCurrent, (GLfloat)w/(GLfloat)h, 10, +8000.0);
    glOrtho(-w/2 * mZoomFactorCurrent, w/2 * mZoomFactorCurrent, -h/2 * mZoomFactorCurrent, h/2 * mZoomFactorCurrent, 1, 10000);
    glTranslatef(camPos.x(), camPos.y(), camPos.z());
    glMatrixMode(GL_MODELVIEW);
}

void GlWidget::moveCamera(const QVector3D &pos)
{
//    qDebug() << "moveCamera to " << pos;
    camPos = pos;
}

void GlWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Here we make mZoomFactorCurrent converge to mZoomFactorTarget for smooth zooming
    float step = 0.0f;
    if(mZoomFactorTarget > (mZoomFactorCurrent + 0.0001f))
        step = (mZoomFactorTarget - mZoomFactorCurrent) / 10.0f;
    else if((mZoomFactorTarget + 0.0001f) < mZoomFactorCurrent)
        step = -(mZoomFactorCurrent - mZoomFactorTarget) / 10.0f;

    if(fabs(step) > 0.00001f)
    {
        if(mTimerIdZoom == 0) mTimerIdZoom = startTimer(20);
        mZoomFactorCurrent += step;
        resizeGL(width(),height()); // this sets up a new view with the new zoomFactor
    }
    else if(mTimerIdZoom != 0)
    {
        killTimer(mTimerIdZoom);
        mTimerIdZoom = 0;
    }

    glLoadIdentity();

    const QVector3D vehiclePosition = mFlightPlanner->getLastKnownVehiclePose().getPosition();
    QVector3D camLookAt = mCamLookAtOffset + vehiclePosition;
    QVector3D min, max;
    mFlightPlanner->getScanVolume(min, max);
//    mCamLookAt = min + (max - min)/2.0;

    gluLookAt(0.0, 500.0, 500.0,
              camLookAt.x(), camLookAt.y(), camLookAt.z(),
              0.0, 1.0, 0.0);

    glTranslatef(camLookAt.x(), camLookAt.y(), camLookAt.z());

    // Mouse Move Rotations
    glRotatef(rotX,1.0,0.0,0.0);
    glRotatef(rotY,0.0,1.0,0.0);
    glRotatef(rotZ,0.0,0.0,1.0);

    glTranslatef(-camLookAt.x(), -camLookAt.y(), -camLookAt.z());

    // Draw base plate for unProjecting
    /*
    glEnable(GL_BLEND);
    glDisable(GL_LIGHTING);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glBlendFunc(GL_CONSTANT_COLOR, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(1.0f, 1.0f, 1.0f, 0.0f);
    glBegin(GL_QUADS);
    glVertex3f(1000, min.y()+15, -1000);
    glVertex3f(1000, min.y()+15, 1000);
    glVertex3f(-1000, min.y()+15, 1000);
    glVertex3f(-1000, min.y()+15, -1000);
    glEnd();
    glEnable(GL_LIGHTING);
    glDisable(GL_BLEND);
    */

    drawAxes(10, 10, 10, 0.8, 0.8, 0.8);

    drawVehiclePath();
    drawVehicle();
    drawVehicleVelocity();

    glDisable(GL_LIGHTING);
    glPointSize(1);
    glColor4f(.5f, .5f, .5f, 0.7f);
    glBegin(GL_POINTS);
    mOctree->handlePoints();
    glEnd();
    glEnable(GL_LIGHTING);

    emit visualizeNow();
}

void GlWidget::drawVehiclePath() const
{
    const QVector<Pose> path = mFlightPlanner->getVehiclePoses();

    glLineWidth(2);
    glBegin(GL_LINE_STRIP);
//    glBegin(GL_POINTS);
    glColor3f(0.0, 1.0, 0.0);
    for(int i = 0; i < path.size(); i++)
        glVertexVector(path.at(i).getPosition());
    glEnd();
}

void GlWidget::drawVehicle() const
{
    // draw vehicle velocity as vector
    const Pose pose = mFlightPlanner->getLastKnownVehiclePose();
    const QVector3D vehiclePosition = pose.getPosition();

    const QVector3D armFront = pose.getOrientation().rotatedVector(QVector3D(0.0, 0.0, -0.4));
    const QVector3D armBack = pose.getOrientation().rotatedVector(QVector3D(0.0, 0.0, 0.4));
    const QVector3D armLeft = pose.getOrientation().rotatedVector(QVector3D(-0.4, 0.0, 0.0));
    const QVector3D armRight = pose.getOrientation().rotatedVector(QVector3D(0.4, 0.0, 0.0));

    const QVector3D landingLegFront = pose.getOrientation().rotatedVector(QVector3D(0.0, -0.2, -0.2));
    const QVector3D landingLegBack = pose.getOrientation().rotatedVector(QVector3D(0.0, -0.2, 0.2));
    const QVector3D landingLegLeft = pose.getOrientation().rotatedVector(QVector3D(-0.2, -0.2, 0.0));
    const QVector3D landingLegRight = pose.getOrientation().rotatedVector(QVector3D(0.2, -0.2, 0.0));

    glTranslatef(0.0f, 0.2f, 0.0f);

    OpenGlUtilities::drawSphere(vehiclePosition, 0.03, 10.0, QColor(80,80,80,200));
    glDisable(GL_LIGHTING);


    glLineWidth(3);
    glBegin(GL_LINES);
    glColor3f(1.0, 0.2, 0.2);

    // draw arms
    glVertexVector(vehiclePosition);
    glVertexVector(vehiclePosition + armFront);

    glColor3f(0.4, 0.4, 0.4);
    glVertexVector(vehiclePosition);
    glVertexVector(vehiclePosition + armBack);

    glVertexVector(vehiclePosition);
    glVertexVector(vehiclePosition + armLeft);

    glVertexVector(vehiclePosition);
    glVertexVector(vehiclePosition + armRight);

    glColor3f(0.7, 0.7, 0.7);

    // draw landing legs
    glVertexVector(vehiclePosition + armFront * 0.5);
    glVertexVector(vehiclePosition + landingLegFront);

    glVertexVector(vehiclePosition + armBack * 0.5);
    glVertexVector(vehiclePosition + landingLegBack);

    glVertexVector(vehiclePosition + armLeft * 0.5);
    glVertexVector(vehiclePosition + landingLegLeft);

    glVertexVector(vehiclePosition + armRight * 0.5);
    glVertexVector(vehiclePosition + landingLegRight);

    glEnd();
    glEnable(GL_LIGHTING);
}

void GlWidget::drawVehicleVelocity() const
{
    // draw vehicle velocity as vector
    const QVector3D vehiclePosition = mFlightPlanner->getLastKnownVehiclePose().getPosition();
    const QVector3D vehicleVelocity = mFlightPlanner->getCurrentVehicleVelocity();
    glDisable(GL_LIGHTING);
    glColor3f(0.0, 1.0, 0.0);
    glLineWidth(1);
    glBegin(GL_LINES);
    glVertex3f(vehiclePosition.x(), vehiclePosition.y(), vehiclePosition.z());
    glVertex3f(vehiclePosition.x()+vehicleVelocity.x(), vehiclePosition.y()+vehicleVelocity.y(), vehiclePosition.z()+vehicleVelocity.z());
    glEnd();
    glEnable(GL_LIGHTING);
}

void GlWidget::drawAxes(
        const GLfloat& x, const GLfloat& y, const GLfloat& z,
        const GLfloat& red, const GLfloat& green, const GLfloat& blue) const
{
    glDisable(GL_LIGHTING);
    glLineWidth(2);
    glBegin(GL_LINES);
    // X
    glColor3f(red,green,blue);
    glVertex3f( -x,0.0f,0.0f); glVertex3f(0.0f,0.0f,0.0f);
    glColor3f(1.0f,green,blue);
    glVertex3f(0.0f,0.0f,0.0f); glVertex3f(  x,0.0f,0.0f);

    // Y
    glColor3f(red,green,blue);
    glVertex3f(0.0f,  -y,0.0f); glVertex3f(0.0f,0.0f,0.0f);
    glColor3f(red,1.0f,blue);
    glVertex3f(0.0f,0.0f,0.0f); glVertex3f(0.0f,   y,0.0f);

    // Y
    glColor3f(red,green,blue);
    glVertex3f(0.0f,0.0f,  -z); glVertex3f(0.0f,0.0f,0.0f);
    glColor3f(red,green,1.0f);
    glVertex3f(0.0f,0.0f,0.0f); glVertex3f(0.0f,0.0f,   z);
    glEnd();
    glEnable(GL_LIGHTING);
}

/*//Mouse Handlers
void GlWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
    if(event->button() != Qt::LeftButton)
        return;

    if(timerId == 0)
    {
        timerId = startTimer(10);
    }
    else
    {
        killTimer(timerId);
        timerId = 0;
    }
}*/

void GlWidget::slotEnableTimerRotation(const bool& enable)
{
    if(enable && mTimerIdRotate == 0)
    {
        qDebug() << "GlWidget::slotEnableTimerRotation(): enabling timer";
        mTimerIdRotate = startTimer(30);
    }
    else if(!enable && mTimerIdRotate != 0)
    {
        qDebug() << "GlWidget::slotEnableTimerRotation(): disabing timer";
        killTimer(mTimerIdRotate);
        mTimerIdRotate = 0;
    }
}

void GlWidget::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();

    emit mouseClickedAtWorldPos(event->button(), convertMouseToWorldPosition(event->pos()));
}

void GlWidget::mouseMoveEvent(QMouseEvent *event)
{
    float DX = float(event->x()-lastPos.x())/width();
    float DY = float(event->y()-lastPos.y())/height();

    if(event->buttons() & Qt::LeftButton)
    {
        rotX += 180 * DY;
        rotY += 180 * DX;
    }
    else if(event->buttons() & Qt::RightButton)
    {
        rotX += 180*DY;
        rotZ += 180*DX;
    }
    else if(event->buttons() & Qt::MiddleButton)
    {
        mCamLookAtOffset.setZ(mCamLookAtOffset.z() + 180*DY);
        mCamLookAtOffset.setX(mCamLookAtOffset.x() + 180*DX);
    }

    lastPos = event->pos();

    rotX = fmod(rotX, 360.0);
    rotY = fmod(rotY, 360.0);
    rotZ = fmod(rotZ, 360.0);

//    qDebug() << "mCamLookAtOffset: " << mCamLookAtOffset << "rotXYZ:" << rotX << rotY << rotZ;

    update();
}

void GlWidget::wheelEvent(QWheelEvent *event)
{
    event->delta() > 0 ? mZoomFactorTarget *= 1.5f : mZoomFactorTarget *= 0.5f;
    mZoomFactorTarget = qBound(0.002f, (float)mZoomFactorTarget, 1.0f);
//    qDebug() << "zoomFactor" << mZoomFactor;
    update();
}

void GlWidget::slotUpdateView()
{
    mTimeOfLastExternalUpdate = QDateTime::currentDateTime();
    update();
}

void GlWidget::timerEvent ( QTimerEvent * event )
{
    if(event->timerId() == mTimerIdRotate)
    {
//        qDebug() << "GlWidget::timerEvent(): rotating...";
        rotY -= 180 * 0.001;
    }
    else if(event->timerId() == mTimerIdZoom)
    {
//        qDebug() << "GlWidget::timerEvent(): zooming...";
    }

    // We should redraw for both zooming and rotating. But if the helicopter is flying, that will mean
    // two sources causing redraws all the time, which is slow. So, only update if there was recent update.
    const int interval = mTimeOfLastExternalUpdate.msecsTo(QDateTime::currentDateTime());
    if(interval > 60)
    {
//        qDebug() << "GlWidget::timerEvent(): last external update was" << interval << "ms ago, updating";
        update();
    }
}

void GlWidget::slotViewFromTop()
{
    rotX = 49.58;
    rotY = -17.71;
    rotZ = 19.67;
    mZoomFactorCurrent = 0.6;
    mZoomFactorTarget = 0.6;
    updateGL();
}

void GlWidget::slotViewFromSide()
{
    rotX = -34.0;
    rotY = 0.25;
    rotZ = -15.5;

    rotX = -23.0;
    rotY = -3.54;
    rotZ = -10.7;

    mZoomFactorCurrent = 0.6;
    mZoomFactorTarget = 0.6;
    updateGL();
}

void GlWidget::slotSaveImage()
{
    renderPixmap(0, 0, true).save(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz").prepend("snapshot-").append(".png"));
}

QVector3D GlWidget::convertMouseToWorldPosition(const QPoint& point)
{
    return QVector3D();
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posX, posY, posZ;

    glLoadIdentity();
    const QVector3D vehiclePosition = mFlightPlanner->getLastKnownVehiclePose().getPosition();
    mCamLookAtOffset = vehiclePosition;
    QVector3D min, max;
    mFlightPlanner->getScanVolume(min, max);
    mCamLookAtOffset = min + (max - min)/2.0;

    gluLookAt(0.0, 500.0, 500.0,
              mCamLookAtOffset.x(), mCamLookAtOffset.y(), mCamLookAtOffset.z(),
              0.0, 1.0, 0.0);

    glTranslatef(mCamLookAtOffset.x(), mCamLookAtOffset.y(), mCamLookAtOffset.z());

    // Mouse Move Rotations
    glRotatef(rotX,1.0,0.0,0.0);
    glRotatef(rotY,0.0,1.0,0.0);
    glRotatef(rotZ,0.0,0.0,1.0);

    glTranslatef(-mCamLookAtOffset.x(), -mCamLookAtOffset.y(), -mCamLookAtOffset.z());

    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );

    winX = (float)point.x();
    winY = (float)viewport[3] - (float)point.y();
    glReadPixels( point.x(), int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );

    gluUnProject(winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);

//    qDebug() << "world pos of mouse" << point.x() << point.y() << "is" << posX << posY << posZ;

    return QVector3D(posX, posY, posZ);
}
