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
//    rotQuad = 0.0f;
//    startTimer(25);
    QGLFormat fmt;
    fmt.setSamples(2);
    fmt.setSampleBuffers(true);
    QGLFormat::setDefaultFormat(fmt);
    setFormat(fmt);


//    mBaseStation = baseStation;

    //Wheel Scaling
    currentScaling = 2.0;
    mZoomFactor = 1.0;

    //Mouse Move Rotations
    rotX = 0;
    rotY = 0;
    rotZ = 0;

    //Timer Animation
    timerId = 0;
    t = 0.0;

//    camPos = QVector3D(0, 500, -500);

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
    glEnable(GL_DEPTH_TEST);
}

void GlWidget::resizeGL(int w, int h)
{
    // setup viewport, projection etc.
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
//    gluPerspective(50.0*mZoomFactor, (GLfloat)w/(GLfloat)h, 10, +8000.0);
//    gluOrtho2D(-w/2.0, w/2.0, -h/2.0, h/2.0);
//    gluOrtho2D(-200.0, 200.0, -200.0, 200.0);
    glOrtho(-150 * mZoomFactor, 150 * mZoomFactor, -150 * mZoomFactor, 150 * mZoomFactor, 1, 10000);
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
//    setShaders();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    resizeGL(width(),height());
    glLoadIdentity();

    const QVector3D vehiclePosition = mFlightPlanner->getLastKnownVehiclePose().position;
    mCamLookAt = vehiclePosition;
    QVector3D min, max;
    mFlightPlanner->getScanVolume(min, max);
    mCamLookAt = min + (max - min)/2.0;

    gluLookAt(0.0, 500.0, 500.0,
              mCamLookAt.x(), mCamLookAt.y(), mCamLookAt.z(),
              0.0, 1.0, 0.0);

    glTranslatef(mCamLookAt.x(), mCamLookAt.y(), mCamLookAt.z());

    // Mouse Move Rotations
    glRotatef(rotX,1.0,0.0,0.0);
    glRotatef(rotY,0.0,1.0,0.0);
    glRotatef(rotZ,0.0,0.0,1.0);

    glTranslatef(-mCamLookAt.x(), -mCamLookAt.y(), -mCamLookAt.z());

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

    // Draw vehicle position
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);			// Type Of Blending To Use
    glDisable(GL_LIGHTING);
    OpenGlUtilities::drawSphere(mFlightPlanner->getLastKnownVehiclePose().position, 1.0, 20.0, QColor(0,0,0,200));
    glEnable(GL_LIGHTING);

    drawAxes(20, 20, 20, 1.0, 1.0, 0.0);

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

void GlWidget::drawVehicleVelocity() const
{
    // draw vehicle velocity as vector
    const QVector3D vehiclePosition = mFlightPlanner->getLastKnownVehiclePose().position;
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
//    glDisable(GL_LIGHTING);
    glColor3f(red,green,blue);
    glLineWidth(1);
    glBegin(GL_LINES);
    glVertex3f(-x,0.0,0.0); glVertex3f( x,0.0,0.0);
    glVertex3f(0.0,-y,0.0); glVertex3f(0.0, y,0.0);
    glVertex3f(0.0,0.0,-z); glVertex3f(0.0,0.0, z);
    glEnd();
//    glEnable(GL_LIGHTING);
}

//Mouse Handlers
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
}

void GlWidget::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();

    emit mouseClickedAtWorldPos(event->button(), convertMouseToWorldPosition(event->pos()));
}

void GlWidget::mouseMoveEvent(QMouseEvent *event)
{
    GLdouble DX = GLdouble(event->x()-lastPos.x())/width();
    GLdouble DY = GLdouble(event->y()-lastPos.y())/height();

    if(event->buttons() & Qt::LeftButton)
    {
        rotX += 180*DY;
        rotY += 180*DX;
//        updateGL();
    }
    else if(event->buttons() & Qt::RightButton)
    {
        rotX += 180*DY;
        rotZ += 180*DX;
//        updateGL();
    }
    else if(event->buttons() & Qt::MiddleButton)
    {
        mCamLookAt.setZ(mCamLookAt.z() + 180*DY);
        mCamLookAt.setX(mCamLookAt.x() + 180*DX);
    }

    lastPos = event->pos();

    rotX = fmod(rotX, 360.0);
    rotY = fmod(rotY, 360.0);
    rotZ = fmod(rotZ, 360.0);

    //qDebug() << "rotXYZ:" << rotX << rotY << rotZ;

    updateGL();
}

void GlWidget::wheelEvent(QWheelEvent *event)
{
    int numDegrees = event->delta()/32;
    double numSteps = numDegrees/30.0;
    double factor = event->delta()/93.0;
//    mZoomFactor *= factor;

    factor > 0 ? mZoomFactor += 0.05 : mZoomFactor -= 0.05;
//    qDebug() << "zoomFactor" << mZoomFactor;

//    if(factor > 0)
//        moveCamera(camPos * factor);
//    else
//        moveCamera(camPos / (-factor));
    //moveCamera(camPos + 100);
    //zoom(pow(ZoomFactor, numSteps));
    //paintGL();
    update();
//    zoom(1);
}

//void GlWidget::zoom(double zoomFactor)
//{
//    currentScaling *= zoomFactor;
//    updateGL();
//}

void GlWidget::timerEvent ( QTimerEvent * event )
{
    if(event->timerId() == timerId)
    {
        t += 0.25; updateGL();
    }
}
/*
void GlWidget::drawPoint(const QVector3D &point)
{
//    glPointSize(1.0);

    glVertex3f(point.x(), point.y(), point.z());
}
*/

void GlWidget::slotViewFromTop()
{
    rotX = 49.58;
    rotY = -17.71;
    rotZ = 19.67;
    mZoomFactor = 0.4;
    mZoomFactor = 0.6;
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

    mZoomFactor = 0.4;
    mZoomFactor = 0.6;
    updateGL();
}

void GlWidget::slotSaveImage()
{
    renderPixmap(0, 0, true).save(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz").prepend("snapshot-").append(".png"));
}

QVector3D GlWidget::convertMouseToWorldPosition(const QPoint& point)
{
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posX, posY, posZ;

    glLoadIdentity();
    const QVector3D vehiclePosition = mFlightPlanner->getLastKnownVehiclePose().position;
    mCamLookAt = vehiclePosition;
    QVector3D min, max;
    mFlightPlanner->getScanVolume(min, max);
    mCamLookAt = min + (max - min)/2.0;

    gluLookAt(0.0, 500.0, 500.0,
              mCamLookAt.x(), mCamLookAt.y(), mCamLookAt.z(),
              0.0, 1.0, 0.0);

    glTranslatef(mCamLookAt.x(), mCamLookAt.y(), mCamLookAt.z());

    // Mouse Move Rotations
    glRotatef(rotX,1.0,0.0,0.0);
    glRotatef(rotY,0.0,1.0,0.0);
    glRotatef(rotZ,0.0,0.0,1.0);

    glTranslatef(-mCamLookAt.x(), -mCamLookAt.y(), -mCamLookAt.z());

    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );

    winX = (float)point.x();
    winY = (float)viewport[3] - (float)point.y();
    glReadPixels( point.x(), int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );

    gluUnProject( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);

    qDebug() << "world pos of mouse" << point.x() << point.y() << "is" << posX << posY << posZ;

    return QVector3D(posX, posY, posZ);
}
