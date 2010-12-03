#include <GL/glew.h>
#include "glwidget.h"

//extern "C" {

#include <GL/gl.h>
#include <GL/glext.h>
//}


GlWidget::GlWidget(Triangulator *triangulator, Octree* octree, FlightPlanner* flightPlanner) :
    QGLWidget((QWidget*)triangulator),
    mOctree(octree),
    mFlightPlanner(flightPlanner)
{
//    rotQuad = 0.0f;
//    startTimer(25);

    mTriangulator = triangulator;

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

    setMinimumSize(640, 480);
}

void GlWidget::initializeGL()
{
    // Set up the rendering context, define display lists etc.:
    glClearColor(0.3, .3, .3, 0.0);
    glEnable(GL_DEPTH_TEST);
//    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
//    glEnable(GL_MULTISAMPLE);
    glEnable(GL_COLOR_MATERIAL);

    // http://nehe.gamedev.net/data/lessons/lesson.asp?lesson=08
    glEnable(GL_BLEND);		// Turn Blending On
//    glDisable(GL_DEPTH_TEST); // ..and dt off, also for blend.

    static GLfloat lightPosition[4] = { -0.5, -5.0, -7.0, -1.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);




//    glShadeModel(GL_SMOOTH);
//    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
//    glClearDepth(1.0f);
//    glEnable(GL_DEPTH_TEST);
//    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    /* we use resizeGL once to set up our initial perspective */
//    resizeGL(width, height);
    /* Reset the rotation angle of our object */
//    rotQuad = 0;
//    glFlush();



}

void GlWidget::resizeGL(int w, int h)
{
    // setup viewport, projection etc.
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(50.0*mZoomFactor, (GLfloat)w/(GLfloat)h, 10, +8000.0);
//    glTranslatef(camPos.x(), camPos.y(), camPos.z());
    glMatrixMode(GL_MODELVIEW);
}

void GlWidget::moveCamera(const QVector3D &pos)
{
    qDebug() << "moveCamera to " << pos;
    camPos = pos;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(130.0, (GLfloat)width()/(GLfloat)height(), 10, +8000.0);
    glTranslatef(camPos.x(), camPos.y(), camPos.z());
    glMatrixMode(GL_MODELVIEW);
}


void GlWidget::paintGL()
{
//    setShaders();

//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//    glLoadIdentity();
//    glTranslatef(0.0f, 0.0f, -7.0f);
//    glRotatef(rotQuad, 0.0f, 1.0f, 0.0f);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    resizeGL(width(),height());
    glLoadIdentity();



//    glTranslatef(camPos.x(), camPos.y(), camPos.z());

//    drawAxes(1.25, 1.1, 1.25, 0.0, 1.0, 0.0);

    //Wheel Scaling
//    glScalef(currentScaling,currentScaling,currentScaling);

    //Timer Animation
//    GLfloat timerScaling = 0.5+pow(cos(0.025*t),2);
//    glScalef(timerScaling,timerScaling,timerScaling);
//    glRotatef(t,1,1,1);

    static bool aimedAtOctreeCenter = false;
//    QVector3D otc;
//    if(mFlightPlanner->mOctree && !aimedAtOctreeCenter)
//    {
//        mCamLookAt = mFlightPlanner->mOctree->root()->center();
//        mCamLookAt = QVector3D(180,80,120);
//        aimedAtOctreeCenter = true;
//        qDebug() << "centering around cloud center" << otc;
//    }
//    else
//    {
//        otc = mOctree->root()->center();
//        otc = mCamLookAt;
//        qDebug() << "centering around 0/0/0" << otc;
//    }

    mCamLookAt = mTriangulator->getCurrentVehiclePosition();

    gluLookAt(0.0, 500.0, 500.0,
              mCamLookAt.x(), mCamLookAt.y(), mCamLookAt.z(),
              0.0, 1.0, 0.0);

    glTranslatef(mCamLookAt.x(), mCamLookAt.y(), mCamLookAt.z());

    // Mouse Move Rotations
    glRotatef(rotX,1.0,0.0,0.0);
    glRotatef(rotY,0.0,1.0,0.0);
    glRotatef(rotZ,0.0,0.0,1.0);

    glTranslatef(-mCamLookAt.x(), -mCamLookAt.y(), -mCamLookAt.z());


//    glTranslatef(-0.0, -100.0, -0.0);


    // Draw vehicle position
    drawSphere(mTriangulator->getCurrentVehiclePosition(), 1.0, 20.0, QColor(20,255,20,100));

    // Draw next waypoint
    drawSphere(mTriangulator->getNextWayPoint(), 1.0, 20.0, QColor(255,255,255,100));

//    glTranslatef(+0.0, +100.0, +0.0);


    drawAxes(10, 10, 10, 1.0, 1.0, 0.0);
    // Draw base plate
//        glDisable(GL_LIGHTING);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE);
    glColor4f(1.0f, 1.0f, 1.0f, 0.2f);
    glBegin(GL_QUADS);
    glVertex3f(1000, 0, -1000);
    glVertex3f(1000, 0, 1000);
    glVertex3f(-1000, 0, 1000);
    glVertex3f(-1000, 0, -1000);
    glEnd();
//    glEnable(GL_LIGHTING);


//    mOctree->drawGl();
    glLineWidth(1);
    glColor4f(1.0f, 1.0f, 0.0f, 1.0f);
    glBegin(GL_POINTS);
    mOctree->handlePoints();
    glEnd();

    // draw
    mFlightPlanner->visualize();
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
}

void GlWidget::mouseMoveEvent(QMouseEvent *event)
{
    GLdouble DX = GLdouble(event->x()-lastPos.x())/width();
    GLdouble DY = GLdouble(event->y()-lastPos.y())/height();

    if(event->buttons() & Qt::LeftButton)
    {
        rotX += 180*DY;
        rotY += 180*DX;
        updateGL();
    }
    else if(event->buttons() & Qt::RightButton)
    {
        rotX += 180*DY;
        rotZ += 180*DX;
        updateGL();
    }
    else if(event->buttons() & Qt::MiddleButton)
    {
        mCamLookAt.setZ(mCamLookAt.z() + 180*DY);
        mCamLookAt.setX(mCamLookAt.x() + 180*DX);
        updateGL();
    }

    lastPos = event->pos();

    rotX = fmod(rotX, 360.0);
    rotY = fmod(rotY, 360.0);
    rotZ = fmod(rotZ, 360.0);

    qDebug() << rotX << rotY << rotZ;
}

void GlWidget::wheelEvent(QWheelEvent *event)
{
    int numDegrees = event->delta()/32;
    double numSteps = numDegrees/30.0;
    double factor = event->delta()/93.0;
//    mZoomFactor *= factor;
    qDebug() << factor << mZoomFactor;

    factor>0? mZoomFactor += 0.05 : mZoomFactor -= 0.05;

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

void GlWidget::drawSphere(const QVector3D &pos, const float radius, const int subdivisions, const QColor color)
{
    GLUquadricObj *quadric = gluNewQuadric();
    gluQuadricNormals(quadric, GLU_SMOOTH);

    glPushMatrix();
    glTranslatef(pos.x(), pos.y(), pos.z());
    glColor4f(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    gluSphere(quadric, radius, subdivisions, subdivisions);
    glPopMatrix();

    gluDeleteQuadric(quadric);
}

void GlWidget::drawSphere(const QVector3D &pos)
{
    drawSphere(pos, 1.0, 15, QColor(255,0,0));
}

void GlWidget::drawPoint(const QVector3D &point)
{
    glVertex3f(point.x(), point.y(), point.z());
}
