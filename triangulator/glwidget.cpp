#include <GL/glew.h>
#include "glwidget.h"

//extern "C" {

#include <GL/gl.h>
#include <GL/glext.h>
//}


GlWidget::GlWidget(QWidget *parent, Octree* octree) :
        QGLWidget(parent),
        mOctree(octree)
{
//    rotQuad = 0.0f;
//    startTimer(25);

    //Wheel Scaling
    currentScaling = 2.0;
    ZoomFactor = 0.3;

    //Mouse Move Rotations
    rotX = 0;
    rotY = 0;
    rotZ = 0;

    //Timer Animation
    timerId = 0;
    t = 0.0;

    camPos = QVector3D(0, 0, -100);

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
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_COLOR_MATERIAL);

    // http://nehe.gamedev.net/data/lessons/lesson.asp?lesson=08
    glEnable(GL_BLEND);		// Turn Blending On
    glDisable(GL_DEPTH_TEST); // ..and dt off, also for blend.

    static GLfloat lightPosition[4] = { 0.5, 5.0, 7.0, 1.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);




//    glShadeModel(GL_SMOOTH);
//    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
//    glClearDepth(1.0f);
//    glEnable(GL_DEPTH_TEST);
//    glDepthFunc(GL_LEQUAL);
//    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
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
    gluPerspective(130.0, (GLfloat)w/(GLfloat)h, 0.25, +80000.0);
    glTranslatef(camPos.x(), camPos.y(), camPos.z());
    glMatrixMode(GL_MODELVIEW);
}

void GlWidget::moveCamera(const QVector3D &pos)
{
    qDebug() << "moveCamera to " << pos;
    camPos = pos;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(130.0, (GLfloat)width()/(GLfloat)height(), 0.25, +80000.0);
    glTranslatef(camPos.x(), camPos.y(), camPos.z());
    glMatrixMode(GL_MODELVIEW);
}

void GlWidget::clear(void)
{
//    mPoints.clear();
}

void GlWidget::addPoints(QList<QVector3D> points)
{
//    mPoints.append(points);
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

    // Mouse Move Rotations
    glRotatef(rotX,1.0,0.0,0.0);
    glRotatef(rotY,0.0,1.0,0.0);
    glRotatef(rotZ,0.0,0.0,1.0);

    glTranslatef(camPos.x(), camPos.y(), camPos.z());

    axes(1.25, 1.1, 1.25, 0.0, 1.0, 0.0);

    //Wheel Scaling
    glScalef(currentScaling,currentScaling,currentScaling);

    //Timer Animation
    GLfloat timerScaling = 0.5+pow(cos(0.025*t),2);
    glScalef(timerScaling,timerScaling,timerScaling);
    glRotatef(t,1,1,1);

    axes(10, 10, 10, 1.0, 1.0, 0.0);

    glTranslatef(-0.5, -0.5, -0.5);

    // Draw base plate
    /*
    glDisable(GL_LIGHTING);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE);
    glColor3f(1.0f, 1.0f, 1.0f);
    glBegin(GL_QUADS);
    glVertex3f(1000, 0, -1000);
    glVertex3f(1000, 0, 1000);
    glVertex3f(-1000, 0, 1000);
    glVertex3f(-1000, 0, -1000);
    glEnd();
    glEnable(GL_LIGHTING);
    */


    mOctree->drawGl();



}

void GlWidget::axes(
        const GLfloat& x, const GLfloat& y, const GLfloat& z,
        const GLfloat& red, const GLfloat& green, const GLfloat& blue) const
{
    glDisable(GL_LIGHTING);
    glColor3f(red,green,blue);
    glLineWidth(1);
    glBegin(GL_LINES);
    glVertex3f(-x,0.0,0.0); glVertex3f( x,0.0,0.0);
    glVertex3f(0.0,-y,0.0); glVertex3f(0.0, y,0.0);
    glVertex3f(0.0,0.0,-z); glVertex3f(0.0,0.0, z);
    glEnd();
    glEnable(GL_LIGHTING);
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
//    qDebug() << numDegrees << numSteps;
    double factor = event->delta()/93.0;
    if(factor > 0)
        moveCamera(camPos * factor);
    else
        moveCamera(camPos / (-factor));
    //moveCamera(camPos + 100);
    //zoom(pow(ZoomFactor, numSteps));
    //paintGL();
    zoom(1);
}

void GlWidget::zoom(double zoomFactor)
{
    currentScaling *= zoomFactor;
    updateGL();
}

void GlWidget::timerEvent ( QTimerEvent * event )
{
    if(event->timerId() == timerId)
    {
        t += 0.25; updateGL();
    }
}

#include <sys/file.h>
#include <stdlib.h>
#include <stdio.h>

char *textFileRead(char *fn) {


        FILE *fp;
        char *content = NULL;

        int f,count;
        f = open(fn, O_RDONLY);

        count = lseek(f, 0, SEEK_END);

        close(f);

        if (fn != NULL) {
                fp = fopen(fn,"rt");

                if (fp != NULL) {


                        if (count > 0) {
                                content = (char *)malloc(sizeof(char) * (count+1));
                                count = fread(content,sizeof(char),count,fp);
                                content[count] = '\0';
                        }
                        fclose(fp);
                }
        }
        return content;
}

                GLuint v,f,p;

void GlWidget::setShaders()
{
/*
    char *vs,*fs;


                v = glCreateShader(GL_VERTEX_SHADER);
//		f = glCreateShader(GL_FRAGMENT_SHADER);

                vs = textFileRead("shader_vertex.c");
//		fs = textFileRead("toon.frag");

                const char * vv = vs;
//		const char * ff = fs;

                glShaderSource(v, 1, &vv,NULL);
//		glShaderSource(f, 1, &ff,NULL);

                free(vs);
//                free(fs);

                glCompileShader(v);
//		glCompileShader(f);

                p = glCreateProgram();

                glAttachShader(p,v);
//		glAttachShader(p,f);

                glLinkProgram(p);
                glUseProgram(p);
                */
}
