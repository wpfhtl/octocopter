#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QtGui>
#include <QtOpenGL>
#include <QGLWidget>

#include "octree.h"

class GlWidget : public QGLWidget
{
    Q_OBJECT

//    GLfloat rotQuad;
    Octree *mOctree;
    void setShaders();
//    QList<QVector3D> mPoints;

    //Timer
    GLint timerId;
    GLfloat t;

    //Mouse Rotations
    QPoint      lastPos;
    GLfloat     rotX, rotY, rotZ;

    //Wheel Scaling
    GLdouble    currentScaling;
    GLdouble    ZoomFactor;

    void mouseDoubleClickEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void zoom(double zoomFactor);

    void axes(const GLfloat& x, const GLfloat& y, const GLfloat& z, const GLfloat& red, const GLfloat& green, const GLfloat& blue) const;

public:
    GlWidget(QWidget *parent, Octree* octree);
    void addPoints(QList<QVector3D>);
    void clear(void);

protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    void timerEvent ( QTimerEvent * event );

signals:

public slots:

};

#endif
