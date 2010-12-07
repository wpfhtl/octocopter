#ifndef OPENGLUTILITIES_H
#define OPENGLUTILITIES_H

#include <QColor>
#include <QVector3D>
#include <GL/glut.h>

class OpenGlUtilities
{
public:
    inline static void drawSphere(const QVector3D &point, const float radius = 5.0, const int subdivisions = 10, const QColor color = QColor(255,0,0));
    inline static void drawSphere(const QVector3D &point);
};

#endif // OPENGLUTILITIES_H
