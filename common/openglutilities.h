#ifndef OPENGLUTILITIES_H
#define OPENGLUTILITIES_H

#include <QColor>
#include <QVector3D>
#include <QMatrix4x4>
#include <GL/freeglut.h>

class OpenGlUtilities
{
public:
    static void drawSphere(const QVector3D &point, const float radius = 2.0, const int subdivisions = 5, const QColor color = QColor(255,255,0));
    static void drawSphere(const QVector3D &point);
    static void drawPoint(const QVector3D &point);
    static void drawAabb(const QVector3D &min, const QVector3D &max, const QColor &color, const quint8& lineWidth = 1);

    static QVector<float> matrixToOpenGl(const QMatrix4x4 &matrix);
};

#endif // OPENGLUTILITIES_H
