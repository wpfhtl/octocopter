#ifndef OPENGLUTILITIES_H
#define OPENGLUTILITIES_H

#include <QColor>
#include <QVector3D>
#include <QMatrix4x4>

class OpenGlUtilities
{
public:
    static QVector<float> matrixToOpenGl(const QMatrix4x4 &matrix);

    static quint32 createVbo(quint32 size);

    static void setVboToBoundingBox(const quint32 vbo, const QVector3D& min, const QVector3D& max);
};

#endif // OPENGLUTILITIES_H
