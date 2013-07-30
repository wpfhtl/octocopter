#ifndef OPENGLUTILITIES_H
#define OPENGLUTILITIES_H

#include <QColor>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLFunctions_4_3_Core>
#include <QVector3D>
#include <QMatrix4x4>
#include <QString>
#include <QByteArray>
#include "common.h"

class OpenGlUtilities
{
private:
    static OPENGL_FUNCTIONS_CLASS* mGlFunctions;

    explicit OpenGlUtilities();

public:
    static void init();

//    static QVector<float> matrixToOpenGl(const QMatrix4x4 &matrix);

    static quint32 createVbo(quint32 size);
    static void deleteVbo(GLuint vbo);

    static void setVboToBoundingBox(const quint32 vbo, const Box3D &box);
};

#endif // OPENGLUTILITIES_H
