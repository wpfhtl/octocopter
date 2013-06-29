#ifndef OPENGLUTILITIES_H
#define OPENGLUTILITIES_H

#include <QColor>
#include <QOpenGLFunctions_4_3_Core>
#include <QVector3D>
#include <QMatrix4x4>
#include <QString>
#include <QByteArray>

class OpenGlUtilities
{
private:
    static QOpenGLFunctions_4_3_Core* mGlFunctions;

    explicit OpenGlUtilities();

public:
    static void init();

//    static QVector<float> matrixToOpenGl(const QMatrix4x4 &matrix);

    static quint32 createVbo(quint32 size);
    static void deleteVbo(GLuint vbo);

    static void setVboToBoundingBox(const quint32 vbo, const QVector3D& min, const QVector3D& max);
};

#endif // OPENGLUTILITIES_H
