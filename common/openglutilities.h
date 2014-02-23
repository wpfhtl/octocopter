#ifndef OPENGLUTILITIES_H
#define OPENGLUTILITIES_H

//#include <QColor>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLFunctions_4_3_Core>
#include <QVector3D>
//#include <QMatrix4x4>
//#include <QString>
//#include <QByteArray>
#include "common.h"
#include <box3d.h>

class OpenGlUtilities
{
private:
    static OPENGL_FUNCTIONS_CLASS* mGlFunctions;

    explicit OpenGlUtilities();

public:
    static void initialize();

    static quint32 createVbo(quint32 size = 1, const GLvoid *data = 0, GLenum usage = GL_DYNAMIC_DRAW);
    static void deleteVbo(GLuint vbo);

    static inline bool checkError()
    {
        GLenum glError;
        glError = mGlFunctions->glGetError();
        if(glError != GL_NO_ERROR)
        {
            qFatal("OpenGlUtilities::checkError(): error %d.\n", glError);
            return false;
        }
        return true;
    }

    static void setVboToBoundingBox(const quint32 vbo, const Box3D *box);
};

#endif // OPENGLUTILITIES_H
