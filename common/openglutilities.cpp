#include <QFile>
#include "openglutilities.h"

OPENGL_FUNCTIONS_CLASS* OpenGlUtilities::mGlFunctions = 0;

void OpenGlUtilities::init()
{
    mGlFunctions = new OPENGL_FUNCTIONS_CLASS;
    mGlFunctions->initializeOpenGLFunctions();
}

quint32 OpenGlUtilities::createVbo(quint32 size)
{
    mGlFunctions->glGetError(); // clear previous GL errors!
    GLuint vbo;
    mGlFunctions->glGenBuffers(1, &vbo);
    mGlFunctions->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    mGlFunctions->glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);
    mGlFunctions->glBindBuffer(GL_ARRAY_BUFFER, 0);
    // TODO: replace this by using a debug context and debug callback
    if(mGlFunctions->glGetError() != GL_NO_ERROR) qFatal("ParticleSystem::createVbo(): error creating VBO!");
    return vbo;
}

void OpenGlUtilities::deleteVbo(GLuint vbo)
{
    mGlFunctions->glDeleteBuffers(1, &vbo);
}


void OpenGlUtilities::setVboToBoundingBox(const quint32 vbo, const Box3D& box)
{
    QVector<float> vertices;

    // Fill the vertices buffer with vertices for quads and lines
    vertices
            // 1 back
            << box.min.x() << box.min.y() << box.min.z() << 1.0f
            << box.max.x() << box.min.y() << box.min.z() << 1.0f
            << box.max.x() << box.max.y() << box.min.z() << 1.0f
            << box.min.x() << box.max.y() << box.min.z() << 1.0f

            // 2 front
            << box.max.x() << box.min.y() << box.max.z() << 1.0f
            << box.min.x() << box.min.y() << box.max.z() << 1.0f
            << box.min.x() << box.max.y() << box.max.z() << 1.0f
            << box.max.x() << box.max.y() << box.max.z() << 1.0f

            // 3 left
            << box.min.x() << box.min.y() << box.max.z() << 1.0f
            << box.min.x() << box.min.y() << box.min.z() << 1.0f
            << box.min.x() << box.max.y() << box.min.z() << 1.0f
            << box.min.x() << box.max.y() << box.max.z() << 1.0f

            // 4 right
            << box.max.x() << box.min.y() << box.min.z() << 1.0f
            << box.max.x() << box.min.y() << box.max.z() << 1.0f
            << box.max.x() << box.max.y() << box.max.z() << 1.0f
            << box.max.x() << box.max.y() << box.min.z() << 1.0f

            // 6 top
            << box.min.x() << box.max.y() << box.min.z() << 1.0f
            << box.max.x() << box.max.y() << box.min.z() << 1.0f
            << box.max.x() << box.max.y() << box.max.z() << 1.0f
            << box.min.x() << box.max.y() << box.max.z() << 1.0f

            // 5 bottom
            << box.min.x() << box.min.y() << box.max.z() << 1.0f
            << box.max.x() << box.min.y() << box.max.z() << 1.0f
            << box.max.x() << box.min.y() << box.min.z() << 1.0f
            << box.min.x() << box.min.y() << box.min.z() << 1.0f;

    mGlFunctions->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    mGlFunctions->glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size(), NULL, GL_STATIC_DRAW);
    mGlFunctions->glBufferSubData(
                GL_ARRAY_BUFFER,
                0, // offset in the VBO
                vertices.size() * sizeof(float), // how many bytes to store?
                (void*)(vertices.constData()) // data to store
                );

    mGlFunctions->glBindBuffer(GL_ARRAY_BUFFER, 0);
}
