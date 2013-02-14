#include <GL/glew.h>
#include "openglutilities.h"

QVector<float> OpenGlUtilities::matrixToOpenGl(const QMatrix4x4 &matrix)
{
    QVector<float> data;
    double* matrixValues = (double*)matrix.constData();
    for(int i=0;i<16;i++)
        data.append((float)matrixValues[i]);

    return data;
}


quint32 OpenGlUtilities::createVbo(quint32 size)
{
    glGetError(); // clear previous GL errors!
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    // TODO: replace this by using a debug context and debug callback
    if(glGetError() != GL_NO_ERROR) qFatal("ParticleSystem::createVbo(): error creating VBO!");
    return vbo;
}


void OpenGlUtilities::setVboToBoundingBox(const quint32 vbo, const QVector3D& min, const QVector3D& max)
{
    QVector<float> vertices;

    // Fill the vertices buffer with vertices for quads and lines
    vertices
            // 1 back
            << min.x() << min.y() << min.z() << 1.0f
            << max.x() << min.y() << min.z() << 1.0f
            << max.x() << max.y() << min.z() << 1.0f
            << min.x() << max.y() << min.z() << 1.0f

            // 2 front
            << max.x() << min.y() << max.z() << 1.0f
            << min.x() << min.y() << max.z() << 1.0f
            << min.x() << max.y() << max.z() << 1.0f
            << max.x() << max.y() << max.z() << 1.0f

            // 3 left
            << min.x() << min.y() << max.z() << 1.0f
            << min.x() << min.y() << min.z() << 1.0f
            << min.x() << max.y() << min.z() << 1.0f
            << min.x() << max.y() << max.z() << 1.0f

            // 4 right
            << max.x() << min.y() << min.z() << 1.0f
            << max.x() << min.y() << max.z() << 1.0f
            << max.x() << max.y() << max.z() << 1.0f
            << max.x() << max.y() << min.z() << 1.0f

            // 6 top
            << min.x() << max.y() << min.z() << 1.0f
            << max.x() << max.y() << min.z() << 1.0f
            << max.x() << max.y() << max.z() << 1.0f
            << min.x() << max.y() << max.z() << 1.0f

            // 5 bottom
            << min.x() << min.y() << max.z() << 1.0f
            << max.x() << min.y() << max.z() << 1.0f
            << max.x() << min.y() << min.z() << 1.0f
            << min.x() << min.y() << min.z() << 1.0f;

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size(), NULL, GL_STATIC_DRAW);
    glBufferSubData(
                GL_ARRAY_BUFFER,
                0, // offset in the VBO
                vertices.size() * sizeof(float), // how many bytes to store?
                (void*)(vertices.constData()) // data to store
                );

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
