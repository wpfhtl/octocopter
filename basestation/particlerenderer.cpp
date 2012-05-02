#include <GL/glew.h>

#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <QDebug>

#include "particlerenderer.h"
//#include "cudashaders.h"

ParticleRenderer::ParticleRenderer()
{
    mParticleRadius = 0.125f * 0.5f;
    mParticleRadius = 3.0f;

    mNumberOfParticles = 0;
    mVbo = 0;
    mColorVbo = 0;

    mShaderProgram = new ShaderProgram(this, "shader-default-vertex.c", "shader-particles-geometry.c", "shader-particles-fragment.c");

    // Clamps Color. Aha. Seems to be disabled by default anyway?!
    glClampColorARB(GL_CLAMP_VERTEX_COLOR_ARB, GL_FALSE);
    glClampColorARB(GL_CLAMP_FRAGMENT_COLOR_ARB, GL_FALSE);
}

ParticleRenderer::~ParticleRenderer()
{
    // TODO: Does this delete the shaders? No.
    mShaderProgram->deleteLater();
}

void ParticleRenderer::setVertexBuffer(unsigned int vbo, int numParticles)
{
    mVbo = vbo;
    mNumberOfParticles = numParticles;
}

void ParticleRenderer::render()
{
//    qDebug() << "ParticleRenderer::render(): drawing" << mNumberOfParticles << "particles with radius" << mParticleRadius << "as spheres into window of size" << mGlWindowSize;
    //glEnable(GL_POINT_SPRITE_ARB); // Only for rendering point sprites.
    //glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
    //glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_NV);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE/*_MINUS_SRC_ALPHA*/);
    //glEnable(GL_BLEND);

    // Program needs to be in use before setting values to uniforms
    mShaderProgram->bind();

    // Set particleRadius variable in the shader program
    Q_ASSERT(glGetUniformLocation(mShaderProgram->programId(), "particleRadius") != -1);
    glUniform1f(glGetUniformLocation(mShaderProgram->programId(), "particleRadius"), mParticleRadius);

    /* we set matrices using an UBO now
    GLfloat matrixPGl[4*4]; for(int i=0;i<16;i++) matrixPGl[i] = *(mMatrixProjection.constData()+i);
//    Q_ASSERT(glGetUniformLocation(mShaderProgram->programId(), "matProjection") != -1);
    glUniformMatrix4fv(glGetUniformLocation(mShaderProgram->programId(), "matProjection"), 1, GL_FALSE, matrixPGl);

    GLfloat matrixMvGl[4*4]; for(int i=0;i<16;i++) matrixMvGl[i] = *(mMatrixModelView.inverted().constData()+i);
    Q_ASSERT(glGetUniformLocation(mShaderProgram->programId(), "matModelView") != -1);
    glUniformMatrix4fv(glGetUniformLocation(mShaderProgram->programId(), "matModelView"), 1, GL_FALSE, matrixMvGl);

//    QVector3D origin;
//    qDebug() << origin << "multiplied with modelview matrix is at camera position:" << mMatrixModelView * origin;

    QVector3D camPos = QVector3D(0, 500, 500); // TODO: implement camera position update
    Q_ASSERT(glGetUniformLocation(mShaderProgram->programId(), "cameraPosition") != -1);
    glUniform3f(glGetUniformLocation(mShaderProgram->programId(), "cameraPosition"), camPos.x(), camPos.y(), camPos.z());
    */

    glBindBuffer(GL_ARRAY_BUFFER, mVbo);
    // Make the contents of this array available at layout position vertexShaderVertexIndex in the vertex shader
    Q_ASSERT(glGetAttribLocation(mShaderProgram->programId(), "in_position") != -1);
    glEnableVertexAttribArray(glGetAttribLocation(mShaderProgram->programId(), "in_position"));
    glVertexAttribPointer(glGetAttribLocation(mShaderProgram->programId(), "in_position"), 4, GL_FLOAT, GL_FALSE, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    if(mColorVbo)
    {
        glBindBuffer(GL_ARRAY_BUFFER, mColorVbo);
        Q_ASSERT(glGetAttribLocation(mShaderProgram->programId(), "in_color") != -1);
        glEnableVertexAttribArray(glGetAttribLocation(mShaderProgram->programId(), "in_color"));
        glVertexAttribPointer(glGetAttribLocation(mShaderProgram->programId(), "in_color"), 4, GL_FLOAT, GL_FALSE, 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    } else qDebug() << "ParticleRenderer::render(): no color VBO present!";

    // Draw using shaders
    glDrawArrays(GL_POINTS, 0, mNumberOfParticles);

    glDisableVertexAttribArray(glGetAttribLocation(mShaderProgram->programId(), "in_position"));
    glDisableVertexAttribArray(glGetAttribLocation(mShaderProgram->programId(), "in_color"));

    // Disable shaders
    mShaderProgram->release();
}

/*
void ParticleRenderer::slotSetMatrices(const QMatrix4x4& modelview, const QMatrix4x4& projection)
{
    mMatrixModelView = modelview;
    mMatrixProjection = projection;
//    qDebug() << "ParticleRenderer::slotSetMatrices(): modelview:" << mMatrixModelView << "projection:" << mMatrixProjection;
}
*/
