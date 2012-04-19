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
//    mGlPointSize = 10.0f;
    mVbo = 0;
    mColorVbo = 0;

    QDir shaderPath = QDir::current();
    shaderPath.cdUp(); // because we're in the build/ subdir

    mShaderProgram = new QGLShaderProgram(this);
    if(mShaderProgram->addShaderFromSourceFile(QGLShader::Vertex, shaderPath.absolutePath() + "/shader-particles-vertex.c"))
        qDebug() << "ParticleRenderer::ParticleRenderer(): compiling vertex shader succeeded, log:" << mShaderProgram->log();
    else
        qDebug() << "ParticleRenderer::ParticleRenderer(): compiling vertex shader failed, log:" << mShaderProgram->log();

    if(mShaderProgram->addShaderFromSourceFile(QGLShader::Geometry, shaderPath.absolutePath() + "/shader-particles-geometry.c"))
        qDebug() << "ParticleRenderer::ParticleRenderer(): compiling geometry shader succeeded, log:" << mShaderProgram->log();
    else
        qDebug() << "ParticleRenderer::ParticleRenderer(): compiling geometry shader failed, log:" << mShaderProgram->log();

    if(mShaderProgram->addShaderFromSourceFile(QGLShader::Fragment, shaderPath.absolutePath() + "/shader-particles-fragment.c"))
        qDebug() << "ParticleRenderer::ParticleRenderer(): compiling fragment shader succeeded, log:" << mShaderProgram->log();
    else
        qDebug() << "ParticleRenderer::ParticleRenderer(): compiling fragment shader failed, log:" << mShaderProgram->log();

    if(!mShaderProgram->link())
        qDebug() << "ParticleRenderer::ParticleRenderer(): linking shader program failed, log:" << mShaderProgram->log();

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
    qDebug() << "ParticleRenderer::render(): drawing particles with radius" << mParticleRadius << "as spheres into window of size" << mGlWindowSize;
    //glEnable(GL_POINT_SPRITE_ARB); // Only for rendering point sprites.
    //glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
    //glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_NV);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE/*_MINUS_SRC_ALPHA*/);
    //glEnable(GL_BLEND);

    // Program needs to bein use before setting values to uniforms
    //glUseProgram(mShaderProgram->programId()); same thing:
    mShaderProgram->bind();

    // Set pointScale and pointRadius variable values in the shader program
    //glUniform1f( glGetUniformLocation(mGlProgramHandle, "pointScale"), m_window_h / tanf(m_fov*0.5f*(float)M_PI/180.0f) );
    //glUniform1f(glGetUniformLocation(mGlProgramHandle, "pointScale"), mGlWindowSize.height() / tanf(mFov * 0.5f * (float)M_PI/180.0f));
    //        glUniform1f(glGetUniformLocation(mGlProgramHandle, "pointScale"), mGlWindowSize.width());
    glUniform1f(glGetUniformLocation(mShaderProgram->programId(), "particleRadius"), mParticleRadius);

    QMatrix4x4 matrixMVP = mMatrixProjection * mMatrixModelView;

    GLfloat matrix[4*4]; for(int i=0;i<16;i++) matrix[i] = *(matrixMVP.constData()+i);
    glUniformMatrix4fv(glGetUniformLocation(mShaderProgram->programId(), "matModelViewProjection"), 1, GL_FALSE, matrix);

    QVector3D camPos = matrixMVP.inverted() * QVector3D(0, 500, 500); // TODO: implement camera position update
    glUniform3f(glGetUniformLocation(mShaderProgram->programId(), "cameraPosition"), camPos.x(), camPos.y(), camPos.z());

    qDebug() << "ParticleRenderer::render(): using VBO to draw" << mNumberOfParticles << "particles";

    glBindBuffer(GL_ARRAY_BUFFER, mVbo);
    // Make the contents of this array available at layout position vertexShaderVertexIndex in the vertex shader
    glEnableVertexAttribArray(glGetAttribLocation(mShaderProgram->programId(), "in_particlePosition"));
    glVertexAttribPointer(glGetAttribLocation(mShaderProgram->programId(), "in_particlePosition"), 4, GL_FLOAT, GL_FALSE, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    if(mColorVbo)
    {
        glBindBuffer(GL_ARRAY_BUFFER, mColorVbo);
        glEnableVertexAttribArray(glGetAttribLocation(mShaderProgram->programId(), "in_particleColor"));
        glVertexAttribPointer(glGetAttribLocation(mShaderProgram->programId(), "in_particleColor"), 4, GL_FLOAT, GL_FALSE, 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    } else qDebug() << "ParticleRenderer::render(): no color VBO present!";

    // Draw using shaders
    glDrawArrays(GL_POINTS, 0, mNumberOfParticles);

    glDisableVertexAttribArray(glGetAttribLocation(mShaderProgram->programId(), "in_particlePosition"));
    glDisableVertexAttribArray(glGetAttribLocation(mShaderProgram->programId(), "in_particleColor"));

    // Disable shaders
    mShaderProgram->release();
}

void ParticleRenderer::slotSetMatrices(const QMatrix4x4& modelview, const QMatrix4x4& projection)
{
    mMatrixModelView = modelview;
    mMatrixProjection = projection;
    qDebug() << "ParticleRenderer::slotSetMatrices(): modelview:" << mMatrixModelView << "projection:" << mMatrixProjection;
}
