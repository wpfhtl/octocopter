#include <GL/glew.h>

#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <QDebug>

#include "particlerenderer.h"

ParticleRenderer::ParticleRenderer()
{
    mParticleRadius = 0.125f * 0.5f;
    mParticleRadius = 3.0f;

    mNumberOfColliders = 0;
    mNumberOfParticles = 0;

    mVboParticlePositions = 0;
    mVboParticleColors = 0;

    mVboColliderPositions = 0;

    mShaderProgram = new ShaderProgram(this, "shader-particles-vertex.c", "shader-particles-geometry.c", "shader-particles-fragment.c");

    // Clamps Color. Aha. Seems to be disabled by default anyway?!
    //    glClampColorARB(GL_CLAMP_VERTEX_COLOR_ARB, GL_FALSE);
    //    glClampColorARB(GL_CLAMP_FRAGMENT_COLOR_ARB, GL_FALSE);
}

ParticleRenderer::~ParticleRenderer()
{
    // TODO: Does this delete the shaders? No.
    mShaderProgram->deleteLater();
}

void ParticleRenderer::render()
{
    if(mNumberOfParticles == 0 || mVboParticleColors == 0 || mVboParticlePositions == 0)
        return;

    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);

    // TODO: render colliders, remove special case w < 0.5 from shaders

    // Program needs to be in use before setting values to uniforms
    mShaderProgram->bind();

    mShaderProgram->setUniformValue("useFixedColor", false);

    // Set particleRadius variable in the shader program
    Q_ASSERT(glGetUniformLocation(mShaderProgram->programId(), "particleRadius") != -1);
    glUniform1f(glGetUniformLocation(mShaderProgram->programId(), "particleRadius"), mParticleRadius);
    //    qDebug() << "ParticleRenderer::render(): particle radius:" << mParticleRadius;

    // Make the contents of this array available at layout position vertexShaderVertexIndex in the vertex shader
    glBindBuffer(GL_ARRAY_BUFFER, mVboParticlePositions);
    Q_ASSERT(glGetAttribLocation(mShaderProgram->programId(), "in_position") != -1);
    glEnableVertexAttribArray(glGetAttribLocation(mShaderProgram->programId(), "in_position"));
    glVertexAttribPointer(glGetAttribLocation(mShaderProgram->programId(), "in_position"), 4, GL_FLOAT, GL_FALSE, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ARRAY_BUFFER, mVboParticleColors);
    Q_ASSERT(glGetAttribLocation(mShaderProgram->programId(), "in_color") != -1);
    glEnableVertexAttribArray(glGetAttribLocation(mShaderProgram->programId(), "in_color"));
    glVertexAttribPointer(glGetAttribLocation(mShaderProgram->programId(), "in_color"), 4, GL_FLOAT, GL_FALSE, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // Draw using shaders
    glDrawArrays(GL_POINTS, 0, mNumberOfParticles);

    glDisableVertexAttribArray(glGetAttribLocation(mShaderProgram->programId(), "in_position"));
    glDisableVertexAttribArray(glGetAttribLocation(mShaderProgram->programId(), "in_color"));

    // Disable shaders
    mShaderProgram->release();


    glDisable(GL_BLEND);
}
