#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <QDebug>
#include <openglutilities.h>

#include "particlerenderer.h"

ParticleRenderer::ParticleRenderer()
{
    mIsInitialized = false;

    mRenderBoundingBox = true;
    mRenderParticles = false;
    mRenderWaypointPressure = false;

    mParticleRadius = 0.0f;
    mNumberOfParticles = 0;

    mVboParticlePositions = 0;
    mVboParticleColors = 0;
    mVboParticleSystemBoundingBox = OpenGlUtilities::createVbo(1);

    mVboGridMapOfWayPointPressure = 0;

    mShaderProgramDefault = new ShaderProgram(this, "shader-default-vertex.c", "", "shader-default-fragment.c");
    mShaderProgramParticles = new ShaderProgram(this, "shader-particles-vertex.c", "shader-particles-geometry.c", "shader-particles-fragment.c");
    mShaderProgramGrid = new ShaderProgram(this, "shader-grid-vertex.c", "shader-grid-geometry.c", "shader-grid-fragment.c");
}

ParticleRenderer::~ParticleRenderer()
{
    // TODO: Does this delete the shaders? No.
    mShaderProgramDefault->deleteLater();
    mShaderProgramParticles->deleteLater();
    mShaderProgramGrid->deleteLater();
}

void ParticleRenderer::slotSetVboInfoParticles(const quint32 vboPositions, const quint32 vboColors, const quint32 count, const QVector3D particleSystemWorldMin, const QVector3D particleSystemWorldMax)
{
    mVboParticleColors = vboColors;
    mVboParticlePositions = vboPositions;
    mNumberOfParticles = count;

    OpenGlUtilities::setVboToBoundingBox(mVboParticleSystemBoundingBox, particleSystemWorldMin, particleSystemWorldMax);

    qDebug() << "ParticleRenderer::slotSetVboInfoParticles(): will render VBO pos" << mVboParticlePositions << "color" << mVboParticleColors << "containing" << mNumberOfParticles << "particles";
}

void ParticleRenderer::slotSetVboInfoGridWaypointPressure(const quint32 vboPressure, const QVector3D &gridBoundingBoxMin, const QVector3D &gridBoundingBoxMax, const Vector3i &grid)
{
    mVboGridMapOfWayPointPressure = vboPressure;
    mWaypointGridMin = gridBoundingBoxMin;
    mWaypointGridMax = gridBoundingBoxMax;
    mGridCellCount = grid;
//    qDebug() << "ParticleRenderer::slotSetVboInfoGridWaypointPressure(): will render VBO pos" << mVboGridMapOfWayPointPressure << "with" << grid.x << grid.y << grid.z << "cells from" << mGridBoundingBoxMin << "to" << mGridBoundingBoxMax;
}

void ParticleRenderer::render()
{
    if(!mIsInitialized)
    {
        initializeOpenGLFunctions();
        mIsInitialized = true;
    }

    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);

    if(mRenderBoundingBox && mShaderProgramDefault != 0)
    {
        mShaderProgramDefault->bind();
        mShaderProgramDefault->setUniformValue("useFixedColor", true);

//        glEnable(GL_BLEND);
//        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Beau.Ti.Ful!
        {
            glBindBuffer(GL_ARRAY_BUFFER, mVboParticleSystemBoundingBox);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0); // position

            // draw the lines around the box
            mShaderProgramDefault->setUniformValue("fixedColor", QVector4D(0.2f, 0.2f, 1.0f, 0.8f));
            glDrawArrays(GL_LINE_LOOP, 0, 4);
            glDrawArrays(GL_LINE_LOOP, 4, 4);
            glDrawArrays(GL_LINE_LOOP, 8, 4);
            glDrawArrays(GL_LINE_LOOP, 12, 4);
            glDrawArrays(GL_LINE_LOOP, 16, 4);
            glDrawArrays(GL_LINE_LOOP, 20, 4);

            // draw a half-transparent box
            mShaderProgramDefault->setUniformValue("fixedColor", QVector4D(1.0f, 1.0f, 1.0f, 0.015f));
            glDrawArrays(GL_QUADS, 0, 24);

            glDisableVertexAttribArray(0);
        }
//        glDisable(GL_BLEND);
        mShaderProgramDefault->release();
    }

    if(mNumberOfParticles != 0 && mVboParticleColors != 0 && mVboParticlePositions != 0 && mRenderParticles)
    {
        // Program needs to be in use before setting values to uniforms
        mShaderProgramParticles->bind();

        mShaderProgramParticles->setUniformValue("useFixedColor", false);

        // Set particleRadius variable in the shader program
        Q_ASSERT(glGetUniformLocation(mShaderProgramParticles->programId(), "particleRadius") != -1);
        glUniform1f(glGetUniformLocation(mShaderProgramParticles->programId(), "particleRadius"), mParticleRadius);
        //    qDebug() << "ParticleRenderer::render(): particle radius:" << mParticleRadius;

        // Make the contents of this array available at layout position vertexShaderVertexIndex in the vertex shader
        glBindBuffer(GL_ARRAY_BUFFER, mVboParticlePositions);
        Q_ASSERT(glGetAttribLocation(mShaderProgramParticles->programId(), "in_position") != -1);
        glEnableVertexAttribArray(glGetAttribLocation(mShaderProgramParticles->programId(), "in_position"));
        glVertexAttribPointer(glGetAttribLocation(mShaderProgramParticles->programId(), "in_position"), 4, GL_FLOAT, GL_FALSE, 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        /* colors abnd their VBO are currently unused
        glBindBuffer(GL_ARRAY_BUFFER, mVboParticleColors);
        Q_ASSERT(glGetAttribLocation(mShaderProgramParticles->programId(), "in_color") != -1);
        glEnableVertexAttribArray(glGetAttribLocation(mShaderProgramParticles->programId(), "in_color"));
        glVertexAttribPointer(glGetAttribLocation(mShaderProgramParticles->programId(), "in_color"), 4, GL_FLOAT, GL_FALSE, 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);*/

        // Draw using shaders
        glDrawArrays(GL_POINTS, 0, mNumberOfParticles);

        glDisableVertexAttribArray(glGetAttribLocation(mShaderProgramParticles->programId(), "in_position"));
        glDisableVertexAttribArray(glGetAttribLocation(mShaderProgramParticles->programId(), "in_color"));

        // Disable shader
        mShaderProgramParticles->release();

    }

    if(mVboGridMapOfWayPointPressure != 0 && mRenderWaypointPressure)
    {
        // Draw grid with waypoint pressure
        mShaderProgramGrid->bind();

        // Set uniform values in the shader program
        Q_ASSERT(mShaderProgramGrid->uniformLocation("boundingBoxMin") != -1);
        mShaderProgramGrid->setUniformValue("boundingBoxMin", mWaypointGridMin);

        Q_ASSERT(mShaderProgramGrid->uniformLocation("boundingBoxMax") != -1);
        mShaderProgramGrid->setUniformValue("boundingBoxMax", mWaypointGridMax);

//        qDebug() << "bbox from" << mGridBoundingBoxMin << "to" << mGridBoundingBoxMax;

        // gridSize is a uint3, not sure how to set this with qt, so lets do opengl:
        Q_ASSERT(mShaderProgramGrid->uniformLocation("gridCellCount") != -1);
        //mShaderProgramGrid->setUniformValue("gridCellCount", mGridCells);
        glUniform3i(mShaderProgramGrid->uniformLocation("gridCellCount"), mGridCellCount.x, mGridCellCount.y, mGridCellCount.z);

        // Make the contents of this array available at layout position vertexShaderVertexIndex in the vertex shader
        glBindBuffer(GL_ARRAY_BUFFER, mVboGridMapOfWayPointPressure);
        Q_ASSERT(mShaderProgramGrid->attributeLocation("in_waypointpressure") != -1);
        //glEnableVertexAttribArray(mShaderProgramGrid->uniformLocation("in_waypointpressure"));
        mShaderProgramGrid->enableAttributeArray("in_waypointpressure");
        glVertexAttribPointer(mShaderProgramGrid->attributeLocation("in_waypointpressure"), 1, GL_UNSIGNED_BYTE, GL_TRUE, 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        // Draw using shaders
        glDrawArrays(GL_POINTS, 0, mGridCellCount.x * mGridCellCount.y * mGridCellCount.z);

        glDisableVertexAttribArray(mShaderProgramGrid->attributeLocation("in_waypointpressure"));

        mShaderProgramGrid->release();
    }

    glDisable(GL_BLEND);
}
