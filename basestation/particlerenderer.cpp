#include <GL/glew.h>

#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <QDebug>

#include "particlerenderer.h"
#include "cudashaders.h"

#ifndef M_PI
#define M_PI    3.1415926535897932384626433832795
#endif

ParticleRenderer::ParticleRenderer()
{
    mParticleRadius = 0.125f * 0.5f;
    mParticleRadius = 3.0f;

//    mPositions = 0;
      mNumberOfParticles = 0;
      mGlPointSize = 10.0f;
      mGlProgramHandle = 0;
      mVbo = 0;
      mColorVbo = 0;

    mGlProgramHandle = compileProgram(vertexShader, spherePixelShader);

    glClampColorARB(GL_CLAMP_VERTEX_COLOR_ARB, GL_FALSE);
    glClampColorARB(GL_CLAMP_FRAGMENT_COLOR_ARB, GL_FALSE);
}

ParticleRenderer::~ParticleRenderer()
{
//    mPositions = 0;
}

/*void ParticleRenderer::setPositions(float *pos, int numParticles)
{
    mPositions = pos;
    mNumberOfParticles = numParticles;
}*/

void ParticleRenderer::setVertexBuffer(unsigned int vbo, int numParticles)
{
    mVbo = vbo;
    mNumberOfParticles = numParticles;
}

void ParticleRenderer::drawPoints()
{
    if (!mVbo)
    {
        qDebug() << "ParticleRenderer::drawPoints(): using glBegin/glEnd, which sucks!";
        Q_ASSERT(false);
        glBegin(GL_POINTS);
        {
            int k = 0;
            for (int i = 0; i < mNumberOfParticles; ++i)
            {
                //glVertex3fv(&mPositions[k]);
                k += 4;
            }
        }
        glEnd();
    }
    else
    {
        qDebug() << "ParticleRenderer::drawPoints(): using VBO to draw" << mNumberOfParticles << "particles";
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, mVbo);
        glVertexPointer(4, GL_FLOAT, 0, 0);
        glEnableClientState(GL_VERTEX_ARRAY);                

        if (mColorVbo)
        {
            glBindBufferARB(GL_ARRAY_BUFFER_ARB, mColorVbo);
            glColorPointer(4, GL_FLOAT, 0, 0);
            glEnableClientState(GL_COLOR_ARRAY);
        }

        glDrawArrays(GL_POINTS, 0, mNumberOfParticles);

        glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);
        glDisableClientState(GL_VERTEX_ARRAY); 
        glDisableClientState(GL_COLOR_ARRAY); 
    }
}

void ParticleRenderer::display(DisplayMode mode /* = PARTICLE_POINTS */)
{
    switch (mode)
    {
    case PARTICLE_POINTS:
        qDebug() << "ParticleRenderer::display(): drawing particles as points.";
        glColor3f(1, 1, 1);
        glPointSize(mGlPointSize);
        drawPoints();
        break;

    default:
    case PARTICLE_SPHERES:
        qDebug() << "ParticleRenderer::display(): drawing particles as spheres into window of size" << mGlWindowSize;
        glEnable(GL_POINT_SPRITE_ARB); Q_ASSERT(glIsEnabled(GL_POINT_SPRITE_ARB));
        glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
        glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_NV); Q_ASSERT(glIsEnabled(GL_VERTEX_PROGRAM_POINT_SIZE_NV));
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST); Q_ASSERT(glIsEnabled(GL_DEPTH_TEST));

        Q_ASSERT(mGlProgramHandle != 0);
        glUseProgram(mGlProgramHandle);

        // Set pointScale and pointRadius variable values in the shader program
        //glUniform1f( glGetUniformLocation(mGlProgramHandle, "pointScale"), m_window_h / tanf(m_fov*0.5f*(float)M_PI/180.0f) );
        glUniform1f(glGetUniformLocation(mGlProgramHandle, "pointScale"), mGlWindowSize.height() / tanf(mFov * 0.5f * (float)M_PI/180.0f));
        glUniform1f(glGetUniformLocation(mGlProgramHandle, "pointRadius"), mParticleRadius);

        glColor3f(1, 1, 1);
        drawPoints();

        glUseProgram(0);
        glDisable(GL_POINT_SPRITE_ARB);
        break;
    }
}

GLuint ParticleRenderer::compileProgram(const char *vsource, const char *fsource)
{
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

    glShaderSource(vertexShader, 1, &vsource, 0);
    glShaderSource(fragmentShader, 1, &fsource, 0);
    
    glCompileShader(vertexShader);
    glCompileShader(fragmentShader);

    GLuint program = glCreateProgram();

    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);

    glLinkProgram(program);

    // check if program linked
    GLint success = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &success);

    if(success)
    {
        qDebug() << "ParticleRenderer::compileProgram(): linked successfully!";
    }
    else
    {
        char linkErrorMessage[256];
        glGetProgramInfoLog(program, 256, 0, linkErrorMessage);
        qDebug() << "ParticleRenderer::compileProgram(): linking failed: " << linkErrorMessage;
        glDeleteProgram(program);
        program = 0;
    }

    return program;
}
