#ifndef RENDER_PARTICLES_H
#define RENDER_PARTICLES_H

#include <QObject>
#include <QMatrix4x4>
#include <QSize>
#include <QFile>
#include <QDir>

#include "shaderprogram.h"

class ParticleRenderer : public QObject
{
    Q_OBJECT
public:
    ParticleRenderer();
    ~ParticleRenderer();

    void setPositions(float *pos, int numParticles);
    void setVertexBuffer(unsigned int vbo, int numParticles);
    void setColorBuffer(unsigned int vbo) { mColorVbo = vbo; }

    enum DisplayMode
    {
        PARTICLE_POINTS,
        PARTICLE_SPHERES,
        PARTICLE_NUM_MODES
    };

    void render();
    void displayGrid();

public slots:
    void slotSetParticleRadius(float r) { mParticleRadius = r; }

private:
    ShaderProgram* mShaderProgram;

protected:
    int mNumberOfParticles;
    float mParticleRadius;
    QSize mGlWindowSize;

    GLuint mVbo;
    GLuint mColorVbo;
};

#endif
