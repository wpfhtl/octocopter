#ifndef RENDER_PARTICLES_H
#define RENDER_PARTICLES_H

#include <QObject>

#include "shaderprogram.h"

class ParticleRenderer : public QObject
{
    Q_OBJECT
public:
    ParticleRenderer();
    ~ParticleRenderer();

    void setVboPositions(unsigned int vbo, int numParticles);
    void setVboColors(unsigned int vbo);

    void render();

public slots:
    void slotSetParticleRadius(float r) { mParticleRadius = r; }

private:
    ShaderProgram* mShaderProgram;
    int mNumberOfParticles;
    float mParticleRadius;
    GLuint mVboPositions;
    GLuint mVboColors;
};

#endif
