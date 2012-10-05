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

    void render();

public slots:
    void slotSetParticleRadius(float r) { mParticleRadius = r; }

    void slotSetVboColors(unsigned int vbo) { mVboColors = vbo; }

    void slotSetVboPositions(unsigned int vbo, unsigned int numParticles)
    {
        mVboPositions = vbo;
        mNumberOfParticles = numParticles;
        qDebug() << "ParticleRenderer::slotSetVboPositions(): will render VBO" << mVboPositions << "containing" << mNumberOfParticles << "particles";
    }

private:
    ShaderProgram* mShaderProgram;
    unsigned int mNumberOfParticles;
    float mParticleRadius;
    GLuint mVboPositions;
    GLuint mVboColors;
};

#endif
