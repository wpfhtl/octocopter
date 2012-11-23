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

    void slotSetVboInfoColliders(const quint32 vboPositions, const quint32 count)
    {
        mVboColliderPositions = vboPositions;
        mNumberOfColliders = count;
        qDebug() << "ParticleRenderer::slotSetVboInfoColliders(): will render VBO pos" << mVboColliderPositions << "containing" << mNumberOfColliders << "colliders";
    }

    void slotSetVboInfoParticles(const quint32 vboPositions, const quint32 vboColors, const quint32 count)
    {
        mVboParticleColors = vboColors;
        mVboParticlePositions = vboPositions;
        mNumberOfParticles = count;
        qDebug() << "ParticleRenderer::slotSetVboInfoParticles(): will render VBO pos" << mVboParticlePositions << "color" << mVboParticleColors << "containing" << mNumberOfParticles << "particles";
    }

private:
    ShaderProgram* mShaderProgram;
    quint32 mNumberOfParticles;
    quint32 mNumberOfColliders;
    float mParticleRadius;
    GLuint mVboColliderPositions;
    GLuint mVboParticlePositions;
    GLuint mVboParticleColors;
};

#endif
