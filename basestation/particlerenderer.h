#ifndef RENDER_PARTICLES_H
#define RENDER_PARTICLES_H

#include <GL/glew.h>
#include <GL/freeglut.h>

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

    void setWindowSize(QSize size) {mGlWindowSize = size;}

public slots:
    void slotSetParticleRadius(float r) { mParticleRadius = r; }
//    void slotSetMatrices(const QMatrix4x4& modelview, const QMatrix4x4& projection);

private:
    ShaderProgram* mShaderProgram;

protected:
    int mNumberOfParticles;
    float mParticleRadius;
//    QMatrix4x4 mMatrixModelView, mMatrixProjection;
    QSize mGlWindowSize;

    GLuint mVbo;
    GLuint mColorVbo;
};

#endif
