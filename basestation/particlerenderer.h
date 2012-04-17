#ifndef RENDER_PARTICLES_H
#define RENDER_PARTICLES_H

#include <GL/glew.h>
#include <GL/freeglut.h>

#include <QObject>
#include <QSize>

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

    void display(DisplayMode mode = PARTICLE_POINTS);
    void displayGrid();

    void setPointSize(float size)  { mGlPointSize = size; }
    void setWindowSize(QSize size) {mGlWindowSize = size;}

public slots:
    void slotSetParticleRadius(float r) { mParticleRadius = r; }
    void slotSetFovVertical(float fov);

protected: // methods
    void drawPoints();
    GLuint compileProgram(const char *vsource, const char *fsource);

protected: // data
//    float *mPositions;
    int mNumberOfParticles;

    float mGlPointSize;
    float mParticleRadius;
    float mFov;
    QSize mGlWindowSize;

    GLuint mGlProgramHandle;

    GLuint mVbo;
    GLuint mColorVbo;
};

#endif
