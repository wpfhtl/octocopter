#ifndef __RENDER_PARTICLES__
#define __RENDER_PARTICLES__

#include <QSize>

class ParticleRenderer
{
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
    void setParticleRadius(float r) { mParticleRadius = r; }
    void setFOV(float fov) { mFov = fov; }
    void setWindowSize(QSize size) {mGlWindowSize = size;}

protected: // methods
//    void _initGL();
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

#endif //__ RENDER_PARTICLES__
