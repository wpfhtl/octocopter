#ifndef RENDER_PARTICLES_H
#define RENDER_PARTICLES_H

#include <QObject>

#include "shaderprogram.h"
#include <common.h>

class ParticleRenderer : public QObject
{
    Q_OBJECT
public:
    ParticleRenderer();
    ~ParticleRenderer();

    void render();

    bool getRenderParticles() const {return mRenderParticles;}
    bool getRenderWaypointPressure() const {return mRenderWaypointPressure;}

public slots:

    void slotSetRenderParticles(const bool render) {mRenderParticles = render;}
    void slotSetRenderWaypointPressure(const bool render) {mRenderWaypointPressure = render;}

    void slotSetParticleRadius(float r) { mParticleRadius = r; }

    void slotSetVboInfoGridWaypointPressure(const quint32 vboPressure, const QVector3D& gridBoundingBoxMin, const QVector3D& gridBoundingBoxMax, const Vector3i& grid);

    void slotSetVboInfoParticles(const quint32 vboPositions, const quint32 vboColors, const quint32 count);

private:
    bool mRenderParticles, mRenderWaypointPressure;

    ShaderProgram *mShaderProgramParticles, *mShaderProgramGrid;
    quint32 mNumberOfParticles;

    QVector3D mGridBoundingBoxMin, mGridBoundingBoxMax;
    Vector3i mGridCellCount;

    float mParticleRadius;
    GLuint mVboGridMapOfWayPointPressure;
    GLuint mVboParticlePositions;
    GLuint mVboParticleColors;
};

#endif
