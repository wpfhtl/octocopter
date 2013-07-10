#ifndef RENDER_PARTICLES_H
#define RENDER_PARTICLES_H

#include <QObject>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLFunctions_4_3_Core>

#include "shaderprogram.h"
#include <common.h>

// TODO: Move all of this and some stuff from GlWindow into a Scene-class.

class ParticleRenderer : public QObject, protected OPENGL_FUNCTIONS_CLASS
{
    Q_OBJECT
public:
    ParticleRenderer();
    ~ParticleRenderer();

    void render();

    bool getRenderParticles() const {return mRenderParticles;}
    bool getRenderBoundingBox() const {return mRenderBoundingBox;}
    bool getRenderInformationGain() const {return mRenderInformationGain;}
    bool getRenderOccupancyGrid() const {return mRenderOccupancyGrid;}

public slots:

    void slotSetRenderBoundingBox(const bool render) {mRenderBoundingBox = render;}
    void slotSetRenderParticles(const bool render) {mRenderParticles = render;}
    void slotSetRenderInformationGain(const bool render) {mRenderInformationGain = render;}
    void slotSetRenderOccupancyGrid(const bool render) {mRenderOccupancyGrid = render;}
    void slotSetRenderPathFinderGrid(const bool render) {mRenderPathFinderGrid = render;}

    void slotSetParticleRadius(float r) { mParticleRadius = r; }

    void slotSetVboInfoGridInformationGain(const quint32 vboPressure, const QVector3D& gridBoundingBoxMin, const QVector3D& gridBoundingBoxMax, const Vector3i& gridCells);
    void slotSetVboInfoGridOccupancy(const quint32 vbo, const QVector3D& gridBoundingBoxMin, const QVector3D& gridBoundingBoxMax, const Vector3i& gridCells);
    void slotSetVboInfoGridPathFinder(const quint32 vbo, const QVector3D &gridBoundingBoxMin, const QVector3D &gridBoundingBoxMax, const Vector3i &gridCells);

    void slotSetVboInfoParticles(const quint32 vboPositions, const quint32 vboColors, const quint32 count, const QVector3D particleSystemWorldMin, const QVector3D particleSystemWorldMax);

private:
    bool mIsInitialized;
    bool mRenderParticles, mRenderInformationGain, mRenderBoundingBox, mRenderOccupancyGrid, mRenderPathFinderGrid;

    ShaderProgram *mShaderProgramDefault, *mShaderProgramParticles, *mShaderProgramGrid;
    quint32 mNumberOfParticles;

    QVector3D mGridInformationGainMin, mGridInformationGainMax;
    Vector3i mGridInformationGainCellCount;

    QVector3D mGridOccupancyMin, mGridOccupancyMax;
    Vector3i mGridOccupancyCellCount;

    QVector3D mGridPathFinderMin, mGridPathFinderMax;
    Vector3i mGridPathFinderCellCount;

    float mParticleRadius;
    GLuint mVboGridMapOfOccupancy;
    GLuint mVboGridMapOfInformationGain;
    GLuint mVboGridMapOfPathFinder;
    GLuint mVboParticlePositions;
    GLuint mVboParticleColors;
    GLuint mVboParticleSystemBoundingBox;
};

#endif
