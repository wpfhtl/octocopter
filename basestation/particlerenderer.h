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

    void slotSetVboInfoGridInformationGain(const quint32 vboPressure, const Box3D& boundingBoxGrid, const Vector3i& gridCells);
    void slotSetVboInfoGridOccupancy(const quint32 vbo, const Box3D& gridBoundingBox, const Vector3i& gridCells);
    void slotSetVboInfoGridPathFinder(const quint32 vbo, const Box3D &gridBoundingBox, const Vector3i &gridCells);
    void slotSetVboInfoParticles(const quint32 vboPositions, const quint32 vboColors, const quint32 count, const Box3D particleSystemBoundingBox);

private:
    bool mIsInitialized;
    bool mRenderParticles, mRenderInformationGain, mRenderBoundingBox, mRenderOccupancyGrid, mRenderPathFinderGrid;

    ShaderProgram *mShaderProgramDefault, *mShaderProgramParticles, *mShaderProgramGrid;
    quint32 mNumberOfParticles;

    Box3D mBoundingBoxGridInformationGain;
    Vector3i mGridInformationGainCellCount;

    Box3D mBoundingBoxGridOccupancy;
    Vector3i mGridOccupancyCellCount;

    Box3D mBoundingBoxGridPathFinder;
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
