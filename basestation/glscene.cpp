#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <QDebug>
#include <openglutilities.h>

#include "glscene.h"
#include "flightplannerparticles.h"

GlScene::GlScene() :
    mWayPointsAhead(nullptr),
    mWayPointsPassed(nullptr),
    mVolumeGlobal(nullptr),
    mVolumeLocal(nullptr),
    mShaderProgramDefault(nullptr),
    mShaderProgramWaypoint(nullptr),
    mShaderProgramPointCloud(nullptr),
    mShaderProgramRawScanRays(nullptr),
    mLastFlightControllerValues(nullptr),
    mLastKnownVehiclePose(nullptr),
    mActiveWayPointVisualizationIndex(-1)
{
    mIsInitialized = false;
    mFlightPlannerProcessingState = FlightPlannerProcessingState::Idle;

    mRenderBoundingBoxGlobal = true;
    mRenderBoundingBoxLocal = true;
    mRenderWayPointsAhead = true;
    mRenderWayPointsPassed = true;
    mRenderRawScanRays = true;
    mRenderAxisBase = true;
    mRenderAxisVehicle = true;
    mRenderTrajectory = true;
    mRenderVehicle = true;
    mRenderSatelliteSignals = false;

    mRenderParticles = false;
    mRenderInformationGain = false;
    mRenderOccupancyGrid = false;
    mRenderPathPlannerGrid = false;

    mVboGridMapOfInformationGain = 0;
    mVboGridMapOfOccupancy = 0;
    mVboGridMapOfPathPlanner = 0;
    mVboBoundingBoxVolumeGlobal = 0;
    mVboBoundingBoxVolumeLocal = 0;
    mVboParticlePositions = 0;
    //mVboRawScanRays = 0;

    mVboVehiclePathElementSize = sizeof(QVector3D) + sizeof(QVector4D); // position and color with alpha
    mVboVehiclePathBytesMaximum = (3600 * 50 * mVboVehiclePathElementSize); // For a flight time of one hour
    mVboVehiclePathBytesCurrent = 0;

    mParticleRadius = 0.0f;
    mParticleOpacity = 1.0f;
    mNumberOfParticles = 0;

    mMaxPointVisualizationDistance = 1000.0f;
    mPointCloudPointSize = 1.0;
    mPointCloudPointAlpha = 1.0;
    mPointCloudColorLow = -2.0f;
    mPointCloudColorHigh = 10.0f;

    // Put camera to a good starting position
    mCameraPosition = QVector3D(0.0f, 0.0f, 500.0f);
    mCameraRotation.setX(-30.0f);
    mCameraZoom = 0.5f;
}

GlScene::~GlScene()
{
    mVaoAxes->deleteLater();
    mVaoPointCloud->deleteLater();
    mVaoTrajectory->deleteLater();
    mVaoBoundingBoxGlobal->deleteLater();
    mVaoBoundingBoxLocal->deleteLater();
    mVaoParticles->deleteLater();
    mVaoGridMapOfInformationGain->deleteLater();
    mVaoGridMapOfOccupancy->deleteLater();
    mVaoGridMapOfPathPlanner->deleteLater();
    mVaoWayPointsAhead->deleteLater();
    mVaoSatelliteSignals->deleteLater();

    // TODO: Does this delete the shaders? No.
    mShaderProgramDefault->deleteLater();
    mShaderProgramGrid->deleteLater();
    mShaderProgramParticles->deleteLater();
    mShaderProgramPointCloud->deleteLater();
    mShaderProgramRawScanRays->deleteLater();
    mShaderProgramWaypoint->deleteLater();
    mShaderProgramSatelliteSignals->deleteLater();


    for(int i=0;i<mRawScanVisualizations.size();i++)
    {
        OpenGlUtilities::deleteVbo(mRawScanVisualizations[i]->vbo);
        mRawScanVisualizations[i]->vao->deleteLater();
    }
}

void GlScene::slotUpdateMatrixModelToCamera()
{
    const QVector3D camLookAt = mCamLookAtOffset + (mLastKnownVehiclePose ? mLastKnownVehiclePose->getPosition() : QVector3D());

    QQuaternion cameraRotation =
            QQuaternion::fromAxisAndAngle(QVector3D(0.0f, 0.0f, 1.0f), mCameraRotation.z())
            * QQuaternion::fromAxisAndAngle(QVector3D(0.0f, 1.0f, 0.0f), mCameraRotation.y())
            * QQuaternion::fromAxisAndAngle(QVector3D(1.0f, 0.0f, 0.0f), mCameraRotation.x());

    const QVector3D camPos = cameraRotation.rotatedVector(mCameraPosition);

    // Write the modelToCamera matrix into our UBO
    mMatrixModelToCamera.setToIdentity();
    mMatrixModelToCamera.lookAt(camPos, camLookAt, QVector3D(0.0f, 1.0f, 0.0f));

    glBindBuffer(GL_UNIFORM_BUFFER, mUboId);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, 64, mMatrixModelToCamera.constData());
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
    //qDebug() << __PRETTY_FUNCTION__ << "camPos" << camPos << "lookAt" << camLookAt << "modelToCamera:" << matrixModelToCamera << matrixModelToCamera.inverted() * QVector3D();
}

void GlScene::slotUpdateMatrixCameraToClip(const quint32 windowWidth, const quint32 windowHeight)
{
    // OpenGL 4 core profile: We set the second matrix (perspective = cameraToClip) in the UBO
    mMatrixCameraToClip.setToIdentity();

    if(false)
    {
        mMatrixCameraToClip.perspective(50.0f * mCameraZoom, (float)windowWidth/(float)windowHeight, 10.0f, +1000.0f);
    }
    else
    {
        mMatrixCameraToClip.ortho(
                    -(windowWidth/2.0f) * mCameraZoom,
                    windowWidth/2.0f * mCameraZoom,
                    -(windowHeight/2.0f) * mCameraZoom,
                    windowHeight/2.0f * mCameraZoom,
                    10.0f, 1000.0f);
    }

    // Set the second matrix (cameraToClip) in the UBO
    glBindBuffer(GL_UNIFORM_BUFFER, mUboId);
    glBufferSubData(GL_UNIFORM_BUFFER, 64, 64, mMatrixCameraToClip.constData());
    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    //qDebug() << "GlWindow::slotUpdateMatrixCameraToClip(): resized gl viewport to" << windowWidth << windowHeight << "- using zoom" << mCameraZoom << "and setting perspective/cameraToClip matrix" << matrixCameraToClip;
}

void GlScene::setCameraRotation(const float yaw, const float pitch, const float roll)
{
    mCameraRotation = QVector3D(pitch, yaw, roll);
    slotUpdateMatrixModelToCamera();
}

void GlScene::rotateCamera(const float yaw, const float pitch, const float roll)
{
    //qDebug() << __PRETTY_FUNCTION__ << yaw << pitch << roll;
    mCameraRotation.setX(qBound(-89.9f, float(mCameraRotation.x() + pitch), 89.9f));
    mCameraRotation.setY(fmod(mCameraRotation.y() + yaw, 360.0f));
    mCameraRotation.setZ(qBound(-89.9f, float(mCameraRotation.z() + roll), 89.9f));
    slotUpdateMatrixModelToCamera();
}

void GlScene::moveCameraTarget(const float x, const float y, const float z)
{
    mCamLookAtOffset += QVector3D(x, y, z);
    slotUpdateMatrixModelToCamera();
}

void GlScene::initialize()
{
    qDebug() << __PRETTY_FUNCTION__;

    if(!initializeOpenGLFunctions())
    {
        qDebug() << __PRETTY_FUNCTION__ << "couldn't initialize OpenGL 4.3 Core context, quitting.";
        exit(1);
    }

    mShaderProgramDefault = new ShaderProgram(this, "shader-default-vertex.c", "", "shader-default-fragment.c");
    mShaderProgramGrid = new ShaderProgram(this, "shader-grid-vertex.c", "shader-grid-geometry.c", "shader-grid-fragment.c");
    mShaderProgramParticles = new ShaderProgram(this, "shader-particles-vertex.c", "shader-particles-geometry.c", "shader-particles-fragment.c");
    mShaderProgramPointCloud = new ShaderProgram(this, "shader-pointcloud-vertex.c", "", "shader-pointcloud-fragment.c");
    mShaderProgramRawScanRays = new ShaderProgram(this, "shader-rawscanrays-vertex.c", "shader-rawscanrays-geometry.c", "shader-default-fragment.c");
    mShaderProgramWaypoint = new ShaderProgram(this, "shader-waypoint-vertex.c", "", "shader-waypoint-fragment.c");
    mShaderProgramSatelliteSignals = new ShaderProgram(this, "shader-satellitesignals-vertex.c", "shader-satellitesignals-geometry.c", "shader-satellitesignals-fragment.c");

    // Create the uniform buffer object (UBO) for all members of the UBO-Block
    mUboSize = 64 + 64; // One Matrix4x4 has 16 floats with 4 bytes each, giving 64 bytes.
    glGenBuffers(1, &mUboId);
    glBindBuffer(GL_UNIFORM_BUFFER, mUboId);
    glBufferData(GL_UNIFORM_BUFFER, mUboSize, NULL, GL_STREAM_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
    // Now bind this UBO to a uniform-block binding-point
    glBindBufferRange(GL_UNIFORM_BUFFER, ShaderProgram::blockBindingPointGlobalMatrices, mUboId, 0, mUboSize);

    // Tell OpenGL what to cull from triangles. See http://www.arcsynthesis.org/gltut/Positioning/Tutorial%2004.html
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CW);

    // Enable Z Buffer. See http://www.arcsynthesis.org/gltut/Positioning/Tut05%20Overlap%20and%20Depth%20Buffering.html
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LEQUAL);
    glDepthRange(0.0f, 1.0f);
    glClearDepth(1.0f);

    // Set Line Antialiasing
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    mVboBoundingBoxVolumeGlobal = OpenGlUtilities::createVbo();
    mVboBoundingBoxVolumeLocal = OpenGlUtilities::createVbo();

    // Create axes positions and colors
    const float data[] = {
        // Gray -X to 0
        -10.0f, +00.0f, +00.0f, 1.0f,
        +00.0f, +00.0f, +00.0f, 1.0f,

        // Red 0 to +X
        +00.0f, +00.0f, +00.0f, 1.0f,
        +10.0f, +00.0f, +00.0f, 1.0f,

        // Gray -Y to 0
        +00.0f, -10.0f, +00.0f, 1.0f,
        +00.0f, +00.0f, +00.0f, 1.0f,

        // Green 0 to +Y
        +00.0f, +00.0f, +00.0f, 1.0f,
        +00.0f, +10.0f, +00.0f, 1.0f,

        // Gray -Z to 0
        +00.0f, +00.0f, -10.0f, 1.0f,
        +00.0f, +00.0f, +00.0f, 1.0f,

        // Blue 0 to +Z
        +00.0f, +00.0f, +00.0f, 1.0f,
        +00.0f, +00.0f, +10.0f, 1.0f,

        0.5f, 0.5f, 0.5f, 0.5f, // Gray
        0.5f, 0.5f, 0.5f, 0.5f, // Gray

        1.0f, 0.0f, 0.0f, 0.5f, // Red
        1.0f, 0.0f, 0.0f, 0.5f, // Red

        0.5f, 0.5f, 0.5f, 0.5f, // Gray
        0.5f, 0.5f, 0.5f, 0.5f, // Gray

        0.0f, 1.0f, 0.0f, 0.5f, // Green
        0.0f, 1.0f, 0.0f, 0.5f, // Green

        0.5f, 0.5f, 0.5f, 0.5f, // Gray
        0.5f, 0.5f, 0.5f, 0.5f, // Gray

        0.0f, 0.0f, 1.0f, 0.5f, // Blue
        0.0f, 0.0f, 1.0f, 0.5f  // Blue
    };

    mVboAxes = OpenGlUtilities::createVbo(sizeof(data), data, GL_STATIC_DRAW);

    // Create Vertex Array Objects to contain our state and vertex specification
    // Vertex specification of axes
    mVaoAxes = new QOpenGLVertexArrayObject(this);
    mVaoAxes->create();
    mVaoAxes->bind();
    glBindBuffer(GL_ARRAY_BUFFER, mVboAxes);
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0); // positions
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, (void*)(12 * sizeof(float) * 4)); // colors
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    mVaoAxes->release();

    // Vertex specification of pointcloud is different from frame to frame due to different point count :|
    mVaoPointCloud = new QOpenGLVertexArrayObject(this);
    mVaoPointCloud->create();

    // Vertex specification of raw scan - created when data comes in
    //mVaoRawScan = new QOpenGLVertexArrayObject(this);

    // Vertex specification of trajectory
    mVboVehiclePath = OpenGlUtilities::createVbo(mVboVehiclePathBytesMaximum);
    mVaoTrajectory = new QOpenGLVertexArrayObject(this);
    mVaoTrajectory->create();
    mVaoTrajectory->bind();
    glBindBuffer(GL_ARRAY_BUFFER, mVboVehiclePath);
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    // Stride is NOT the number of useless bytes between two packets, its the
    // "distance" between two beginnings of two consecutive useful packets
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 28, 0); // position.
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 28, (void*)12); // color
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    mVaoTrajectory->release();

    // Vertex specification for bounding boxes will be created when bboxes are set
    mVaoBoundingBoxGlobal = new QOpenGLVertexArrayObject(this);
    mVaoBoundingBoxLocal = new QOpenGLVertexArrayObject(this);

    // Vertex specification for waypoint lists will be created when bboxes are set
    mVaoWayPointsAhead = new QOpenGLVertexArrayObject(this);
    mVaoWayPointsPassed = new QOpenGLVertexArrayObject(this);

    mVaoSatelliteSignals = new QOpenGLVertexArrayObject(this);

    mVaoParticles = new QOpenGLVertexArrayObject(this);

    mVaoGridMapOfInformationGain = new QOpenGLVertexArrayObject(this);

    mVaoGridMapOfOccupancy = new QOpenGLVertexArrayObject(this);
    mVaoGridMapOfOccupancy->create();
    mVaoGridMapOfOccupancy->bind();
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 1, GL_UNSIGNED_BYTE, GL_TRUE, 0, 0);
    mVaoGridMapOfOccupancy->release();

    mVaoGridMapOfPathPlanner = new QOpenGLVertexArrayObject(this);
    mVaoGridMapOfPathPlanner->create();
    mVaoGridMapOfPathPlanner->bind();
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 1, GL_UNSIGNED_BYTE, GL_TRUE, 0, 0);
    mVaoGridMapOfPathPlanner->release();


    // Find the models and load them
    QDir modelPath = QDir::current();
    modelPath.cdUp();

    mModelVehicle = new Model(QFile(modelPath.absolutePath() + "/media/oktokopter.obj"), QString("../media/"), this);
    mModelThrust = new Model(QFile(modelPath.absolutePath() + "/media/cone-grey.obj"), QString("../media/"), this);
    mModelConeYaw = new Model(QFile(modelPath.absolutePath() + "/media/cone-green.obj"), QString("../media/"), this);
    mModelConePitch = new Model(QFile(modelPath.absolutePath() + "/media/cone-red.obj"), QString("../media/"), this);
    mModelConeRoll = new Model(QFile(modelPath.absolutePath() + "/media/cone-blue.obj"), QString("../media/"), this);
    mModelHoverPosition = new Model(QFile(modelPath.absolutePath() + "/media/target.obj"), QString("../media/"), this);

    mModelTrajectoryStart = new Model(QFile(modelPath.absolutePath() + "/media/trajectory-start.obj"), QString("../media/"), this);
    mModelTrajectoryGoal = new Model(QFile(modelPath.absolutePath() + "/media/trajectory-goal.obj"), QString("../media/"), this);

    mModelVelocityArrow = new Model(QFile(modelPath.absolutePath() + "/media/velocity-arrow.obj"), QString("../media/"), this);

    mModelControllerP = new Model(QFile(modelPath.absolutePath() + "/media/controller-p.obj"), QString("../media/"), this);
    mModelControllerI = new Model(QFile(modelPath.absolutePath() + "/media/controller-i.obj"), QString("../media/"), this);
    mModelControllerD = new Model(QFile(modelPath.absolutePath() + "/media/controller-d.obj"), QString("../media/"), this);

    slotUpdateMatrixModelToCamera();

    mIsInitialized = true;
}

void GlScene::slotSetVboInfoParticles(const quint32 vboPositions, const quint32 count, const float particleRadius, const Box3D particleSystemBoundingBox)
{
    mVboParticlePositions = vboPositions;
    mNumberOfParticles = count;
    mParticleRadius = particleRadius;

    OpenGlUtilities::setVboToBoundingBox(mVboBoundingBoxVolumeLocal, &particleSystemBoundingBox);

    if(!mVaoParticles->isCreated()) mVaoParticles->create();

    mVaoParticles->bind();
    glBindBuffer(GL_ARRAY_BUFFER, mVboParticlePositions);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
    mVaoParticles->release();

    qDebug() << "GlScene::slotSetVboInfoParticles(): will render VBO pos" << mVboParticlePositions << "containing" << mNumberOfParticles << "particles";
}

void GlScene::slotSetVboInfoGridInformationGain(const quint32 vboPressure, const Box3D& gridBoundingBox, const Vector3i &gridCells)
{
    mVboGridMapOfInformationGain = vboPressure;
    mBoundingBoxGridInformationGain = gridBoundingBox;
    mGridInformationGainCellCount = gridCells;

    if(!mVaoGridMapOfInformationGain->isCreated()) mVaoGridMapOfInformationGain->create();
    mVaoGridMapOfInformationGain->bind();
    glBindBuffer(GL_ARRAY_BUFFER, mVboGridMapOfInformationGain);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 1, GL_UNSIGNED_BYTE, GL_TRUE, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    mVaoGridMapOfInformationGain->release();
    qDebug() << "GlScene::slotSetVboInfoGridInformationGain(): will render VBO pos" << mVboGridMapOfInformationGain << "with" << gridCells << "cells using bbox" << gridBoundingBox;
}

void GlScene::slotSetVboInfoGridOccupancy(const quint32 vbo, const Box3D& gridBoundingBox, const Vector3i &gridCells)
{
    mVboGridMapOfOccupancy = vbo;
    mBoundingBoxGridOccupancy = gridBoundingBox;
    mGridOccupancyCellCount = gridCells;

    if(!mVaoGridMapOfOccupancy->isCreated()) mVaoGridMapOfOccupancy->create();
    mVaoGridMapOfOccupancy->bind();
    glBindBuffer(GL_ARRAY_BUFFER, mVboGridMapOfOccupancy);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 1, GL_UNSIGNED_BYTE, GL_TRUE, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    mVaoGridMapOfOccupancy->release();
    qDebug() << "GlScene::slotSetVboInfoGridOccupancyGain(): will render VBO pos" << mVboGridMapOfOccupancy << "with" << gridCells << "cells using bbox" << gridBoundingBox;
}

void GlScene::slotSetVboInfoGridPathPlanner(const quint32 vbo, const Box3D& gridBoundingBox, const Vector3i &gridCells)
{
    mVboGridMapOfPathPlanner = vbo;
    mBoundingBoxGridPathFinder = gridBoundingBox;
    mGridPathFinderCellCount = gridCells;

    if(!mVaoGridMapOfPathPlanner->isCreated()) mVaoGridMapOfPathPlanner->create();
    mVaoGridMapOfPathPlanner->bind();
    glBindBuffer(GL_ARRAY_BUFFER, mVboGridMapOfPathPlanner);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 1, GL_UNSIGNED_BYTE, GL_TRUE, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    mVaoGridMapOfPathPlanner->release();
    qDebug() << "GlScene::slotSetVboInfoGridPathFinder(): will render VBO pos" << mVboGridMapOfPathPlanner << "with" << gridCells << "cells using bbox" << gridBoundingBox;
}

void GlScene::render()
{
    Q_ASSERT(mIsInitialized);

    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);

    // In the geometry shader (e.g. grid rendering), the quads are sometimes oriented away from the camera.
    // Thus, we need to do two-sided rendering!
    glDisable(GL_CULL_FACE);

    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Beau.Ti.Ful!
    {
        mShaderProgramPointCloud->bind();
        mShaderProgramPointCloud->setUniformValue("maxPointVisualizationDistance", (GLfloat)pow(mMaxPointVisualizationDistance, 2.0)); // distances are squared in the point cloud, too!
        mShaderProgramPointCloud->setUniformValue("pointcloudPointAlpha", mPointCloudPointAlpha);
        mShaderProgramPointCloud->setUniformValue("pointcloudColorLow", mPointCloudColorLow);
        mShaderProgramPointCloud->setUniformValue("pointcloudColorHigh", mPointCloudColorHigh);
        mShaderProgramPointCloud->setUniformValue("pointcloudPointSize", mPointCloudPointSize);
        for(int i=0;i<mPointCloudsToRender.size();i++)
        {
            QVector<PointCloud::RenderInfo*>* renderInfoList = mPointCloudsToRender.at(i)->getRenderInfo();
            for(int j=0;j<renderInfoList->size();j++)
            {
                PointCloud::RenderInfo* renderInfo = renderInfoList->at(j);

                if(renderInfo->size == 0) continue;

                // If the pointcloud has a color set, use it. Otherwise, use the jet colormap.
                if(renderInfo->color.isValid())
                {
                    mShaderProgramPointCloud->setUniformValue("useFixedColor", true);
                    mShaderProgramPointCloud->setUniformValue("fixedColor", renderInfo->color);
                }
                else
                {
                    mShaderProgramPointCloud->setUniformValue("useFixedColor", false);
                }

                if(!renderInfo->vao)
                {
                    qDebug() << __PRETTY_FUNCTION__ << "now setting up VAO for pointcloud with vbo" << renderInfo->vbo;
                    renderInfo->vao = new QOpenGLVertexArrayObject;
                    renderInfo->vao->create();
                    renderInfo->vao->bind();
                    glBindBuffer(GL_ARRAY_BUFFER, renderInfo->vbo);
                    glEnableVertexAttribArray(0);
                    glVertexAttribPointer(0, renderInfo->elementSize, GL_FLOAT, GL_FALSE, renderInfo->stride, 0);
                    glPointSize(mPointCloudPointSize);
                    glDrawArrays(GL_POINTS, 0, renderInfo->size); // Number of elements, not bytes
                    glBindBuffer(GL_ARRAY_BUFFER, 0);
                    renderInfo->vao->release();
                }

                renderInfo->vao->bind();
                glDrawArrays(GL_POINTS, 0, renderInfo->size); // Number of elements, not bytes
                renderInfo->vao->release();
            }
        }
        mShaderProgramPointCloud->release();

        if(mRenderRawScanRays)
        {
            mShaderProgramRawScanRays->bind();
            for(int i=0;i<mRawScanVisualizations.size();i++)
            {
                RawScanRayVisualization* rsrv = mRawScanVisualizations[i];
                quint8 rayStride = 0; // how many rays to ignore between visualized rays
                mShaderProgramRawScanRays->setUniformValue("rayStride", rayStride);

                QMatrix4x4 lidarMatrix;
                if(mLastKnownVehiclePose) lidarMatrix = mLastKnownVehiclePose->getMatrixCopy();

                QColor rayColor(i==0?255:100, i==2?255:100, i==1?255:100, 128);

                mShaderProgramRawScanRays->setUniformValue("useFixedColor", true);
                mShaderProgramRawScanRays->setUniformValue("fixedColor", rayColor);
                mShaderProgramRawScanRays->setUniformValue("firstUsableDistance", rsrv->distanceIndexFirst);
                mShaderProgramRawScanRays->setUniformValue("useMatrixExtra", true);
                mShaderProgramRawScanRays->setUniformValue("matrixExtra", lidarMatrix * rsrv->relativeScannerPose);

                rsrv->vao->bind();
                glDrawArrays(GL_POINTS, 0, (rsrv->distanceIndexLast-rsrv->distanceIndexFirst) / (rayStride+1));
                rsrv->vao->release();
            }
            mShaderProgramRawScanRays->release();
        }

        mShaderProgramDefault->bind();
        {
            if(mRenderTrajectory && mVboVehiclePathBytesCurrent > 0)
            {
                // Render the vehicle's path - same shader, but variable color
                mShaderProgramDefault->setUniformValue("useFixedColor", false);
                mVaoTrajectory->bind();
                glBindBuffer(GL_ARRAY_BUFFER, mVboVehiclePath);
                glPointSize(1.0);
                glDrawArrays(GL_POINTS, 0, (mVboVehiclePathBytesCurrent / mVboVehiclePathElementSize));
                glBindBuffer(GL_ARRAY_BUFFER, 0);
                mVaoTrajectory->release();
            }

            // Prepare axis rendering if desired
            if(mRenderAxisBase || mRenderAxisVehicle)
            {
                mShaderProgramDefault->setUniformValue("useFixedColor", false);
                mVaoAxes->bind();
                glBindBuffer(GL_ARRAY_BUFFER, mVboAxes);
            }

            // At the origin
            if(mRenderAxisBase)
            {
                glDrawArrays(GL_LINES, 0, 12);
            }

            // At the vehicle
            if(mRenderAxisVehicle && mLastKnownVehiclePose)
            {
                mShaderProgramDefault->setUniformValue("useMatrixExtra", true);
                mShaderProgramDefault->setUniformValue("matrixExtra", mLastKnownVehiclePose->getMatrixConst());
                glDrawArrays(GL_LINES, 0, 12);
                mShaderProgramDefault->setUniformValue("useMatrixExtra", false);
            }

            // Clean up axis rendering
            if(mRenderAxisBase || mRenderAxisVehicle)
            {
                mVaoAxes->release();
                glBindBuffer(GL_ARRAY_BUFFER, 0);
            }
        }
        mShaderProgramDefault->release();
    }
    glDisable(GL_BLEND);

    QMatrix4x4 transformVehicle;

    // At startup, a vehiclePose might not exist yet. If so, use the identity matrix
    if(mLastKnownVehiclePose) transformVehicle = mLastKnownVehiclePose->getMatrixConst();

    // Only show controller input if its present and less than 500 ms old.
    if(mLastFlightControllerValues && mLastKnownVehiclePose->timestamp - mLastFlightControllerValues->lastKnownPose.timestamp < 500)
    {
        //        qDebug() << "now visualizing motioncommand from" << mLastFlightControllerValues->lastKnownPose.timestamp << mLastFlightControllerValues->motionCommand;

        // Render controller yaw input
        if(fabs(mLastFlightControllerValues->motionCommand.yaw) > 0.1f)
        {
            QMatrix4x4 trYaw(transformVehicle);
            trYaw.rotate(mLastFlightControllerValues->motionCommand.yaw, QVector3D(0,1,0));
            if(mLastFlightControllerValues->motionCommand.yaw > 0)
                trYaw.rotate(90.0f, QVector3D(0,0,1));
            else
                trYaw.rotate(-90.0f, QVector3D(0,0,1));
            trYaw.translate(0, 0, -0.7);
            mModelConeYaw->slotSetModelTransform(trYaw);
            mModelConeYaw->render();

            QMatrix4x4 trControllerYaw(transformVehicle);
            trControllerYaw.translate(0, 0, -0.7); // move forward on red arm, away from x axis.
            trControllerYaw.rotate(-90.0f, QVector3D(1,0,0)); // pitch forward, so bars go forward instead of upwards.
            renderController(trControllerYaw, &mLastFlightControllerValues->controllerYaw);
        }

        // Render controller pitch input
        if(fabs(mLastFlightControllerValues->motionCommand.pitch) > 0.01f)
        {
            QMatrix4x4 trPitch(transformVehicle);
            trPitch.translate(0, fabs(mLastFlightControllerValues->motionCommand.pitch) * 0.1f, 0);
            if(mLastFlightControllerValues->motionCommand.pitch > 0)
                trPitch.translate(0, 0, -0.7);
            else
                trPitch.translate(0, 0, 0.7);
            mModelConePitch->slotSetModelTransform(trPitch);
            mModelConePitch->render();

            QMatrix4x4 trPitchController(transformVehicle);
            // turn right 90deg, so that renderController can move pos or neg depending on controller outputs?
            trPitchController.rotate(90.0f, QVector3D(0.0f, 1.0f, 0.0f));
            renderController(trPitchController, &mLastFlightControllerValues->controllerPitch);
        }

        // Render controller roll input
        if(fabs(mLastFlightControllerValues->motionCommand.roll) > 0.01f)
        {
            QMatrix4x4 trRoll(transformVehicle);
            trRoll.translate(0, fabs(mLastFlightControllerValues->motionCommand.roll) * 0.1f, 0);
            trRoll.rotate(-90.0f, QVector3D(0,1,0));
            if(mLastFlightControllerValues->motionCommand.roll > 0)
                trRoll.translate(0, 0, -0.7);
            else
                trRoll.translate(0, 0, 0.7);
            mModelConeRoll->slotSetModelTransform(trRoll);
            mModelConeRoll->render();

            QMatrix4x4 trRollController(transformVehicle);
            // turn right 90deg, so that renderController can move pos or neg depending on controller outputs?
            trRollController.rotate(0.0f, QVector3D(0.0f, 1.0f, 0.0f));
            renderController(trRollController, &mLastFlightControllerValues->controllerRoll);
        }

        if(mLastFlightControllerValues->motionCommand.thrust != MotionCommand::thrustHover && mLastFlightControllerValues->motionCommand.thrust > 0)
        {
            // Render controller thrust input
            QMatrix4x4 trThrust(transformVehicle);
            // Move the thrust-arrow according to, well, thrust.
            trThrust.translate(
                        QVector3D(
                            0,
                            (mLastFlightControllerValues->motionCommand.thrust - MotionCommand::thrustHover) / 50.0f,
                            0)
                        );
            // Make the arrow point downwards if thrust is below hover-thrust
            if(mLastFlightControllerValues->motionCommand.thrust < MotionCommand::thrustHover)
                trThrust.rotate(180.0f, QVector3D(1,0,0));
            mModelThrust->slotSetModelTransform(trThrust);
            mModelThrust->render();

            QMatrix4x4 trThrustController(transformVehicle);
            // turn right 90deg, so that renderController can move pos or neg depending on controller outputs?
            trThrustController.rotate(45.0f, QVector3D(0,1,0));
            //            renderController(trThrustController, &mLastFlightControllerValues->controllerThrust);
        }

        // Render hover position!
        if(!mLastFlightControllerValues->hoverPosition.isNull())
        {
            QMatrix4x4 tr;
            tr.translate(mLastFlightControllerValues->hoverPosition);
            mModelHoverPosition->slotSetModelTransform(tr);
            mModelHoverPosition->render();
        }

        // Render start and goal
        if(!mLastFlightControllerValues->trajectoryStart.isNull())
        {
            QMatrix4x4 tr;
            tr.translate(mLastFlightControllerValues->trajectoryStart);
            mModelTrajectoryStart->slotSetModelTransform(tr);
            mModelTrajectoryStart->render();
        }
        if(!mLastFlightControllerValues->trajectoryGoal.isNull())
        {
            QMatrix4x4 tr;
            tr.translate(mLastFlightControllerValues->trajectoryGoal);
            mModelTrajectoryGoal->slotSetModelTransform(tr);
            mModelTrajectoryGoal->render();
        }
    }

    // Render velocities - preferably from the flightcontroller.
    const Pose* p;
    if(mLastFlightControllerValues && mLastKnownVehiclePose->timestamp - mLastFlightControllerValues->lastKnownPose.timestamp < 500)
        p = &mLastFlightControllerValues->lastKnownPose;
    else
        p = mLastKnownVehiclePose;

    if(mLastFlightControllerValues)
    {
        QVector3D velocity = p->getVelocity();
        //qDebug() << "GlWidget::slotRenderNow(): fcv-time" << mLastFlightControllerValues->timestamp << "pose:" << *p;
        velocity.setY(0.0f);
        const float velocityScalar = velocity.length();

        const float angleBetweenFrontAndVelocity = Pose::getShortestTurnRadians(p->getYawRadians() - atan2(-velocity.x(), -velocity.z()));

        float velocityOverPitch = cos(angleBetweenFrontAndVelocity) * velocityScalar;
        float velocityOverRoll = sin(angleBetweenFrontAndVelocity) * velocityScalar;

        QMatrix4x4 trVelocity;
        trVelocity.translate(p->getPosition());
        trVelocity.rotate(RAD2DEG(atan2(-velocity.x(), -velocity.z())), QVector3D(0,1,0));
        trVelocity.scale(1.0f, 1.0f, velocityScalar);
        trVelocity.translate(0.0f, 0.2f, 0.0f);
        mModelVelocityArrow->slotSetModelTransform(trVelocity);
        mModelVelocityArrow->render();

        QMatrix4x4 trVelocityPitch;
        trVelocityPitch.translate(p->getPosition());
        trVelocityPitch.rotate(p->getYawDegrees(), QVector3D(0,1,0));
        trVelocityPitch.scale(1.0f, 1.0f, velocityOverPitch);
        trVelocityPitch.translate(0.0f, 0.2f, 0.0f);
        mModelVelocityArrow->slotSetModelTransform(trVelocityPitch);
        mModelVelocityArrow->render();

        QMatrix4x4 trVelocityRoll;
        trVelocityRoll.translate(p->getPosition());
        trVelocityRoll.rotate(p->getYawDegrees() - 90.0f, QVector3D(0,1,0));
        trVelocityRoll.scale(1.0f, 1.0f, velocityOverRoll);
        trVelocityRoll.translate(0.0f, 0.2f, 0.0f);
        mModelVelocityArrow->slotSetModelTransform(trVelocityRoll);
        mModelVelocityArrow->render();
    }

    if(mRenderVehicle)
    {
        mModelVehicle->slotSetModelTransform(transformVehicle);
        mModelVehicle->render();
    }

    if(mRenderSatelliteSignals && mVaoSatelliteSignals->isCreated())
    {
        mShaderProgramSatelliteSignals->bind();
        mShaderProgramSatelliteSignals->setUniformValue("matrixExtra", mLastKnownVehiclePose->getMatrixConst());
        mShaderProgramSatelliteSignals->setUniformValue("numSatsGps", mNumberOfSatellitesGps);
        mShaderProgramSatelliteSignals->setUniformValue("numSatsGlonass", mNumberOfSatellitesGlonass);

        mVaoSatelliteSignals->bind();
        glDrawArrays(GL_POINTS, 0, mNumberOfSatelliteSignals);
        mVaoSatelliteSignals->release();
        mShaderProgramSatelliteSignals->release();
    }

    if(mRenderBoundingBoxGlobal && mVaoBoundingBoxGlobal->isCreated())
    {
        mShaderProgramDefault->bind();
        mShaderProgramDefault->setUniformValue("useFixedColor", true);

        //glEnable(GL_BLEND);
        //glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Beau.Ti.Ful!
        {
            mVaoBoundingBoxGlobal->bind();
            //glBindBuffer(GL_ARRAY_BUFFER, mVboBoundingBoxVolumeGlobal);
            // draw the lines around the box
            mShaderProgramDefault->setUniformValue("fixedColor", QVector4D(0.3f, 0.3f, 1.0f, 0.8f));
            glDrawArrays(GL_LINE_LOOP, 0, 4);
            glDrawArrays(GL_LINE_LOOP, 4, 4);
            glDrawArrays(GL_LINE_LOOP, 8, 4);
            glDrawArrays(GL_LINE_LOOP, 12, 4);
            glDrawArrays(GL_LINE_LOOP, 16, 4);
            glDrawArrays(GL_LINE_LOOP, 20, 4);

            // draw a half-transparent box, GL_QUADS was removed from opengl core 4
            //mShaderProgramDefault->setUniformValue("fixedColor", QVector4D(1.0f, 1.0f, 1.0f, 0.015f));
            //glDrawArrays(GL_QUADS, 0, 24);
            mVaoBoundingBoxGlobal->release();
        }
        //glDisable(GL_BLEND);
        mShaderProgramDefault->release();
    }

    if(mRenderBoundingBoxLocal && mVaoBoundingBoxLocal->isCreated())
    {
        mShaderProgramDefault->bind();
        mShaderProgramDefault->setUniformValue("useFixedColor", true);

        glEnable(GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Beau.Ti.Ful!
        {
            mVaoBoundingBoxLocal->bind();
            //glBindBuffer(GL_ARRAY_BUFFER, mVboBoundingBoxVolumeLocal);
            // draw the lines around the box
            mShaderProgramDefault->setUniformValue("fixedColor", QVector4D(0.4f, 0.4f, 0.4f, 0.8f));
            glDrawArrays(GL_LINE_LOOP, 0, 4);
            glDrawArrays(GL_LINE_LOOP, 4, 4);
            glDrawArrays(GL_LINE_LOOP, 8, 4);
            glDrawArrays(GL_LINE_LOOP, 12, 4);
            glDrawArrays(GL_LINE_LOOP, 16, 4);
            glDrawArrays(GL_LINE_LOOP, 20, 4);

            // draw a half-transparent box
            //mShaderProgramDefault->setUniformValue("fixedColor", QVector4D(1.0f, 1.0f, 1.0f, 0.015f));
            //glDrawArrays(GL_QUADS, 0, 24);
            mVaoBoundingBoxLocal->release();
        }
        glDisable(GL_BLEND);
        mShaderProgramDefault->release();
    }

    // Render information gain
    if(mVboGridMapOfInformationGain != 0 && mVaoGridMapOfInformationGain->isCreated() && (mRenderInformationGain || mFlightPlannerProcessingState == FlightPlannerProcessingState::ParticleSimulation || mFlightPlannerProcessingState == FlightPlannerProcessingState::WayPointComputation))
    {
        // Draw grid with waypoint pressure
        mShaderProgramGrid->bind();

        mShaderProgramGrid->setUniformValue("fixedColor", QColor(255,0,0));
        // If we have a value of (quint8)1, that'll be 1/255=0.004 in the shader's float. Amplify this?
        mShaderProgramGrid->setUniformValue("alphaMultiplication", 2.0f);
        mShaderProgramGrid->setUniformValue("alphaExponentiation", 1.0f);
        mShaderProgramGrid->setUniformValue("quadSizeFactor", 1.0f);

        // Set uniform values in the shader program
        Q_ASSERT(mShaderProgramGrid->uniformLocation("boundingBoxMin") != -1);
        mShaderProgramGrid->setUniformValue("boundingBoxMin", mBoundingBoxGridInformationGain.min);

        Q_ASSERT(mShaderProgramGrid->uniformLocation("boundingBoxMax") != -1);
        mShaderProgramGrid->setUniformValue("boundingBoxMax", mBoundingBoxGridInformationGain.max);

        // gridSize is a uint3, not sure how to set this with qt, so lets do opengl:
        Q_ASSERT(mShaderProgramGrid->uniformLocation("gridCellCount") != -1);
        //mShaderProgramGrid->setUniformValue("gridCellCount", mGridCells);
        glUniform3i(mShaderProgramGrid->uniformLocation("gridCellCount"), mGridInformationGainCellCount.x, mGridInformationGainCellCount.y, mGridInformationGainCellCount.z);

        glEnable(GL_BLEND);
        mVaoGridMapOfInformationGain->bind();
        glBindBuffer(GL_ARRAY_BUFFER, mVboGridMapOfInformationGain);
        glDrawArrays(GL_POINTS, 0, mGridInformationGainCellCount.x * mGridInformationGainCellCount.y * mGridInformationGainCellCount.z);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        mVaoGridMapOfInformationGain->release();
        glDisable(GL_BLEND);

        mShaderProgramGrid->release();
    }

    if(mNumberOfParticles != 0 && mVboParticlePositions != 0 && (mRenderParticles || mFlightPlannerProcessingState == FlightPlannerProcessingState::ParticleSimulation))
    {
        glEnable(GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Beau.Ti.Ful!
        // Program needs to be in use before setting values to uniforms
        mShaderProgramParticles->bind();
        mShaderProgramParticles->setUniformValue("useFixedColor", false);
        mShaderProgramParticles->setUniformValue("particleOpacity", std::max(mParticleOpacity-0.1f, 0.0f));

        mVaoParticles->bind();

        // Set particleRadius variable in the shader program
        Q_ASSERT(glGetUniformLocation(mShaderProgramParticles->programId(), "particleRadius") != -1);
        glUniform1f(glGetUniformLocation(mShaderProgramParticles->programId(), "particleRadius"), mParticleRadius);

        glBindBuffer(GL_ARRAY_BUFFER, mVboParticlePositions);
        glDrawArrays(GL_POINTS, 0, mNumberOfParticles);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        mVaoParticles->release();
        mShaderProgramParticles->release();
        glDisable(GL_BLEND);
    }

    // Render occupancy grid
    if(mVboGridMapOfOccupancy != 0 && mVaoGridMapOfOccupancy->isCreated() && (mRenderOccupancyGrid || mFlightPlannerProcessingState == FlightPlannerProcessingState::WayPointChecking || mFlightPlannerProcessingState == FlightPlannerProcessingState::WayPointComputation))
    {
        glEnable(GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Beau.Ti.Ful!
        // Draw grid with occupancy
        mShaderProgramGrid->bind();

        mShaderProgramGrid->setUniformValue("fixedColor", QColor(128,128,255,128));
        // If we have a value of (quint8)1, that'll be 1/255=0.004 in the shader's float. Amplify this?
        mShaderProgramGrid->setUniformValue("alphaMultiplication", 1.0f);
        mShaderProgramGrid->setUniformValue("alphaExponentiation", 1.0f);
        mShaderProgramGrid->setUniformValue("quadSizeFactor", 0.6f);

        // Set uniform values in the shader program
        Q_ASSERT(mShaderProgramGrid->uniformLocation("boundingBoxMin") != -1);
        mShaderProgramGrid->setUniformValue("boundingBoxMin", mBoundingBoxGridOccupancy.min);

        Q_ASSERT(mShaderProgramGrid->uniformLocation("boundingBoxMax") != -1);
        mShaderProgramGrid->setUniformValue("boundingBoxMax", mBoundingBoxGridOccupancy.max);

        // gridSize is a uint3, not sure how to set this with qt, so lets do opengl:
        Q_ASSERT(mShaderProgramGrid->uniformLocation("gridCellCount") != -1);
        glUniform3i(mShaderProgramGrid->uniformLocation("gridCellCount"), mGridOccupancyCellCount.x, mGridOccupancyCellCount.y, mGridOccupancyCellCount.z);

        mVaoGridMapOfOccupancy->bind();
        glBindBuffer(GL_ARRAY_BUFFER, mVboGridMapOfOccupancy);
        glDrawArrays(GL_POINTS, 0, mGridOccupancyCellCount.x * mGridOccupancyCellCount.y * mGridOccupancyCellCount.z);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        mVaoGridMapOfOccupancy->release();

        mShaderProgramGrid->release();
        glDisable(GL_BLEND);
    }

    // Render pathfinder grid
    if(mVboGridMapOfPathPlanner != 0 && mRenderPathPlannerGrid && mVaoGridMapOfPathPlanner->isCreated())
    {
        // Draw grid with waypoint pressure
        mShaderProgramGrid->bind();

        mShaderProgramGrid->setUniformValue("fixedColor", QColor(0,255,0));
        // If we have a value of (quint8)1, that'll be 1/255=0.004 in the shader's float. Amplify this?
        mShaderProgramGrid->setUniformValue("alphaMultiplication", 1.0f);
        mShaderProgramGrid->setUniformValue("alphaExponentiation", 2.0f);
        mShaderProgramGrid->setUniformValue("quadSizeFactor", 0.2f);

        // Set uniform values in the shader program
        Q_ASSERT(mShaderProgramGrid->uniformLocation("boundingBoxMin") != -1);
        mShaderProgramGrid->setUniformValue("boundingBoxMin", mBoundingBoxGridPathFinder.min);

        Q_ASSERT(mShaderProgramGrid->uniformLocation("boundingBoxMax") != -1);
        mShaderProgramGrid->setUniformValue("boundingBoxMax", mBoundingBoxGridPathFinder.max);

        // gridSize is a uint3, not sure how to set this with qt, so lets do opengl:
        Q_ASSERT(mShaderProgramGrid->uniformLocation("gridCellCount") != -1);
        glUniform3i(mShaderProgramGrid->uniformLocation("gridCellCount"), mGridPathFinderCellCount.x, mGridPathFinderCellCount.y, mGridPathFinderCellCount.z);

        // Make the contents of this array available at layout position vertexShaderVertexIndex in the vertex shader
        mVaoGridMapOfPathPlanner->bind();
        glBindBuffer(GL_ARRAY_BUFFER, mVboGridMapOfPathPlanner);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glDrawArrays(GL_POINTS, 0, mGridPathFinderCellCount.x * mGridPathFinderCellCount.y * mGridPathFinderCellCount.z);
        mVaoGridMapOfPathPlanner->release();

        mShaderProgramGrid->release();
    }
    glEnable(GL_CULL_FACE);

    glEnable(GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Beau.Ti.Ful!
    {
        if(mRenderWayPointsAhead && mVaoWayPointsAhead->isCreated() && !mWayPointsAhead->isEmpty())
        {
            const QColor c = mWayPointsAhead->color();
            mWayPointsAhead->updateVbo();
            mShaderProgramDefault->bind();
            mShaderProgramDefault->setUniformValue("useFixedColor", true);
            mShaderProgramDefault->setUniformValue("fixedColor", QVector4D(c.redF(),c.greenF(),c.blueF(),c.alphaF()));
            mVaoWayPointsAhead->bind();
//            glLineWidth(2.0f);
            glDrawArrays(GL_LINE_STRIP, 0, mWayPointsAhead->size());
            mShaderProgramDefault->release();

            mShaderProgramWaypoint->bind();
            mShaderProgramWaypoint->setUniformValue("activeWayPointIndex", mActiveWayPointVisualizationIndex);
            mShaderProgramWaypoint->setUniformValue("fixedColor", QVector4D(c.redF(),c.greenF(),c.blueF(),c.alphaF()));
            glDrawArrays(GL_POINTS, 0, mWayPointsAhead->size());
            mVaoWayPointsAhead->release();
            mShaderProgramWaypoint->release();
        }

        if(mRenderWayPointsPassed && mVaoWayPointsPassed->isCreated() && !mWayPointsPassed->isEmpty())
        {
            const QColor c = mWayPointsPassed->color();
            mWayPointsPassed->updateVbo();
            mShaderProgramDefault->bind();
            mShaderProgramDefault->setUniformValue("useFixedColor", true);
            mShaderProgramDefault->setUniformValue("fixedColor", QVector4D(c.redF(),c.greenF(),c.blueF(),c.alphaF()));
            mVaoWayPointsAhead->bind();
//            glLineWidth(2.0f);
            glDrawArrays(GL_LINE_STRIP, 0, mWayPointsPassed->size());
            mShaderProgramDefault->release();

            mShaderProgramWaypoint->bind();
            mShaderProgramWaypoint->setUniformValue("activeWayPointIndex", mActiveWayPointVisualizationIndex);
            mShaderProgramWaypoint->setUniformValue("fixedColor", QVector4D(c.redF(),c.greenF(),c.blueF(),c.alphaF()));
            glDrawArrays(GL_POINTS, 0, mWayPointsPassed->size());
            mVaoWayPointsAhead->release();
            mShaderProgramWaypoint->release();
        }
    }
    glDisable(GL_BLEND);
}

void GlScene::renderController(const QMatrix4x4& transform, const PidController* const controller)
{
    // Now render the controller-values
    QMatrix4x4 trControllerP(transform);
    const float sqrtWeightP = sqrt(controller->getWeightP());
    if(controller->getLastOutputP() < 0.0f)
    {
        // Make the bar point upwards, not downwards!
        trControllerP.rotate(180.0f, QVector3D(1,0,0));
    }
    else
    {
        // Make the bar appear on the other side of the kopter if pitch is positive!
        trControllerP.rotate(180.0f, QVector3D(0,1,0));
    }

    trControllerP.translate(0.8f + sqrtWeightP * 0.5f, 0.0f, 0.0f);

    trControllerP.scale(
                sqrtWeightP,
                controller->getLastOutputP(),
                sqrtWeightP);

    mModelControllerP->slotSetModelTransform(trControllerP);
    mModelControllerP->render();


    QMatrix4x4 trControllerI(transform);
    const float sqrtWeightI = sqrt(controller->getWeightI());
    if(controller->getLastOutputI() < 0.0f)
    {
        // Make the bar point upwards, not downwards!
        trControllerI.rotate(180.0f, QVector3D(1,0,0));
    }
    else
    {
        // Make the bar appear on the other side of the kopter if pitch is positive!
        trControllerI.rotate(180.0f, QVector3D(0,1,0));
    }

    trControllerI.translate(0.8f + sqrtWeightI * 0.5f, 0.0f, 0.0f);

    trControllerI.scale(
                sqrtWeightI,
                controller->getLastOutputI(),
                sqrtWeightI);

    mModelControllerI->slotSetModelTransform(trControllerI);
    mModelControllerI->render();


    QMatrix4x4 trControllerD(transform);
    const float sqrtWeightD = sqrt(controller->getWeightD());
    if(controller->getLastOutputD() < 0.0f)
    {
        // Make the bar point upwards, not downwards!
        trControllerD.rotate(180.0f, QVector3D(1,0,0));
    }
    else
    {
        // Make the bar appear on the other side of the kopter if pitch is positive!
        trControllerD.rotate(180.0f, QVector3D(0,1,0));
    }

    trControllerD.translate(0.8f + sqrtWeightD * 0.5f, 0.0f, 0.0f);

    trControllerD.scale(
                sqrtWeightD,
                controller->getLastOutputD(),
                sqrtWeightD);

    mModelControllerD->slotSetModelTransform(trControllerD);
    mModelControllerD->render();
}

void GlScene::slotNewVehiclePose(const Pose* const pose)
{
    if(mVboVehiclePathBytesCurrent + mVboVehiclePathElementSize < mVboVehiclePathBytesMaximum)
    {
        const QVector3D pos = pose->getPosition();

        // Precise and integrated poses are green, others are red.
        QColor color;
        if(
                pose->precision & Pose::RtkFixed &&
                pose->precision & Pose::AttitudeAvailable &&
                pose->precision & Pose::CorrectionAgeLow &&
                pose->precision & Pose::HeadingFixed &&
                pose->precision & Pose::ModeIntegrated
                )
            color.setRgb(255, 0, 0);
        else
            color.setRgb(0, 0, 200);

        // If the poses CV sucks, fade it.
        if(pose->covariances > Pose::maximumUsableCovariance) color.setAlpha(128);

        const float data[] = {
            (float)pos.x(),
            (float)pos.y(),
            (float)pos.z(),
            (float)color.redF(),
            (float)color.greenF(),
            (float)color.blueF(),
            (float)color.alphaF()};

        glBindBuffer(GL_ARRAY_BUFFER, mVboVehiclePath);

        glBufferSubData(
                    GL_ARRAY_BUFFER,
                    mVboVehiclePathBytesCurrent, // offset in the VBO
                    mVboVehiclePathElementSize, // how many bytes to store?
                    (void*)data // data to store
                    );

        mVboVehiclePathBytesCurrent += mVboVehiclePathElementSize;
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
    else
    {
        qDebug() << "GlWidget::slotNewVehiclePose(): VBO is full, discarding new pose.";
    }

    mLastKnownVehiclePose = pose;

    // We need to update that matrix, because the camera follows the vehicle.
    slotUpdateMatrixModelToCamera();
}

void GlScene::slotClearVehicleTrajectory()
{
    mVboVehiclePathBytesCurrent = 0;
}


void GlScene::slotPointCloudRegister(PointCloud* p)
{
    mPointCloudsToRender.append(p);
}

void GlScene::slotPointCloudUnregister(PointCloud* p)
{
    mPointCloudsToRender.removeOne(p);
}

bool GlScene::isPointCloudRegistered(PointCloud* p)
{
    return mPointCloudsToRender.contains(p);
}

void GlScene::slotSetFlightControllerValues(const FlightControllerValues* const fcv)
{
    mLastFlightControllerValues = fcv;
    slotNewVehiclePose(&fcv->lastKnownPose);
}

void GlScene::slotNewRawScan(const RawScan* const rawScan)
{
    if(mRenderRawScanRays)
    {
        RawScanRayVisualization* rsrv = nullptr;

        for(int i=0;i<mRawScanVisualizations.size();i++)
        {
            if(mRawScanVisualizations[i]->relativeScannerPose == *rawScan->relativeScannerPose)
            {
                rsrv = mRawScanVisualizations[i];
                break;
            }
        }

        if(rsrv == nullptr)
        {
            rsrv = new RawScanRayVisualization;
            rsrv->relativeScannerPose = *rawScan->relativeScannerPose;
            mRawScanVisualizations.append(rsrv);
        }

        rsrv->distanceIndexFirst = rawScan->firstUsableDistance;
        rsrv->distanceIndexLast = rawScan->firstUsableDistance + rawScan->numberOfDistances - 1;

        // If needed, create a VAO and VBO for the scan rays
        if(rsrv->vao == nullptr)
        {
            qDebug() << __PRETTY_FUNCTION__ << "creationg new visulization for scanner sitting at" << rawScan->relativeScannerPose;
            rsrv->vbo = OpenGlUtilities::createVbo();

            rsrv->vao = new QOpenGLVertexArrayObject(this);
            rsrv->vao->create();
            rsrv->vao->bind();
            quint32 rayStride = 0;
            glBindBuffer(GL_ARRAY_BUFFER, rsrv->vbo);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(
                        0,          // index
                        1,          // one element is one vertex attribute
                        GL_UNSIGNED_SHORT,   // quint16 is GL_UNSIGNED_SHORT
                        GL_TRUE,    // normalize to float [0-1]
                        (rayStride+1) * sizeof(quint16),          // bytes stride, not ray stride
                        0);         // no offset
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            rsrv->vao->release();
        }

        glBindBuffer(GL_ARRAY_BUFFER, rsrv->vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quint16) * rawScan->numberOfDistances, rawScan->distances, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
}

void GlScene::reloadShaders()
{
    mShaderProgramDefault->initialize();
    mShaderProgramGrid->initialize();
    mShaderProgramParticles->initialize();
    mShaderProgramPointCloud->initialize();
    mShaderProgramRawScanRays->initialize();
    mShaderProgramWaypoint->initialize();
    mShaderProgramSatelliteSignals->initialize();

    emit suggestVisualization();
}

void GlScene::slotSetVolumeGlobal(const Box3D* volume)
{
    qDebug() << __PRETTY_FUNCTION__ << *volume;
    mVolumeGlobal = volume;
    OpenGlUtilities::setVboToBoundingBox(mVboBoundingBoxVolumeGlobal, mVolumeGlobal);

    if(!mVaoBoundingBoxGlobal->isCreated()) mVaoBoundingBoxGlobal->create();

    mVaoBoundingBoxGlobal->bind();
    glBindBuffer(GL_ARRAY_BUFFER, mVboBoundingBoxVolumeGlobal);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0); // position
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    mVaoBoundingBoxGlobal->release();

    emit suggestVisualization();
}

void GlScene::slotSetVolumeLocal(const Box3D* volume)
{
    qDebug() << __PRETTY_FUNCTION__ << *volume;
    mVolumeLocal = volume;
    OpenGlUtilities::setVboToBoundingBox(mVboBoundingBoxVolumeLocal, mVolumeLocal);

    if(!mVaoBoundingBoxLocal->isCreated()) mVaoBoundingBoxLocal->create();

    mVaoBoundingBoxLocal->bind();
    glBindBuffer(GL_ARRAY_BUFFER, mVboBoundingBoxVolumeLocal);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0); // position
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    mVaoBoundingBoxLocal->release();

    emit suggestVisualization();
}

void GlScene::slotSetWayPointListAhead(WayPointList* wpl)
{
    mWayPointsAhead = wpl;

    if(!mVaoWayPointsAhead->isCreated()) mVaoWayPointsAhead->create();
    mVaoWayPointsAhead->bind();
    glBindBuffer(GL_ARRAY_BUFFER, mWayPointsAhead->vbo());
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 20, 0); // position
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 20, (void*)16);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    mVaoWayPointsAhead->release();

    emit suggestVisualization();
}

void GlScene::slotSetWayPointListPassed(WayPointList* wpl)
{
    mWayPointsPassed = wpl;

    if(!mVaoWayPointsPassed->isCreated()) mVaoWayPointsPassed->create();
    mVaoWayPointsPassed->bind();
    glBindBuffer(GL_ARRAY_BUFFER, mWayPointsPassed->vbo());
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 20, 0); // position
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 20, (void*)16);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    mVaoWayPointsPassed->release();

    emit suggestVisualization();
}

void GlScene::slotSetInsStatus(const GnssStatus* const g)
{
    // If needed, create a VAO and VBO for the scan rays
    if(!mVaoSatelliteSignals->isCreated())
    {
        mVaoSatelliteSignals->create();
        mVaoSatelliteSignals->bind();

        mVboSatelliteSignals = OpenGlUtilities::createVbo();

        glBindBuffer(GL_ARRAY_BUFFER, mVboSatelliteSignals);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(
                    0,          // index
                    4,          // one element is one vertex attribute
                    GL_UNSIGNED_BYTE,   // quint16 is GL_UNSIGNED_SHORT
                    GL_FALSE,    // normalize to float [0-1]
                    0,
                    0);         // no offset
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        mVaoSatelliteSignals->release();
    }

    quint8 sats[60*4*3] = {0};
    mNumberOfSatelliteSignals = 0;
    mNumberOfSatellitesGps = 0;
    mNumberOfSatellitesGlonass = 0;
    for(int sat=0;sat<g->receivedSatellites.size();sat++)
    {
        const GnssStatus::SatelliteReceptionStatus& srs = g->receivedSatellites.at(sat);

        if(srs.constellation == GnssConstellation::ConstellationSbas) continue;

        QMapIterator<GnssStatus::GnssSignalType,float> i(srs.carrierOverNoise);
        while(i.hasNext())
        {
            i.next();

            sats[4 * mNumberOfSatelliteSignals + 0] = static_cast<quint8>(srs.constellation);
            sats[4 * mNumberOfSatelliteSignals + 1] = srs.constellation == GnssConstellation::ConstellationGps ? mNumberOfSatellitesGps : mNumberOfSatellitesGlonass;
            sats[4 * mNumberOfSatelliteSignals + 2] = i.key();
            sats[4 * mNumberOfSatelliteSignals + 3] = (quint8)i.value() * 4;

            mNumberOfSatelliteSignals++;
        }

        if(srs.constellation == GnssConstellation::ConstellationGps)
            mNumberOfSatellitesGps++;
        else
            mNumberOfSatellitesGlonass++;
    }

    // sats now contains a list of signal-bytes that has to be drawn by shaders:
    // [constellation] [svid] [signal] [rssi] ...

    glBindBuffer(GL_ARRAY_BUFFER, mVboSatelliteSignals);
    glBufferData(GL_ARRAY_BUFFER,
                 mNumberOfSatelliteSignals * 4, // bytes should be enough
                 sats,
                 GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    emit suggestVisualization();
}
