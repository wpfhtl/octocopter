#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include "pointcloud.h"
#include "glwindow.h"
#include "cudahelper.h"

GlWindow::GlWindow(QWindow* parent) :
    QWindow(parent),
    mShaderProgramDefault(nullptr),
    mShaderProgramPointCloud(0),
    mShaderProgramRawScanRays(0),
    mLastFlightControllerValues(0),
    mLastKnownVehiclePose(0),
    mIsCurrentlyRendering(false)
{
    // Tell Qt we will use OpenGL for this window
    setSurfaceType(QWindow::OpenGLSurface);

    // Specify the format and create platform-specific surface
    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setMajorVersion(OPENGL_VERSION_MAJOR);
    format.setMinorVersion(OPENGL_VERSION_MINOR);
    format.setSamples(4);
    format.setProfile(QSurfaceFormat::CoreProfile);
    //format.setOption( QSurfaceFormat::DebugContext );
    setFormat(format);
    create();

    // Create an OpenGL context
    mOpenGlContext = new QOpenGLContext;
    mOpenGlContext->setFormat(format);
    mOpenGlContext->create();


    mVboVehiclePathElementSize = sizeof(QVector3D) + sizeof(QVector4D); // position and color with alpha
    mVboVehiclePathBytesMaximum = (3600 * 50 * mVboVehiclePathElementSize); // For a flight time of one hour
    mVboVehiclePathBytesCurrent = 0;

    mBackgroundBrightness = 0.2f;
    mMaxPointVisualizationDistance = 1000.0f;
    mPointCloudPointSize = 1.0;
    mPointCloudPointAlpha = 1.0;

    mPointCloudColorLow = 0.0f;
    mPointCloudColorHigh = 10.0f;

    mFramesRenderedThisSecond = 0;

    mVboRawScanRays = 0;

    mZoomFactorCurrent = 0.5;
    mZoomFactorTarget = 0.5;

    mCameraPosition = QVector3D(0.0f, 0.0f, 500.0f);
    // Rotate the camera to a good starting position
    mCameraRotation.setX(-30.0f);

    // Timed Animation
    mRotationPerFrame = 0.0f;
    mViewZooming = false;

    mRenderRawScanRays = true;
    mRenderAxisBase = true;
    mRenderAxisVehicle = true;
    mRenderTrajectory = true;
    mRenderVehicle = true;

    mTimerUpdate = new QTimer(this);
    mTimerUpdate->setInterval(1000 / 50);
    connect(mTimerUpdate, &QTimer::timeout, this, &GlWindow::slotRenderLater);
}

void GlWindow::slotInitialize()
{
    qDebug() << __PRETTY_FUNCTION__;
    mOpenGlContext->makeCurrent(this);

    if(!initializeOpenGLFunctions())
    {
        qDebug() << __PRETTY_FUNCTION__ << "couldn't initialize OpenGL 4.3 Core context, quitting.";
        exit(1);
    }

    OpenGlUtilities::init();

    // Create Vertex Array Object to contain our VBOs
    glGenVertexArrays(1, &mVertexArrayObject);
    // Bind the VAO to the context
    glBindVertexArray(mVertexArrayObject);

    CudaHelper::initializeCuda();

    // Create the uniform buffer object (UBO) for all members of the UBO-Block
    mUboSize = 64 + 64; // One Matrix4x4 has 16 floats with 4 bytes each, giving 64 bytes.
    glGenBuffers(1, &mUboId);
    glBindBuffer(GL_UNIFORM_BUFFER, mUboId);
    glBufferData(GL_UNIFORM_BUFFER, mUboSize, NULL, GL_STREAM_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
    // Now bind this UBO to a uniform-block binding-point
    glBindBufferRange(GL_UNIFORM_BUFFER, ShaderProgram::blockBindingPointGlobalMatrices, mUboId, 0, mUboSize);

    mShaderProgramDefault = new ShaderProgram(this, "shader-default-vertex.c", "", "shader-default-fragment.c");
    mShaderProgramPointCloud = new ShaderProgram(this, "shader-pointcloud-vertex.c", "", "shader-pointcloud-fragment.c");
    mShaderProgramRawScanRays = new ShaderProgram(this, "shader-rawscanrays-vertex.c", "shader-rawscanrays-geometry.c", "shader-default-fragment.c");

    // Create a VBO for the vehicle's path.
    glGenBuffers(1, &mVboVehiclePath);
    qDebug() << "GlWidget::initializeGL(): creating vehicle-path VBO" << mVboVehiclePath << "of size" << mVboVehiclePathBytesMaximum;
    glBindBuffer(GL_ARRAY_BUFFER, mVboVehiclePath);
    glBufferData(GL_ARRAY_BUFFER, mVboVehiclePathBytesMaximum, NULL, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

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

    // Create a VBO for the axes
    glGenBuffers(1, &mVboAxes);
    glBindBuffer(GL_ARRAY_BUFFER, mVboAxes);
    glBufferData(GL_ARRAY_BUFFER, sizeof(data), data, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

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

    glClearColor(mBackgroundBrightness, mBackgroundBrightness, mBackgroundBrightness, 0.0f);

    // Set Line Antialiasing
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

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

    emit initializingInGlContext();
}

void GlWindow::resize()
{
    mOpenGlContext->makeCurrent(this);

    int w = width();
    int h = height();

//    qDebug() << __PRETTY_FUNCTION__ << "size:" << w << h;

    // setup viewport, projection etc.
    glViewport(0, 0, w, h);

    // OpenGL 4 core profile: We set the second matrix (perspective = cameraToClip) in the UBO
    QMatrix4x4 matrixCameraToClip;
    //matrixCameraToClip.perspective(50.0f * mZoomFactorCurrent, (float)w/(float)h, 10.0f, +1000.0f);
    matrixCameraToClip.ortho(-w/2.0f * mZoomFactorCurrent, w/2.0f * mZoomFactorCurrent, -h/2.0f * mZoomFactorCurrent, h/2.0f * mZoomFactorCurrent, 1.0, 10000.0);

    //qDebug() << "GlWindow::resizeGL(): resizing gl viewport to" << w << h << "setting perspective/cameraclip matrix" << matrixCameraToClip;

    // Set the second matrix (cameraToClip) in the UBO
    glBindBuffer(GL_UNIFORM_BUFFER, mUboId);
    glBufferSubData(GL_UNIFORM_BUFFER, 64, 64, matrixCameraToClip.constData());
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}



void GlWindow::moveCamera(const QVector3D &pos)
{
    //    qDebug() << "moveCamera to " << pos;
    mCameraPosition = pos;
    //    slotEmitModelViewProjectionMatrix();
    slotRenderNow();
}

void GlWindow::slotRenderNow()
{
    if(!isExposed()) return;

        // Make the context current
    //    qDebug() << __PRETTY_FUNCTION__ << "making context current.";
    Q_ASSERT(mIsCurrentlyRendering == false);
    mIsCurrentlyRendering = true;
    mOpenGlContext->makeCurrent(this);

    mFramesRenderedThisSecond++;

    const QTime currentTime = QTime::currentTime();

    if(mTimeOfLastRender.second() != currentTime.second())
    {
        // A second has passed!
//        qDebug() << "GlWidget::slotRenderNow(): currently rendering at" << mFramesRenderedThisSecond << "fps.";
        mFramesRenderedThisSecond = 0;
    }

    mTimeOfLastRender = currentTime;

    if(fabs(mRotationPerFrame) > 0.00001)
        mCameraRotation.setY(mCameraRotation.y() + mRotationPerFrame);

    // Clear color buffer and depth buffer
    glClearColor(mBackgroundBrightness, mBackgroundBrightness, mBackgroundBrightness, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const QVector3D camLookAt = mCamLookAtOffset + (mLastKnownVehiclePose ? mLastKnownVehiclePose->getPosition() : QVector3D());

    QQuaternion cameraRotation =
            QQuaternion::fromAxisAndAngle(QVector3D(0.0f, 1.0f, 0.0f), mCameraRotation.y())
            * QQuaternion::fromAxisAndAngle(QVector3D(1.0f, 0.0f, 0.0f), mCameraRotation.x());

    const QVector3D camPos = cameraRotation.rotatedVector(mCameraPosition);

    // Write the modelToCamera matrix into our UBO
    QMatrix4x4 matrixModelToCamera;
    matrixModelToCamera.lookAt(camPos, camLookAt, QVector3D(0.0f, 1.0f, 0.0f));
    glBindBuffer(GL_UNIFORM_BUFFER, mUboId);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, 64, matrixModelToCamera.constData());
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
    //qDebug() << "GlWidget::slotRenderNow(): camPos" << camPos << "lookAt" << camLookAt << "modelToCamera:" << matrixModelToCamera << matrixModelToCamera.inverted() * QVector3D();

    // Here we make mZoomFactorCurrent converge to mZoomFactorTarget for smooth zooming
    float step = 0.0f;
    if(mZoomFactorTarget > (mZoomFactorCurrent + 0.0001f))
        step = (mZoomFactorTarget - mZoomFactorCurrent) / 10.0f;
    else if((mZoomFactorTarget + 0.0001f) < mZoomFactorCurrent)
        step = -(mZoomFactorCurrent - mZoomFactorTarget) / 10.0f;

    if(mViewZooming && fabs(step) > 0.00001f)
    {
        mZoomFactorCurrent += step;
        mViewZooming = true;
        resize(); // this sets up a new view with the new zoomFactor
    }
    else
    {
        // Stop zooming, lower framerate
        mTimerUpdate->setInterval(1000 / 50);
        mViewZooming = false;
    }

    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Beau.Ti.Ful!
    {
        mShaderProgramPointCloud->bind();
        mShaderProgramPointCloud->setUniformValue("maxPointVisualizationDistance", (GLfloat)pow(mMaxPointVisualizationDistance, 2.0)); // distances are squared in the point cloud, too!
        mShaderProgramPointCloud->setUniformValue("pointcloudPointAlpha", mPointCloudPointAlpha);
        mShaderProgramPointCloud->setUniformValue("pointcloudColorLow", mPointCloudColorLow);
        mShaderProgramPointCloud->setUniformValue("pointcloudColorHigh", mPointCloudColorHigh);
        for(int i=0;i<mPointCloudsToRender.size();i++)
        {
            const QVector<PointCloud::VboInfo>& vboInfoList = mPointCloudsToRender.at(i)->getVboInfo();
            for(int j=0;j<vboInfoList.size();j++)
            {
                const PointCloud::VboInfo& vboInfo = vboInfoList.at(j);

                if(vboInfo.size == 0) continue;

                // If the pointcloud has a color set, use it. Otherwise, use the jet colormap.
                if(vboInfo.color.isValid())
                {
                    mShaderProgramPointCloud->setUniformValue("useFixedColor", true);
                    mShaderProgramPointCloud->setUniformValue("fixedColor", vboInfo.color);
                }
                else
                {
                    mShaderProgramPointCloud->setUniformValue("useFixedColor", false);
                }

                glBindBuffer(GL_ARRAY_BUFFER, vboInfo.vbo);
                glEnableVertexAttribArray(0);
                glVertexAttribPointer(0, vboInfo.elementSize, GL_FLOAT, GL_FALSE, vboInfo.stride, 0);
                glPointSize(mPointCloudPointSize);
                glDrawArrays(GL_POINTS, 0, vboInfo.size); // Number of elements, not bytes
                glDisableVertexAttribArray(0);
                glBindBuffer(GL_ARRAY_BUFFER, 0);
            }
        }
        mShaderProgramPointCloud->release();

        if(mRenderRawScanRays && mVboRawScanRays)
        {
            quint8 rayStride = 0; // how many rays to ignore between visualized rays
            mShaderProgramRawScanRays->bind();
            mShaderProgramRawScanRays->setUniformValue("rayStride", rayStride);

            QMatrix4x4 lidarMatrix;
            if(mLastKnownVehiclePose) lidarMatrix = mLastKnownVehiclePose->getMatrixCopy();
            //lidarMatrix.translate(QVector3D(0.0f, -0.04f, -0.14f));

            mShaderProgramRawScanRays->setUniformValue("useFixedColor", false);
            mShaderProgramRawScanRays->setUniformValue("useMatrixExtra", true);
            mShaderProgramRawScanRays->setUniformValue("firstUsableDistance", mRayVisualizationUsableDistanceIndexFirst);
            mShaderProgramRawScanRays->setUniformValue("matrixExtra", lidarMatrix * mRayVisualizationRelativeScannerMatrix);

            glBindBuffer(GL_ARRAY_BUFFER, mVboRawScanRays);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(
                        0,          // index
                        1,          // one element is one vertex attribute
                        GL_UNSIGNED_SHORT,   // quint16 is GL_UNSIGNED_SHORT
                        GL_TRUE,    // normalize to float [0-1]
                        (rayStride+1) * sizeof(quint16),          // bytes stride, not ray stride
                        0);         // no offset
            // There's 1081 rays from hokuyo, but we get only from first to last usable rays delivered. We could reconstruct the other
            // one, but it doesn't look good and is expensive.
            glDrawArrays(GL_POINTS, 0, (mRayVisualizationUsableDistanceIndexLast-mRayVisualizationUsableDistanceIndexFirst) / (rayStride+1));
            glDisableVertexAttribArray(0);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            mShaderProgramRawScanRays->release();
        }

        mShaderProgramDefault->bind();
        {
            if(mRenderTrajectory && mVboVehiclePathBytesCurrent > 0)
            {
                // Render the vehicle's path - same shader, but variable color
                mShaderProgramDefault->setUniformValue("useFixedColor", false);
                glBindBuffer(GL_ARRAY_BUFFER, mVboVehiclePath);
                glEnableVertexAttribArray(0);
                glEnableVertexAttribArray(1);
                // Stride is NOT the number of useless bytes between two packets, its the
                // "distance" between two beginnings of two consecutive useful packets
                glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 28, 0); // position.
                glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 28, (void*)12); // color
                glPointSize(1.0);
                glDrawArrays(GL_POINTS, 0, (mVboVehiclePathBytesCurrent / mVboVehiclePathElementSize));
                glDisableVertexAttribArray(0);
                glDisableVertexAttribArray(1);
                glBindBuffer(GL_ARRAY_BUFFER, 0);
            }

            // Prepare axis rendering if desired
            if(mRenderAxisBase || mRenderAxisVehicle)
            {
                mShaderProgramDefault->setUniformValue("useFixedColor", false);
                glBindBuffer(GL_ARRAY_BUFFER, mVboAxes);
                glEnableVertexAttribArray(0);
                glEnableVertexAttribArray(1);
                glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0); // positions
                glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, (void*)(12 * sizeof(float) * 4)); // colors
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
                glDisableVertexAttribArray(0);
                glDisableVertexAttribArray(1);
                glBindBuffer(GL_ARRAY_BUFFER, 0);
            }
        }
        mShaderProgramDefault->release();
    }
    glDisable(GL_BLEND);

    emit visualizeNow();

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

    // Render velocities - preferrably from the flightcontroller.
    const Pose* p;
    if(mLastFlightControllerValues && mLastKnownVehiclePose->timestamp - mLastFlightControllerValues->lastKnownPose.timestamp < 500)
        p = &mLastFlightControllerValues->lastKnownPose;
    else
        p = mLastKnownVehiclePose;

    // tetsing, only use fc values
    p = &mLastFlightControllerValues->lastKnownPose;

    if(mLastFlightControllerValues/*p != 0*/)
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

    mOpenGlContext->swapBuffers(this);
    mIsCurrentlyRendering = false;
}

void GlWindow::renderController(const QMatrix4x4& transform, const PidController* const controller)
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

void GlWindow::slotNewVehiclePose(const Pose* const pose)
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

    slotRenderLater();
}

void GlWindow::slotClearVehicleTrajectory()
{
    mVboVehiclePathBytesCurrent = 0;
}

void GlWindow::slotSetCameraRotation(const float rotation)
{
    mRotationPerFrame = rotation;

    // Enable the timer if we want to rotate and its not running already
    if(fabs(mRotationPerFrame) > 0.00001 && !mViewZooming)
        mTimerUpdate->start();
}

//void GlWindow::slotEnableTimerRotation(const bool& enable){}

/*void GlWindow::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_Q)
    {
        slotSetCameraRotation(mRotationPerFrame - 0.0005f);
    }
    else if(event->key() == Qt::Key_W)
    {
        slotSetCameraRotation(mRotationPerFrame + 0.0005f);
    }
    else if(event->key() == Qt::Key_V)
    {
        mRenderVehicle = !mRenderVehicle;
        emit message(LogImportance::Information, "GlWidget::keyPressEvent()", "toggling background brightness");
        slotRenderLater();
    }
    else if(event->key() == Qt::Key_T)
    {
        mRenderTrajectory = !mRenderTrajectory;
        emit message(LogImportance::Information, "GlWidget::keyPressEvent()", "toggling visualization of vehicle trajectory");
        slotRenderLater();
    }
    else if(event->key() == Qt::Key_A && (event->modifiers() & Qt::ShiftModifier))
    {
        mRenderAxisBase = !mRenderAxisBase;
        emit message(LogImportance::Information, "GlWidget::keyPressEvent()", "toggling visualization of coordinate system at base");
        slotRenderLater();
    }
    else if(event->key() == Qt::Key_A && !(event->modifiers() & Qt::ShiftModifier))
    {
        mRenderAxisVehicle = !mRenderAxisVehicle;
        emit message(LogImportance::Information, "GlWidget::keyPressEvent()", "toggling visualization of coordinate system at vehicle");
        slotRenderLater();
    }
    else if(event->key() == Qt::Key_Minus)
    {
        mMaxPointVisualizationDistance = qBound(0.0f, mMaxPointVisualizationDistance - 0.5f, 30.0f);
        emit message(LogImportance::Information, "GlWidget::keyPressEvent()", QString("setting max point visualization distance to %1").arg(mMaxPointVisualizationDistance));
        slotRenderLater();
    }
    else if(event->key() == Qt::Key_Plus)
    {
        mMaxPointVisualizationDistance = qBound(0.0f, mMaxPointVisualizationDistance + 0.5f, 30.0f);
        emit message(LogImportance::Information, "GlWidget::keyPressEvent()", QString("setting max point visualization distance to %1").arg(mMaxPointVisualizationDistance));
        slotRenderLater();
    }
    else if(event->key() == Qt::Key_S)
    {
        mRenderRawScanRays = !mRenderRawScanRays;
        emit message(LogImportance::Information, "GlWidget::keyPressEvent()", QString("toggling visualization of raw scan rays"));
        slotRenderLater();
    }
    else
        QWindow::keyPressEvent(event);
}
*/

void GlWindow::mousePressEvent(QMouseEvent *event)
{
    mLastMousePosition = event->pos();
}

void GlWindow::mouseMoveEvent(QMouseEvent *event)
{
    const float deltaX = -float(event->x()-mLastMousePosition.x())/width();
    const float deltaY = -float(event->y()-mLastMousePosition.y())/height();

    if(event->buttons() & Qt::LeftButton)
    {
        mCameraRotation.setX(qBound(-89.9f, float(mCameraRotation.x() + 180.0f * deltaY), 89.9f));
        mCameraRotation.setY(fmod(mCameraRotation.y() + 180 * deltaX, 360.0f));
    }
    else if(event->buttons() & Qt::MiddleButton)
    {
        mCamLookAtOffset.setZ(mCamLookAtOffset.z() + 180.0f * deltaY);
        mCamLookAtOffset.setX(mCamLookAtOffset.x() + 180.0f * deltaX);
    }

    mLastMousePosition = event->pos();

    //qDebug() << "mCamLookAtOffset: " << mCamLookAtOffset << "rotXYZ:" << rotX << rotY << rotZ;

    // update();
    slotRenderLater();
}

void GlWindow::wheelEvent(QWheelEvent *event)
{
    event->delta() > 0 ? mZoomFactorTarget *= 1.5f : mZoomFactorTarget *= 0.5f;
    mZoomFactorTarget = qBound(0.002f, (float)mZoomFactorTarget, 1.0f);
    mViewZooming = true;
    mTimerUpdate->setInterval(1000 / 50);
    mTimerUpdate->start();
    slotRenderLater();
}


void GlWindow::slotRenderLater()
{
    // quick hack to see ALL generated poses
    // update(); return;

    if(mTimeOfLastRender.msecsTo(QTime::currentTime()) > mTimerUpdate->interval())
    {
        slotRenderNow();

        if(!fabs(mRotationPerFrame) > 0.00001 && !mViewZooming)
            mTimerUpdate->stop();
    }
    else
    {
        if(!mTimerUpdate->isActive())
            mTimerUpdate->start();
    }

//    QTimer::singleShot(desiredInterval + 1 - mTimeOfLastRender.msecsTo(QTime::currentTime()), this, SLOT(slotUpdateView()));
}

void GlWindow::exposeEvent(QExposeEvent *event)
{
//    qDebug() << __PRETTY_FUNCTION__;
    Q_UNUSED(event);

    if(isExposed()) slotRenderNow();
}

void GlWindow::resizeEvent(QResizeEvent *event)
{
//    qDebug() << __PRETTY_FUNCTION__;
    Q_UNUSED(event);

    resize();

    if(isExposed()) slotRenderNow();
}

void GlWindow::slotViewFromTop()
{
    mCameraRotation = QVector2D(50, -17.71);
    mZoomFactorCurrent = 0.6;
    mZoomFactorTarget = 0.6;
    slotRenderNow();
}

void GlWindow::slotViewFromSide()
{
    mCameraRotation = QVector2D(-23, -3.5);

    mZoomFactorCurrent = 0.6;
    mZoomFactorTarget = 0.6;
    slotRenderNow();
}

void GlWindow::slotPointCloudRegister(PointCloud* p)
{
    mPointCloudsToRender.append(p);
}

void GlWindow::slotPointCloudUnregister(PointCloud* p)
{
    mPointCloudsToRender.removeOne(p);
}

bool GlWindow::isPointCloudRegistered(PointCloud* p)
{
    return mPointCloudsToRender.contains(p);
}

void GlWindow::slotSetFlightControllerValues(const FlightControllerValues* const fcv)
{
    mLastFlightControllerValues = fcv;
    slotNewVehiclePose(&fcv->lastKnownPose);
}

void GlWindow::slotNewRawScan(const RawScan* const rawScan)
{
    if(mRenderRawScanRays)
    {
        mRayVisualizationRelativeScannerMatrix = *(rawScan->relativeScannerPose);
        mRayVisualizationUsableDistanceIndexFirst = rawScan->firstUsableDistance;
        mRayVisualizationUsableDistanceIndexLast = rawScan->firstUsableDistance + rawScan->numberOfDistances - 1;

        // If needed, create a VBO for the scan rays
        if(!mVboRawScanRays) glGenBuffers(1, &mVboRawScanRays);

        glBindBuffer(GL_ARRAY_BUFFER, mVboRawScanRays);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quint16) * rawScan->numberOfDistances, rawScan->distances, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        slotRenderLater();
    }
}

void GlWindow::reloadShaders()
{
    mShaderProgramDefault->initialize();
    mShaderProgramPointCloud->initialize();
    mShaderProgramRawScanRays->initialize();
}
