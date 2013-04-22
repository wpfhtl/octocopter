#include <GL/glew.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include "pointcloud.h"
#include "glwidget.h"
#include "cudahelper.h"

GlWidget::GlWidget(QWidget* parent) :
    QGLWidget(parent),
    mLastFlightControllerValues(0),
    mLastKnownVehiclePose(0)
{
    QGLFormat glFormat;
    glFormat.setSamples(4);
    glFormat.setSampleBuffers(true);
    glFormat.setVersion(4,0);
    // This means no OpenGL-deprecated stuff is used (like glBegin() and glEnd())
    //    glFormat.setProfile(QGLFormat::CoreProfile);
    glFormat.setProfile(QGLFormat::CompatibilityProfile);
    QGLFormat::setDefaultFormat(glFormat);
    setFormat(glFormat);

    mVboVehiclePathElementSize = sizeof(QVector3D) + sizeof(QVector4D); // position and color with alpha
    mVboVehiclePathBytesMaximum = (3600 * 50 * mVboVehiclePathElementSize); // For a flight time of one hour
    mVboVehiclePathBytesCurrent = 0;

    mMaxPointVisualizationDistance = 1000.0f;
    mBackgroundDarkOrBright = true;

    mFramesRenderedThisSecond = 0;

    mVboRawScanRays = 0;

    mZoomFactorCurrent = 0.5;
    mZoomFactorTarget = 0.5;

    mCameraPosition = QVector3D(0.0f, 0.0f, 500.0f);
    // Rotate the camera to a good starting position
    mCameraRotation.setX(-30.0f);

    // Timed Animation
    mRotationPerFrame = 0.001f;
    mViewRotating = false;
    mViewZooming = false;

    mRenderRawScanRays = false;
    mRenderAxisBase = true;
    mRenderAxisVehicle = true;
    mRenderTrajectory = true;
    mRenderVehicle = true;

    mTimerUpdate = new QTimer(this);
    mTimerUpdate->setInterval(1000 / 60);
    connect(mTimerUpdate, SIGNAL(timeout()), SLOT(slotUpdateView()));

    setMinimumSize(640, 480);
    setFocusPolicy(Qt::ClickFocus);
}

void GlWidget::initializeGL()
{
    glewExperimental = GL_TRUE; // needed for core profile... :|
    glewInit();

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

    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);					// Black Background
    glClearColor(1.0f, 1.0f, 1.0f, 0.0f);					// White Background
//    glClearColor(0.2f, 0.2f, 0.2f, 0.0f);					// Gray  Background

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

    mModelControllerP = new Model(QFile(modelPath.absolutePath() + "/media/controller-p.obj"), QString("../media/"), this);
    mModelControllerI = new Model(QFile(modelPath.absolutePath() + "/media/controller-i.obj"), QString("../media/"), this);
    mModelControllerD = new Model(QFile(modelPath.absolutePath() + "/media/controller-d.obj"), QString("../media/"), this);

    emit initializingInGlContext();
}

void GlWidget::resizeGL(int w, int h)
{
    // setup viewport, projection etc.
    glViewport(0, 0, w, h);

    // OpenGL 4 core profile: We set the second matrix (perspective = cameraToClip) in the UBO
    QMatrix4x4 matrixCameraToClip;
    //matrixCameraToClip.perspective(50.0f * mZoomFactorCurrent, (float)w/(float)h, 10.0f, +1000.0f);
    matrixCameraToClip.ortho(-w/2.0f * mZoomFactorCurrent, w/2.0f * mZoomFactorCurrent, -h/2.0f * mZoomFactorCurrent, h/2.0f * mZoomFactorCurrent, 1.0, 10000.0);

    //    qDebug() << "GlWidget::resizeGL(): resizing gl viewport to" << w << h << "setting perspective/cameraclip matrix" << matrixCameraToClip;

    // Set the second matrix (cameraToClip) in the UBO
    glBindBuffer(GL_UNIFORM_BUFFER, mUboId);
    glBufferSubData(GL_UNIFORM_BUFFER, 64, 64, OpenGlUtilities::matrixToOpenGl(matrixCameraToClip).constData());
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

void GlWidget::moveCamera(const QVector3D &pos)
{
    //    qDebug() << "moveCamera to " << pos;
    mCameraPosition = pos;
    //    slotEmitModelViewProjectionMatrix();
    update();
}

void GlWidget::paintGL()
{
    mFramesRenderedThisSecond++;

    const QTime currentTime = QTime::currentTime();

    if(mTimeOfLastRender.second() != currentTime.second())
    {
        // A second has passed!
//        qDebug() << "GlWidget::paintGL(): currently rendering at" << mFramesRenderedThisSecond << "fps.";
        mFramesRenderedThisSecond = 0;
    }

    mTimeOfLastRender = currentTime;

    if(mViewRotating)
        mCameraRotation.setY(mCameraRotation.y() + 180 * mRotationPerFrame);

    // Clear color buffer and depth buffer
    if(mBackgroundDarkOrBright)
        glClearColor(0.2f, 0.2f, 0.2f, 0.0f);					// Dark Background
    else
        glClearColor(1.0f, 1.0f, 1.0f, 0.0f);					// White Background

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
    glBufferSubData(GL_UNIFORM_BUFFER, 0, 64, OpenGlUtilities::matrixToOpenGl(matrixModelToCamera).constData());
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
    //    qDebug() << "GlWidget::paintGL(): camPos" << camPos << "lookAt" << camLookAt << "modelToCamera:" << matrixModelToCamera << matrixModelToCamera.inverted() * QVector3D();

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
        resizeGL(width(),height()); // this sets up a new view with the new zoomFactor
    }
    else
    {
        // Stop zooming, lower framerate
        mTimerUpdate->setInterval(1000 / 60);
        mViewZooming = false;
    }

    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Beau.Ti.Ful!
    {
        mShaderProgramPointCloud->bind();
        mShaderProgramPointCloud->setUniformValue("maxPointVisualizationDistance", (GLfloat)pow(mMaxPointVisualizationDistance, 2.0)); // distances are squared in the point cloud, too!
        for(int i=0;i<mPointCloudsToRender.size();i++)
        {
            const QVector<PointCloud::VboInfo>& vboInfoList = mPointCloudsToRender.at(i)->getVboInfo();
            for(int j=0;j<vboInfoList.size();j++)
            {
                const PointCloud::VboInfo& vboInfo = vboInfoList.at(j);

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
                glDrawArrays(GL_POINTS, 0, vboInfo.size); // Number of elements, not bytes
                glDisableVertexAttribArray(0);
                glBindBuffer(GL_ARRAY_BUFFER, 0);
            }
        }
        mShaderProgramPointCloud->release();

        if(mRenderRawScanRays && mVboRawScanRays)
        {
            quint8 rayStride = 1; // how many rays to ignore between visualized rays
            mShaderProgramRawScanRays->bind();
            mShaderProgramRawScanRays->setUniformValue("rayStride", rayStride);

            QMatrix4x4 lidarMatrix;
            if(mLastKnownVehiclePose) lidarMatrix = mLastKnownVehiclePose->getMatrixCopy();
            lidarMatrix.translate(QVector3D(0.0f, -0.04f, -0.14f));

            mShaderProgramRawScanRays->setUniformValue("useFixedColor", false);
            mShaderProgramRawScanRays->setUniformValue("useMatrixExtra", true);
            mShaderProgramRawScanRays->setUniformValue("matrixExtra", lidarMatrix);

            glBindBuffer(GL_ARRAY_BUFFER, mVboRawScanRays);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(
                        0,          // index
                        1,          // one element is one vertex attribute
                        GL_UNSIGNED_SHORT,   // quint16 is GL_UNSIGNED_SHORT
                        GL_TRUE,    // normalize to float [0-1]
                        (rayStride+1) * sizeof(quint16),          // bytes stride, not ray stride
                        0);         // no offset
            glDrawArrays(GL_POINTS, 0, 1080 / (rayStride+1)); // 1080 rays from hokuyo...
            glDisableVertexAttribArray(0);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            mShaderProgramRawScanRays->release();
        }

        mShaderProgramDefault->bind();
        {
            if(mRenderTrajectory)
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
    if(mLastFlightControllerValues/* && mLastKnownVehiclePose->timestamp - mLastFlightControllerValues->lastKnownPose.timestamp < 500*/)
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

        if(mLastFlightControllerValues->motionCommand.thrust != mLastFlightControllerValues->motionCommand.thrustHover && mLastFlightControllerValues->motionCommand.thrust > 0)
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

    if(mRenderVehicle)
    {
        mModelVehicle->slotSetModelTransform(transformVehicle);
        mModelVehicle->render();
    }
}

void GlWidget::renderController(const QMatrix4x4& transform, const PidController* const controller)
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

void GlWidget::slotNewVehiclePose(const Pose* const pose)
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
            color.setRgb(0, 255, 0);
        else
            color.setRgb(255, 0, 0);

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

    slotUpdateView();
}

void GlWidget::slotClearVehicleTrajectory()
{
    mVboVehiclePathBytesCurrent = 0;
}

void GlWidget::slotEnableTimerRotation(const bool& enable)
{
    if(mViewRotating == enable) return;

    mViewRotating = enable;

    // Enable the timer if we want to rotate and its not running already
    if(mViewRotating && !mViewZooming)
        mTimerUpdate->start();

    emit rotating(enable);
}
void GlWidget::mousePressEvent(QMouseEvent *event)
{
    mLastMousePosition = event->pos();
}

void GlWidget::mouseMoveEvent(QMouseEvent *event)
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
    slotUpdateView();
}

void GlWidget::wheelEvent(QWheelEvent *event)
{
    event->delta() > 0 ? mZoomFactorTarget *= 1.5f : mZoomFactorTarget *= 0.5f;
    mZoomFactorTarget = qBound(0.002f, (float)mZoomFactorTarget, 1.0f);
    mViewZooming = true;
    mTimerUpdate->setInterval(1000 / 60);
    mTimerUpdate->start();
    slotUpdateView();
}

void GlWidget::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_Q)
    {
        mRotationPerFrame -= 0.0005f;
        slotEnableTimerRotation(true);
    }
    else if(event->key() == Qt::Key_W)
    {
        mRotationPerFrame += 0.0005f;
        slotEnableTimerRotation(true);
    }
    else if(event->key() == Qt::Key_L)
    {
        mBackgroundDarkOrBright = !mBackgroundDarkOrBright;
        emit message(LogImportance::Information, "GlWidget::keyPressEvent()", "toggling background brightness");
        slotUpdateView();
    }
    else if(event->key() == Qt::Key_V)
    {
        mRenderVehicle = !mRenderVehicle;
        emit message(LogImportance::Information, "GlWidget::keyPressEvent()", "toggling background brightness");
        slotUpdateView();
    }
    else if(event->key() == Qt::Key_T)
    {
        mRenderTrajectory = !mRenderTrajectory;
        emit message(LogImportance::Information, "GlWidget::keyPressEvent()", "toggling visualization of vehicle trajectory");
        slotUpdateView();
    }
    else if(event->key() == Qt::Key_A && (event->modifiers() & Qt::ShiftModifier))
    {
        mRenderAxisBase = !mRenderAxisBase;
        emit message(LogImportance::Information, "GlWidget::keyPressEvent()", "toggling visualization of coordinate system at base");
        slotUpdateView();
    }
    else if(event->key() == Qt::Key_A && !(event->modifiers() & Qt::ShiftModifier))
    {
        mRenderAxisVehicle = !mRenderAxisVehicle;
        emit message(LogImportance::Information, "GlWidget::keyPressEvent()", "toggling visualization of coordinate system at vehicle");
        slotUpdateView();
    }
    else if(event->key() == Qt::Key_Minus)
    {
        mMaxPointVisualizationDistance = qBound(0.0f, mMaxPointVisualizationDistance - 0.5f, 30.0f);
        emit message(LogImportance::Information, "GlWidget::keyPressEvent()", QString("setting max point visualization distance to %1").arg(mMaxPointVisualizationDistance));
        slotUpdateView();
    }
    else if(event->key() == Qt::Key_Plus)
    {
        mMaxPointVisualizationDistance = qBound(0.0f, mMaxPointVisualizationDistance + 0.5f, 30.0f);
        emit message(LogImportance::Information, "GlWidget::keyPressEvent()", QString("setting max point visualization distance to %1").arg(mMaxPointVisualizationDistance));
        slotUpdateView();
    }
    else if(event->key() == Qt::Key_S)
    {
        mRenderRawScanRays = !mRenderRawScanRays;
        emit message(LogImportance::Information, "GlWidget::keyPressEvent()", QString("toggling visualization of raw scan rays"));
        slotUpdateView();
    }
    else
        QGLWidget::keyPressEvent(event);
}

void GlWidget::slotUpdateView()
{
    // quick hack to see ALL generated poses
    // update(); return;

    if(mTimeOfLastRender.msecsTo(QTime::currentTime()) > mTimerUpdate->interval())
    {
        update();

        if(!mViewRotating && !mViewZooming)
            mTimerUpdate->stop();
    }
    else
    {
        if(!mTimerUpdate->isActive())
            mTimerUpdate->start();
    }

//    QTimer::singleShot(desiredInterval + 1 - mTimeOfLastRender.msecsTo(QTime::currentTime()), this, SLOT(slotUpdateView()));
}

void GlWidget::slotViewFromTop()
{
    mCameraRotation = QVector2D(50, -17.71);
    mZoomFactorCurrent = 0.6;
    mZoomFactorTarget = 0.6;
    updateGL();
}

void GlWidget::slotViewFromSide()
{
    mCameraRotation = QVector2D(-23, -3.5);

    mZoomFactorCurrent = 0.6;
    mZoomFactorTarget = 0.6;
    updateGL();
}

void GlWidget::slotSaveImage()
{
    renderPixmap(0, 0, true).save(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz").prepend("snapshot-").append(".png"));
}

void GlWidget::slotPointCloudRegister(PointCloud* p)
{
    mPointCloudsToRender.append(p);
}

void GlWidget::slotPointCloudUnregister(PointCloud* p)
{
    mPointCloudsToRender.removeOne(p);
}

bool GlWidget::isPointCloudRegistered(PointCloud* p)
{
    return mPointCloudsToRender.contains(p);
}

void GlWidget::slotSetFlightControllerValues(const FlightControllerValues* const fcv)
{
    mLastFlightControllerValues = fcv;
    mLastKnownVehiclePose = &fcv->lastKnownPose;
    slotUpdateView();
}

void GlWidget::slotNewScanData(const qint32& timestampScanScanner, std::vector<quint16> * const distances)
{
    if(mRenderRawScanRays)
    {
        // If needed, create a VBO for the scan rays
        if(!mVboRawScanRays) glGenBuffers(1, &mVboRawScanRays);

//        std::reverse(distances->begin(), distances->end());
        glBindBuffer(GL_ARRAY_BUFFER, mVboRawScanRays);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quint16) * distances->size(), distances->data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
//        std::reverse(distances->begin(), distances->end());

        slotUpdateView();
    }
}
