#include <GL/glew.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include "pointcloud.h"
#include "glwidget.h"

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

    mCameraPosition = QVector3D(0.0f, 500.0f, 500.0f);

    mVboVehiclePathElementSize = sizeof(QVector3D) + sizeof(QVector4D); // position and color with alpha
    mVboVehiclePathBytesMaximum = (3600 * 50 * mVboVehiclePathElementSize); // For a flight time of one hour
    mVboVehiclePathBytesCurrent = 0;

    mZoomFactorCurrent = 0.5;
    mZoomFactorTarget = 0.5;

    // Mouse Move Rotations
    rotX = rotY = rotZ = 0.0f;

    // Timed Animation
    mViewRotating = false;
    mViewZooming = false;

    mTimerUpdate = new QTimer(this);
    mTimerUpdate->setInterval(1000 / 30);
    connect(mTimerUpdate, SIGNAL(timeout()), SLOT(slotUpdateView()));

    setMinimumSize(320, 240);
}

void GlWidget::initializeGL()
{
    glewExperimental = GL_TRUE; // needed for core profile... :|
    glewInit();

    // Create Vertex Array Object to contain our VBOs
    glGenVertexArrays(1, &mVertexArrayObject);
    // Bind the VAO to the context
    glBindVertexArray(mVertexArrayObject);

    // Initialize CUDA
    int numberOfCudaDevices;
    cudaGetDeviceCount(&numberOfCudaDevices);
    Q_ASSERT(numberOfCudaDevices && "FlightPlannerParticles::slotInitialize(): No CUDA devices found, exiting.");

    cudaError_t cudaError;

    int activeCudaDevice;
    cudaError = cudaGetDevice(&activeCudaDevice);
    if(cudaError != cudaSuccess) qFatal("FlightPlannerParticles::slotInitialize(): couldn't get device: code %d: %s, exiting.", cudaError, cudaGetErrorString(cudaError));

    // Necessary for OpenGL graphics interop: GlWidget and CUDA-based FlightPlanners have a close relationship because cudaGlSetGlDevice() needs to be called in GL context and before any other CUDA calls.
    cudaError = cudaGLSetGLDevice(activeCudaDevice);
    if(cudaError != cudaSuccess) qFatal("FlightPlannerParticles::slotInitialize(): couldn't set device to GL interop mode: code %d: %s, exiting.", cudaError, cudaGetErrorString(cudaError));

    cudaError = cudaSetDeviceFlags(cudaDeviceMapHost);// in order for the cudaHostAllocMapped flag to have any effect
    if(cudaError != cudaSuccess) qFatal("FlightPlannerParticles::slotInitialize(): couldn't set device flag: code %d: %s, exiting.", cudaError, cudaGetErrorString(cudaError));

    cudaDeviceProp deviceProps;
    cudaGetDeviceProperties(&deviceProps, activeCudaDevice);

    size_t memTotal, memFree;
    cudaMemGetInfo(&memFree, &memTotal);

    qDebug() << "FlightPlannerParticles::FlightPlannerParticles(): device" << deviceProps.name << "has compute capability" << deviceProps.major << deviceProps.minor << "and"
             << memFree / 1048576 << "of" << memTotal / 1048576 << "mb free, has"
             << deviceProps.multiProcessorCount << "multiprocessors,"
             << (deviceProps.integrated ? "is" : "is NOT" ) << "integrated,"
             << (deviceProps.canMapHostMemory ? "can" : "can NOT") << "map host mem, has"
             << deviceProps.memoryClockRate / 1000 << "Mhz mem clock, a"
             << deviceProps.memoryBusWidth << "bit mem bus and max"
             << deviceProps.maxTexture1DLinear / 1048576 << "mb of 1d texture bound to linear memory.";

    // Create the uniform buffer object (UBO) for all members of the UBO-Block
    mUboSize = 64 + 64; // One Matrix4x4 has 16 floats with 4 bytes each, giving 64 bytes.
    glGenBuffers(1, &mUboId);
    glBindBuffer(GL_UNIFORM_BUFFER, mUboId);
    glBufferData(GL_UNIFORM_BUFFER, mUboSize, NULL, GL_STREAM_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
    // Now bind this UBO to a uniform-block binding-point
    glBindBufferRange(GL_UNIFORM_BUFFER, ShaderProgram::blockBindingPointGlobalMatrices, mUboId, 0, mUboSize);

    mShaderProgramDefault = new ShaderProgram(this, "shader-default-vertex.c", "", "shader-default-fragment.c");

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
//    glClearColor(0.3f, 0.3f, 0.3f, 0.0f);					// Gray  Background

    // Set Line Antialiasing
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    // Find the oktokopter model and load it
    QDir modelPath = QDir::current();
    modelPath.cdUp();

    mModelVehicle = new Model(QFile(modelPath.absolutePath() + "/media/oktokopter.obj"), QString("../media/"), this);
    mModelThrust = new Model(QFile(modelPath.absolutePath() + "/media/cone-grey.obj"), QString("../media/"), this);
    mModelConeYaw = new Model(QFile(modelPath.absolutePath() + "/media/cone-green.obj"), QString("../media/"), this);
    mModelConePitch = new Model(QFile(modelPath.absolutePath() + "/media/cone-red.obj"), QString("../media/"), this);
    mModelConeRoll = new Model(QFile(modelPath.absolutePath() + "/media/cone-blue.obj"), QString("../media/"), this);
    mModelTarget = new Model(QFile(modelPath.absolutePath() + "/media/target.obj"), QString("../media/"), this);

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
    matrixCameraToClip.perspective(50.0f * mZoomFactorCurrent, (float)w/(float)h, 10.0f, +1000.0f);
    //matrixCameraToClip.ortho(-w/2.0f * mZoomFactorCurrent, w/2.0f * mZoomFactorCurrent, -h/2.0f * mZoomFactorCurrent, h/2.0f * mZoomFactorCurrent, 1.0, 10000.0);

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
    mTimeOfLastRender = QDateTime::currentDateTime();

    if(mViewRotating)
        rotY -= 180 * 0.001;

//    qDebug() << "GlWidget::paintGL(): frame counter:" << mFrameCounter++;

    // Clear color buffer and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const QVector3D camLookAt = mCamLookAtOffset + (mLastKnownVehiclePose ? mLastKnownVehiclePose->getPosition() : QVector3D());

    QQuaternion cameraRotation =
            QQuaternion::fromAxisAndAngle(QVector3D(0.0f, 0.0f, 1.0f), rotZ)
            * QQuaternion::fromAxisAndAngle(QVector3D(0.0f, 1.0f, 0.0f), rotY)
            * QQuaternion::fromAxisAndAngle(QVector3D(1.0f, 0.0f, 0.0f), -rotX);

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
        mTimerUpdate->setInterval(1000 / 30);
        mViewZooming = false;
    }

    mShaderProgramDefault->bind();
    {
        mShaderProgramDefault->setUniformValue("useMatrixExtra", false);

        glEnable (GL_BLEND); glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Beau.Ti.Ful!
        {
            for(int i=0;i<mPointCloudsToRender.size();i++)
            {
                const QVector<PointCloud::VboInfo>& vboInfoList = mPointCloudsToRender.at(i)->getVboInfo();
//                octree->updateVbo(); // update VBO from octree point-vector.
                mShaderProgramDefault->setUniformValue("useFixedColor", true);

                for(int j=0;j<vboInfoList.size();j++)
                {
                    const PointCloud::VboInfo& vboInfo = vboInfoList.at(j);

                    mShaderProgramDefault->setUniformValue("fixedColor",
                                                           QVector4D(
                                                               vboInfo.color.redF(),
                                                               vboInfo.color.greenF(),
                                                               vboInfo.color.blueF(),
                                                               vboInfo.color.alphaF()
                                                               )
                                                           );

                    glBindBuffer(GL_ARRAY_BUFFER, vboInfo.vbo);
                    glEnableVertexAttribArray(0);
                    glVertexAttribPointer(0, vboInfo.elementSize, GL_FLOAT, GL_FALSE, vboInfo.stride, 0);
                    glDrawArrays(GL_POINTS, 0, vboInfo.size); // Number of Elements, not bytes
                    glDisableVertexAttribArray(0);
                    glBindBuffer(GL_ARRAY_BUFFER, 0);
                }
            }

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

            // Render axes once at origin, once at vehicle
            {
                // At the origin
                mShaderProgramDefault->setUniformValue("useFixedColor", false);
                glBindBuffer(GL_ARRAY_BUFFER, mVboAxes);
                glEnableVertexAttribArray(0);
                glEnableVertexAttribArray(1);
                glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0); // positions
                glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, (void*)(12 * sizeof(float) * 4)); // colors
                glDrawArrays(GL_LINES, 0, 12);

                // At the vehicle
                if(mLastKnownVehiclePose)
                {
                    mShaderProgramDefault->setUniformValue("useMatrixExtra", true);
                    mShaderProgramDefault->setUniformValue("matrixExtra", mLastKnownVehiclePose->getMatrixConst());
                    glDrawArrays(GL_LINES, 0, 12);
                    glDisableVertexAttribArray(0);
                    glDisableVertexAttribArray(1);
                    glBindBuffer(GL_ARRAY_BUFFER, 0);
                    mShaderProgramDefault->setUniformValue("useMatrixExtra", false);
                }
            }
        }
        glDisable(GL_BLEND);
    }
    mShaderProgramDefault->release();

    emit visualizeNow();

    QMatrix4x4 transformVehicle;

    // At startup, a vehiclePose might not exist yet. If so, use the identty matrix
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

        // Render target position!
        if(!mLastFlightControllerValues->targetPosition.isNull())
        {
            QMatrix4x4 trTarget;
            trTarget.translate(mLastFlightControllerValues->targetPosition);
            mModelTarget->slotSetModelTransform(trTarget);
            mModelTarget->render();
        }
    }

    mModelVehicle->slotSetModelTransform(transformVehicle);
    mModelVehicle->render();
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

        const float data[] = {pos.x(), pos.y(), pos.z(), color.redF(), color.greenF(), color.blueF(), color.alphaF()};

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
    mViewRotating = enable;

    // Enable the timer if we want to rotate and its not running already
    if(mViewRotating && !mViewZooming)
        mTimerUpdate->start();
}
void GlWidget::mousePressEvent(QMouseEvent *event)
{
    mLastMousePosition = event->pos();
}

void GlWidget::mouseMoveEvent(QMouseEvent *event)
{
    float DX = float(event->x()-mLastMousePosition.x())/width();
    float DY = float(event->y()-mLastMousePosition.y())/height();

    if(event->buttons() & Qt::LeftButton)
    {
        rotX += 180 * DY;
        rotY += 180 * DX;
    }
    else if(event->buttons() & Qt::RightButton)
    {
        rotX += 180*DY;
        rotZ += 180*DX;
    }
    else if(event->buttons() & Qt::MiddleButton)
    {
        mCamLookAtOffset.setZ(mCamLookAtOffset.z() + 180*DY);
        mCamLookAtOffset.setX(mCamLookAtOffset.x() + 180*DX);
    }

    mLastMousePosition = event->pos();

    rotX = fmod(rotX, 360.0);
    rotY = fmod(rotY, 360.0);
    rotZ = fmod(rotZ, 360.0);

    //    qDebug() << "mCamLookAtOffset: " << mCamLookAtOffset << "rotXYZ:" << rotX << rotY << rotZ;

    //    slotEmitModelViewProjectionMatrix();

    update();
}

void GlWidget::wheelEvent(QWheelEvent *event)
{
    event->delta() > 0 ? mZoomFactorTarget *= 1.5f : mZoomFactorTarget *= 0.5f;
    mZoomFactorTarget = qBound(0.002f, (float)mZoomFactorTarget, 1.0f);
    //    qDebug() << "zoomFactor" << mZoomFactor;
    mViewZooming = true;
    mTimerUpdate->setInterval(1000 / 60);
    mTimerUpdate->start();
    slotUpdateView();
}

void GlWidget::slotUpdateView()
{
    if(mTimeOfLastRender.msecsTo(QDateTime::currentDateTime()) > mTimerUpdate->interval())
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

//    QTimer::singleShot(desiredInterval + 1 - mTimeOfLastRender.msecsTo(QDateTime::currentDateTime()), this, SLOT(slotUpdateView()));
}

void GlWidget::slotViewFromTop()
{
    rotX = 49.58;
    rotY = -17.71;
    rotZ = 19.67;
    mZoomFactorCurrent = 0.6;
    mZoomFactorTarget = 0.6;
    updateGL();
}

void GlWidget::slotViewFromSide()
{
    rotX = -34.0;
    rotY = 0.25;
    rotZ = -15.5;

    rotX = -23.0;
    rotY = -3.54;
    rotZ = -10.7;

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

void GlWidget::slotSetFlightControllerValues(const FlightControllerValues* const fcv)
{
    mLastFlightControllerValues = fcv;
    mLastKnownVehiclePose = &fcv->lastKnownPose;
    slotUpdateView();
}
