#include <GL/glew.h>
#include <GL/gl.h>

#include "octree.h"
#include "glwidget.h"

GlWidget::GlWidget(QWidget* parent, FlightPlannerInterface* flightPlanner) :
    QGLWidget(parent),
    mFlightPlanner(flightPlanner)
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

//    mVboPointCloudBytesCurrent = 0;
//    mVboPointCloudBytesMax = 2 * 1000 * 1000 * sizeof(QVector3D); // storage for 2 million points

    mVboVehiclePathElementSize = sizeof(QVector3D) + sizeof(QVector4D); // position and color with alpha
    mVboVehiclePathBytesMaximum = (3600 * 50 * mVboVehiclePathElementSize); // For a flight time of one hour
    mVboVehiclePathBytesCurrent = 0;

    mZoomFactorCurrent = 0.5;
    mZoomFactorTarget = 0.5;

    // Mouse Move Rotations
    rotX = rotY = rotZ = 0.0f;

    // Timer Animation
    mTimerIdZoom = 0;
    mTimerIdRotate = 0;

    mTimeOfLastExternalUpdate = QDateTime::currentDateTime();

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

    // Give e.g. FlightPlannerCuda a chance to initialize CUDA in a GL context
    emit initializingInGlContext();

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
    glClearColor(0.3f, 0.3f, 0.3f, 0.0f);					// Gray  Background

    // Set Line Antialiasing
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    //    glEnable(GL_LIGHTING);
    //    glEnable(GL_LIGHT0);

    // Enable Blending and set the type to be used
    //    glEnable(GL_BLEND);
    //    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //    glBlendFunc( GL_SRC_ALPHA, GL_ONE );
    //    glBlendFunc( GL_ZERO, GL_ONE_MINUS_SRC_ALPHA );
    //    glBlendFunc( GL_SRC_ALPHA_SATURATE, GL_ONE );
    //    glBlendFunc(GL_ONE_MINUS_DST_ALPHA,GL_DST_ALPHA);

    // Just for debugging
    /*
    QVector<QVector3D> p;
    p.append(QVector3D(0, 0, 0));
    p.append(QVector3D(0, 0, 1.0));
    p.append(QVector3D(0, 1.0, 0));
    p.append(QVector3D(0, 1.0, 1.0));
    p.append(QVector3D(1.0, 0, 0));
    p.append(QVector3D(1.0, 0, 1.0));
    p.append(QVector3D(1.0, 1.0, 0));
    p.append(QVector3D(1.0, 1.0, 1.0));
    syncOctreeToVbo(p);*/

    // Find the oktokopter model and load it
    QDir modelPath = QDir::current();
    modelPath.cdUp();
    mModelVehicle = new Model(QFile(modelPath.absolutePath() + "/media/oktokopter.obj"), QString("../media/"), this);
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
    QTime renderTime;
    renderTime.start();

    // Clear color buffer and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const QVector3D vehiclePosition = mFlightPlanner->getLastKnownVehiclePose().getPosition();
    const QVector3D camLookAt = mCamLookAtOffset + vehiclePosition;

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

    if(fabs(step) > 0.00001f)
    {
        if(mTimerIdZoom == 0) mTimerIdZoom = startTimer(20);
        mZoomFactorCurrent += step;
        resizeGL(width(),height()); // this sets up a new view with the new zoomFactor
    }
    else if(mTimerIdZoom != 0)
    {
        killTimer(mTimerIdZoom);
        mTimerIdZoom = 0;
    }

    emit visualizeNow();

    mShaderProgramDefault->bind();
    {
        mShaderProgramDefault->setUniformValue("useMatrixExtra", false);

        glEnable (GL_BLEND); glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Beau.Ti.Ful!
        {
            syncOctreeToVbo();
            for(int i=0;i<mOctrees.size();i++)
            {
                Octree* octree = mOctrees.at(i);
                mShaderProgramDefault->setUniformValue("useFixedColor", true);
                mShaderProgramDefault->setUniformValue("fixedColor",
                                                       QVector4D(
                                                           octree->mPointColor.redF(),
                                                           octree->mPointColor.greenF(),
                                                           octree->mPointColor.blueF(),
                                                           octree->mPointColor.alphaF()
                                                           )
                                                       );

                // Render pointcloud using all initialized VBOs (there might be none when no points exist)
                QMapIterator<quint32, quint32> i(octree->mVboIdsAndSizes);
                while(i.hasNext())
                {
                    i.next();
                    glBindBuffer(GL_ARRAY_BUFFER, i.key());
                    glEnableVertexAttribArray(0);
                    //glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
//                    qDebug() << "GlWidget::paintGL(): rendering octree" << octree << "vbo" << i.key() << "with" << i.value() << "elements";
                    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 24, 0);
                    glDrawArrays(GL_POINTS, 0, i.value()); // Number of Elements, not bytes

//                    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 24, 0);
//                    glDrawArrays(GL_POINTS, 0, i.value()); // Number of Elements, not bytes

//                    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 24, 0);
//                    glDrawArrays(GL_POINTS, 0, i.value()); // Number of Elements, not bytes

//                    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 24, 0);
//                    glDrawArrays(GL_POINTS, 0, i.value()); // Number of Elements, not bytes

                    glDisableVertexAttribArray(0);
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
                mShaderProgramDefault->setUniformValue("useMatrixExtra", true);
                mShaderProgramDefault->setUniformValue("matrixExtra", mLastKnownVehiclePose.getMatrix());
                glDrawArrays(GL_LINES, 0, 12);
                glDisableVertexAttribArray(0);
                glDisableVertexAttribArray(1);
                glBindBuffer(GL_ARRAY_BUFFER, 0);
                mShaderProgramDefault->setUniformValue("useMatrixExtra", false);
            }
        }
        glDisable(GL_BLEND);
    }
    mShaderProgramDefault->release();

    mModelVehicle->slotSetModelTransform(mLastKnownVehiclePose.getMatrix());
    mModelVehicle->render();

    //    qDebug() << "GlWidget::paintGL(): rendering time in milliseconds:" << renderTime.elapsed();
}

void GlWidget::syncOctreeToVbo()
{
    for(int i=0;i<mOctrees.size();i++)
    {
        Octree* octree = mOctrees.at(i);

        // Check whether this octree has more points stored than the VBO
        quint32 numberOfPointsToStoreInAllVbos = octree->getNumberOfItems() - octree->mElementsStoredInAllVbos;

        while(numberOfPointsToStoreInAllVbos)
        {
            // Insert the lidarpoints into any VBO that can accomodate them
            // This maps from VBO-ID to numberOfElements (not bytes)
            QMapIterator<quint32, quint32> it(octree->mVboIdsAndSizes);
            while(it.hasNext() && numberOfPointsToStoreInAllVbos)
            {
                it.next();

                //qDebug() << "GlWidget::syncOctreeToVbo(): checking VBO" << it.key() << "which already contains" << it.value() << "bytes...";

                // Only fill this VBO if it has enough free space for at least one point
                // An Octree can grow bigger than octree::mExpectedMaximumElementCount, but the VBOs are created for these many elements
                const unsigned int numberOfPointsToStoreInThisVbo = std::min(numberOfPointsToStoreInAllVbos, octree->mExpectedMaximumElementCount - it.value());

                if(numberOfPointsToStoreInThisVbo)
                {
                    glBindBuffer(GL_ARRAY_BUFFER, it.key());

                    quint32 byteOffset = it.value() * sizeof(LidarPoint);

                    // For updates < 32kb, glBufferSubData is supposed to be better than glMapBuffer
                    qDebug() << "GlWidget::syncOctreeToVbo(): octree elements:" << octree->data()->size() <<
                                "appending" << numberOfPointsToStoreInThisVbo <<
                                "elements /" << numberOfPointsToStoreInThisVbo * sizeof(LidarPoint) <<
                                "bytes into VBO" << it.key() <<
                                "at offset" << it.value() << "elements /" << byteOffset << "bytes";

//                    qDebug() << "GlWidget::syncOctreeToVbo(): first x:" << ((LidarPoint*)(octree->data()->constData() + (it.value() * sizeof(LidarPoint))))->position.x();

                    /*qDebug() << "GlWidget::syncOctreeToVbo(): last  x:" <<
                                ((LidarPoint*)(
                                     octree->data()->constData()
                                     + ((numberOfPointsToStoreInThisVbo-2) * sizeof(LidarPoint))
                                     + (it.value() * sizeof(LidarPoint))))
                                ->position.x();*/

                    glBufferSubData(
                                GL_ARRAY_BUFFER,
                                byteOffset, // offset in the VBO
                                numberOfPointsToStoreInThisVbo * sizeof(LidarPoint), // how many bytes to store?
                                (void*)(octree->data()->constData() + it.value()) // data to store
                                );

                    glBindBuffer(GL_ARRAY_BUFFER, 0);

                    numberOfPointsToStoreInAllVbos -= numberOfPointsToStoreInThisVbo;

                    // Update the number of bytes used
                    octree->mVboIdsAndSizes.insert(it.key(), it.value() + numberOfPointsToStoreInThisVbo);
                    octree->mElementsStoredInAllVbos += numberOfPointsToStoreInThisVbo;
                }
            }

            // We filled the existing VBOs with points above. But if we still
            // have a numberOfPointsToStore, we need to create a new VBO
            if(numberOfPointsToStoreInAllVbos)
            {
                // call glGetError() to clear eventually present errors
                glGetError();

                // Initialize the pointcloud-VBO
                const quint32 vboNewByteSize = octree->mExpectedMaximumElementCount * sizeof(LidarPoint);
                GLuint vboNew;
                glGenBuffers(1, &vboNew);
                glBindBuffer(GL_ARRAY_BUFFER, vboNew);
                glBufferData(GL_ARRAY_BUFFER, vboNewByteSize, NULL, GL_DYNAMIC_DRAW);

                if(glGetError() == GL_NO_ERROR)
                {
                    qDebug() << "GlWidget::syncOctreeToVbo(): Created new VBO" << vboNew << "containing" << vboNewByteSize << "bytes";
                    octree->mVboIdsAndSizes.insert(vboNew, 0);
                }
                else
                {
                    qDebug() << "GlWidget::syncOctreeToVbo(): Couldn't create VBO containing" << vboNewByteSize << "bytes!";
                }

                glBindBuffer(GL_ARRAY_BUFFER, 0);
            }
        }


        /*qDebug() << "GlWidget::syncOctreeToVbo(): created" << numberOfVbosCreated << "VBOs, inserted" << numberOfPointsSucessfullyStored << "points. Statistics follow:";

    QMapIterator<GLuint, unsigned int> it2(mVbosPointCloud);
    while(it2.hasNext())
    {
        it2.next();
        qDebug() << "GlWidget::syncOctreeToVbo(): VBO" << it2.key() << "has size" << it2.value();
    }*/
    }
}

void GlWidget::slotNewVehiclePose(Pose pose)
{
    if(mVboVehiclePathBytesCurrent + mVboVehiclePathElementSize < mVboVehiclePathBytesMaximum)
    {
        const QVector3D pos = pose.getPosition();
        const QVector3D vel = (pos - mLastKnownVehiclePose.getPosition()) / ((mLastKnownVehiclePose.timestamp - pose.timestamp) / 1000.0f);

        QColor color;
        if(
                pose.precision & Pose::RtkFixed &&
                pose.precision & Pose::AttitudeAvailable &&
                pose.precision & Pose::CorrectionAgeLow &&
                pose.precision & Pose::HeadingFixed &&
                pose.precision & Pose::ModeIntegrated
                )
            color.setRgb(0, 255, 0);
        else
            color.setRgb(255, 0, 0);

        // If the poses CV sucks, fade it.
        if(pose.covariances > Pose::maximumUsableCovariance) color.setAlpha(128);

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

void GlWidget::slotEnableTimerRotation(const bool& enable)
{
    if(enable && mTimerIdRotate == 0)
    {
        qDebug() << "GlWidget::slotEnableTimerRotation(): enabling timer";
        mTimerIdRotate = startTimer(30);
    }
    else if(!enable && mTimerIdRotate != 0)
    {
        qDebug() << "GlWidget::slotEnableTimerRotation(): disabing timer";
        killTimer(mTimerIdRotate);
        mTimerIdRotate = 0;
    }
}

void GlWidget::mousePressEvent(QMouseEvent *event)
{
    mLastMousePosition = event->pos();

    emit mouseClickedAtWorldPos(event->button(), convertMouseToWorldPosition(event->pos()));
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
    update();
}

void GlWidget::slotUpdateView()
{
    mTimeOfLastExternalUpdate = QDateTime::currentDateTime();
    update();
}

void GlWidget::timerEvent ( QTimerEvent * event )
{
    if(event->timerId() == mTimerIdRotate)
    {
        //        qDebug() << "GlWidget::timerEvent(): rotating...";
        rotY -= 180 * 0.001;
    }
    else if(event->timerId() == mTimerIdZoom)
    {
        //        qDebug() << "GlWidget::timerEvent(): zooming...";
    }

    // We should redraw for both zooming and rotating. But if the helicopter is flying, that will mean
    // two sources causing redraws all the time, which is slow. So, only update if there was recent update.
    const int interval = mTimeOfLastExternalUpdate.msecsTo(QDateTime::currentDateTime());
    if(interval > 60)
    {
        //        qDebug() << "GlWidget::timerEvent(): last external update was" << interval << "ms ago, updating";
        //        slotEmitModelViewProjectionMatrix();
        update();
    }
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

QVector3D GlWidget::convertMouseToWorldPosition(const QPoint& point)
{
    return QVector3D();
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posX, posY, posZ;

    glLoadIdentity();
    const QVector3D vehiclePosition = mFlightPlanner->getLastKnownVehiclePose().getPosition();
    mCamLookAtOffset = vehiclePosition;
    QVector3D min, max;
    mFlightPlanner->getScanVolume(min, max);
    mCamLookAtOffset = min + (max - min)/2.0;

    gluLookAt(0.0, 500.0, 500.0,
              mCamLookAtOffset.x(), mCamLookAtOffset.y(), mCamLookAtOffset.z(),
              0.0, 1.0, 0.0);

    glTranslatef(mCamLookAtOffset.x(), mCamLookAtOffset.y(), mCamLookAtOffset.z());

    // Mouse Move Rotations
    glRotatef(rotX,1.0,0.0,0.0);
    glRotatef(rotY,0.0,1.0,0.0);
    glRotatef(rotZ,0.0,0.0,1.0);

    glTranslatef(-mCamLookAtOffset.x(), -mCamLookAtOffset.y(), -mCamLookAtOffset.z());

    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );

    winX = (float)point.x();
    winY = (float)viewport[3] - (float)point.y();
    glReadPixels( point.x(), int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );

    gluUnProject(winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);

    //    qDebug() << "world pos of mouse" << point.x() << point.y() << "is" << posX << posY << posZ;

    return QVector3D(posX, posY, posZ);
}

void GlWidget::slotOctreeRegister(Octree* o)
{
    mOctrees.append(o);
}

void GlWidget::slotOctreeUnregister(Octree* o)
{
    mOctrees.removeOne(o);
}
