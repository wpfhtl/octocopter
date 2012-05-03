#include <GL/glew.h>
#include "glwidget.h"

GlWidget::GlWidget(QWidget* parent, Octree* octree, FlightPlannerInterface* flightPlanner) :
    QGLWidget(parent),
    mOctree(octree),
    mFlightPlanner(flightPlanner)
{
    QGLFormat glFormat;
    glFormat.setSamples(2);
    glFormat.setSampleBuffers(true);
    glFormat.setVersion(4,0);
    // This means no OpenGL-deprecated stuff is used (like glBegin() and glEnd())
//    glFormat.setProfile(QGLFormat::CoreProfile);
    glFormat.setProfile(QGLFormat::CompatibilityProfile);
    QGLFormat::setDefaultFormat(glFormat);
    setFormat(glFormat);

    mCameraPosition = QVector3D(0.0f, 500.0f, 500.0f);

    mVboPointCloudBytesCurrent = 0;
    mVboPointCloudBytesMax = 2 * 1000 * 1000 * sizeof(QVector3D); // storage for 2 million points

    // For a flight time of one hour, the vboVehiclePath will need (3600 seconds * 50 Positions/second * (sizeof(QVector3D_Position) + sizeof(QVector3D_Velocity))) bytes
    mVboVehiclePathBytesMaximum = (3600 * 50 * (sizeof(QVector3D) + sizeof(QVector3D)));
    mVboVehiclePathBytesCurrent = 0;
    mVboVehiclePathElementSize = sizeof(QVector3D); // just one Vector3D, no velocity yet.

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

    mShaderProgramPointCloud = new ShaderProgram(this, "shader-default-vertex.c", "", "shader-pointcloud-fragment.c");

    mShaderProgramVehiclePath = new ShaderProgram(this, "shader-default-vertex.c", "", "shader-default-fragment.c");

    // Create a VBO for the vehicle's path.
    glGenBuffers(1, &mVboVehiclePath);
    qDebug() << "GlWidget::initializeGL(): creating vehicle-path VBO" << mVboVehiclePath << "of size" << mVboVehiclePathBytesMaximum;
    glBindBuffer(GL_ARRAY_BUFFER, mVboVehiclePath);
    glBufferData(GL_ARRAY_BUFFER, mVboVehiclePathBytesMaximum, NULL, GL_DYNAMIC_DRAW);
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
    QVector<QVector3D> p;
    p.append(QVector3D(0, 0, 0));
    p.append(QVector3D(0, 0, 1.0));
    p.append(QVector3D(0, 1.0, 0));
    p.append(QVector3D(0, 1.0, 1.0));
    p.append(QVector3D(1.0, 0, 0));
    p.append(QVector3D(1.0, 0, 1.0));
    p.append(QVector3D(1.0, 1.0, 0));
    p.append(QVector3D(1.0, 1.0, 1.0));

    slotInsertLidarPoints(p);

    mModel = new Model("oktokopter.xml", this);
}

void GlWidget::resizeGL(int w, int h)
{
    // setup viewport, projection etc.
    glViewport(0, 0, w, h);

    // OpenGL 4 core profile: We set the second matrix (perspective = cameraToClip) in the UBO
    QMatrix4x4 matrixCameraToClip;
    matrixCameraToClip.perspective(50.0f * mZoomFactorCurrent, (float)w/(float)h, 10.0f, +8000.0f);
    //matrixCameraToClip.ortho(-w/2.0f * mZoomFactorCurrent, w/2.0f * mZoomFactorCurrent, -h/2.0f * mZoomFactorCurrent, h/2.0f * mZoomFactorCurrent, 1.0, 10000.0);

    qDebug() << "GlWidget::resizeGL(): resizing gl viewport to" << w << h << "setting perspective/cameraclip matrix" << matrixCameraToClip;

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

    QVector3D camPos = cameraRotation.rotatedVector(mCameraPosition);

    qDebug() << "GlWidget::paintGL(): cam pos after rotating:" << camPos;

    // Write the modelToCamera matrix into our UBO
    QMatrix4x4 matrixModelToCamera;
    matrixModelToCamera.lookAt(camPos, camLookAt, QVector3D(0.0f, 1.0f, 0.0f));
    glBindBuffer(GL_UNIFORM_BUFFER, mUboId);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, 64, OpenGlUtilities::matrixToOpenGl(matrixModelToCamera).constData());
    glBindBuffer(GL_UNIFORM_BUFFER, 0);

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

//    drawAxes(10, 10, 10, 0.8, 0.8, 0.8);
//    drawVehicle();
//    drawVehicleVelocity();

    // Render the vehicle's path
    mShaderProgramVehiclePath->bind();
    glBindBuffer(GL_ARRAY_BUFFER, mVboVehiclePath);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0); // 12 bytes stride, we don't draw 3d velocity yet.
    //qDebug() << "GlWidget::paintGL(): vehicle path elements:" << mVboVehiclePathBytesCurrent / mVboVehiclePathElementSize << "bytes:" << mVboVehiclePathBytesCurrent;
    glDrawArrays(GL_POINTS, 0, mVboVehiclePathBytesCurrent / mVboVehiclePathElementSize);
    glDisableVertexAttribArray(0);
    mShaderProgramVehiclePath->release();

    // Render pointcloud using all initialized VBOs (there might be none when no points exist)
    mShaderProgramPointCloud->bind();
    QMapIterator<GLuint, unsigned int> i(mVboIdsPointCloud);
    while (i.hasNext())
    {
        i.next();
        glBindBuffer(GL_ARRAY_BUFFER, i.key());
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glDrawArrays(GL_POINTS, 0, i.value() / sizeof(QVector3D));
        glDisableVertexAttribArray(0);
    }
    mShaderProgramPointCloud->release();

    mModel->render();

    /* old pointcloud rendering code
    glDisable(GL_LIGHTING);
    glPointSize(1);
    glColor4f(.5f, .5f, .5f, 0.7f);
    glBegin(GL_POINTS);
    mOctree->handlePoints();
    glEnd();
    //glEnable(GL_LIGHTING);*/

    emit visualizeNow();
    qDebug() << "GlWidget::paintGL(): rendering time in milliseconds:" << renderTime.elapsed();
}

void GlWidget::slotNewVehiclePose(Pose pose)
{
    if(mVboVehiclePathBytesCurrent + mVboVehiclePathElementSize < mVboVehiclePathBytesMaximum)
    {
        const QVector3D pos = pose.getPosition();
        const QVector3D vel = (pos - mLastKnownVehiclePose.getPosition()) / ((mLastKnownVehiclePose.timestamp - pose.timestamp) / 1000.0f);
        const float data[6] = {pos.x(), pos.y(), pos.z(), vel.x(), vel.y(), vel.z()};

        glBindBuffer(GL_ARRAY_BUFFER, mVboVehiclePath);

        glBufferSubData(
                    GL_ARRAY_BUFFER,
                    mVboVehiclePathBytesCurrent, // offset in the VBO
                    mVboVehiclePathElementSize, // how many bytes to store?
                    (void*)data // data to store
                    );

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        mVboVehiclePathBytesCurrent += mVboVehiclePathElementSize;
        qDebug() << "GlWidget::slotNewVehiclePose(): inserted data" << pose.getPosition() << "into VBO, will now redraw";
    }
    else
    {
        qDebug() << "GlWidget::slotNewVehiclePose(): VBO is full, discarding new pose.";
    }

    mLastKnownVehiclePose = pose;

    slotUpdateView();
}

void GlWidget::drawVehicle() const
{
    // draw vehicle velocity as vector
    const QVector3D vehiclePosition = mLastKnownVehiclePose.getPosition();
    const QQuaternion vehicleOrientation = mLastKnownVehiclePose.getOrientation();

    const QVector3D armFront = vehicleOrientation.rotatedVector(QVector3D(0.0, 0.0, -0.4));
    const QVector3D armBack = vehicleOrientation.rotatedVector(QVector3D(0.0, 0.0, 0.4));
    const QVector3D armLeft = vehicleOrientation.rotatedVector(QVector3D(-0.4, 0.0, 0.0));
    const QVector3D armRight = vehicleOrientation.rotatedVector(QVector3D(0.4, 0.0, 0.0));

    const QVector3D landingLegFront = vehicleOrientation.rotatedVector(QVector3D(0.0, -0.2, -0.2));
    const QVector3D landingLegBack = vehicleOrientation.rotatedVector(QVector3D(0.0, -0.2, 0.2));
    const QVector3D landingLegLeft = vehicleOrientation.rotatedVector(QVector3D(-0.2, -0.2, 0.0));
    const QVector3D landingLegRight = vehicleOrientation.rotatedVector(QVector3D(0.2, -0.2, 0.0));

    glTranslatef(0.0f, 0.2f, 0.0f);

    OpenGlUtilities::drawSphere(vehiclePosition, 0.03, 10.0, QColor(80,80,80,200));
    glDisable(GL_LIGHTING);


    glLineWidth(3);
    glBegin(GL_LINES);
    glColor3f(1.0, 0.2, 0.2);

    // draw arms
    glVertexVector(vehiclePosition);
    glVertexVector(vehiclePosition + armFront);

    glColor3f(0.4, 0.4, 0.4);
    glVertexVector(vehiclePosition);
    glVertexVector(vehiclePosition + armBack);

    glVertexVector(vehiclePosition);
    glVertexVector(vehiclePosition + armLeft);

    glVertexVector(vehiclePosition);
    glVertexVector(vehiclePosition + armRight);

    glColor3f(0.7, 0.7, 0.7);

    // draw landing legs
    glVertexVector(vehiclePosition + armFront * 0.5);
    glVertexVector(vehiclePosition + landingLegFront);

    glVertexVector(vehiclePosition + armBack * 0.5);
    glVertexVector(vehiclePosition + landingLegBack);

    glVertexVector(vehiclePosition + armLeft * 0.5);
    glVertexVector(vehiclePosition + landingLegLeft);

    glVertexVector(vehiclePosition + armRight * 0.5);
    glVertexVector(vehiclePosition + landingLegRight);

    glEnd();
    glEnable(GL_LIGHTING);
}

void GlWidget::drawVehicleVelocity() const
{
    // draw vehicle velocity as vector
    const QVector3D vehiclePosition = mFlightPlanner->getLastKnownVehiclePose().getPosition();
    const QVector3D vehicleVelocity = mFlightPlanner->getCurrentVehicleVelocity();
    glDisable(GL_LIGHTING);
    glColor3f(0.0, 1.0, 0.0);
    glLineWidth(1);
    glBegin(GL_LINES);
    glVertex3f(vehiclePosition.x(), vehiclePosition.y(), vehiclePosition.z());
    glVertex3f(vehiclePosition.x()+vehicleVelocity.x(), vehiclePosition.y()+vehicleVelocity.y(), vehiclePosition.z()+vehicleVelocity.z());
    glEnd();
    glEnable(GL_LIGHTING);
}

void GlWidget::drawAxes(
        const GLfloat& x, const GLfloat& y, const GLfloat& z,
        const GLfloat& red, const GLfloat& green, const GLfloat& blue) const
{
    glDisable(GL_LIGHTING);
    glLineWidth(2);
    glBegin(GL_LINES);
    // X
    glColor3f(red,green,blue);
    glVertex3f( -x,0.0f,0.0f); glVertex3f(0.0f,0.0f,0.0f);
    glColor3f(1.0f,green,blue);
    glVertex3f(0.0f,0.0f,0.0f); glVertex3f(  x,0.0f,0.0f);

    // Y
    glColor3f(red,green,blue);
    glVertex3f(0.0f,  -y,0.0f); glVertex3f(0.0f,0.0f,0.0f);
    glColor3f(red,1.0f,blue);
    glVertex3f(0.0f,0.0f,0.0f); glVertex3f(0.0f,   y,0.0f);

    // Y
    glColor3f(red,green,blue);
    glVertex3f(0.0f,0.0f,  -z); glVertex3f(0.0f,0.0f,0.0f);
    glColor3f(red,green,1.0f);
    glVertex3f(0.0f,0.0f,0.0f); glVertex3f(0.0f,0.0f,   z);
    glEnd();
    glEnable(GL_LIGHTING);
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

void GlWidget::slotInsertLidarPoints(const QVector<QVector3D>& list)
{
    //qDebug() << "GlWidget::slotInsertLidarPoints(): trying to insert" << list.size() << "elements," << list.size() * sizeof(QVector3D) << "bytes.";
    unsigned int numberOfPointsToStore = list.size();
    unsigned int numberOfPointsSucessfullyStored = 0;
    unsigned int numberOfVbosCreated = 0;

    while(numberOfPointsToStore)
    {
        // Insert the lidarpoints into any VBO that can accomodate them
        QMapIterator<GLuint, unsigned int> it(mVboIdsPointCloud);
        while(it.hasNext() && numberOfPointsToStore)
        {
            it.next();

            //qDebug() << "GlWidget::slotInsertLidarPoints(): checking VBO" << it.key() << "which already contains" << it.value() << "bytes...";

            // Only fill this VBO if it has enough free space for at least one point
            const unsigned int numberOfPointsStorableInThisVbo = std::min((long unsigned int)numberOfPointsToStore, (mVboPointCloudBytesMax/sizeof(QVector3D)) - it.value()/sizeof(QVector3D));
            if(numberOfPointsStorableInThisVbo)
            {
                glBindBuffer(GL_ARRAY_BUFFER, it.key());

                glBufferSubData(
                            GL_ARRAY_BUFFER,
                            it.value(), // offset in the VBO
                            numberOfPointsStorableInThisVbo * sizeof(QVector3D), // how many bytes to store?
                            (void*)(list.data() + (numberOfPointsSucessfullyStored * sizeof(QVector3D))) // data to store
                            );

                glBindBuffer(GL_ARRAY_BUFFER, 0);

                numberOfPointsSucessfullyStored += numberOfPointsStorableInThisVbo;
                numberOfPointsToStore -= numberOfPointsStorableInThisVbo;

                // Update the number of bytes used
                mVboIdsPointCloud.insert(it.key(), it.value() + numberOfPointsStorableInThisVbo * sizeof(QVector3D));
            }
        }

        // We filled the existing VBOs with points above. But if we still
        // have a numberOfPointsToStore, we need to create a new VBO
        if(numberOfPointsToStore)
        {
            // call glGetError() to clear eventually present errors
            glGetError();

            // Initialize the pointcloud-VBO
            GLuint vboPointCloud;
            glGenBuffers(1, &vboPointCloud);
            glBindBuffer(GL_ARRAY_BUFFER, vboPointCloud);
            glBufferData(GL_ARRAY_BUFFER, mVboPointCloudBytesMax, NULL, GL_DYNAMIC_DRAW);

            if(glGetError() == GL_NO_ERROR)
            {
                qDebug() << "GlWidget::slotInsertLidarPoints(): Created new VBO" << vboPointCloud << "containing" << mVboPointCloudBytesMax << "bytes";
                mVboIdsPointCloud.insert(vboPointCloud, 0);
                numberOfVbosCreated++;
            }
            else
            {
                qDebug() << "GlWidget::slotInsertLidarPoints(): Couldn't create VBO containing" << mVboPointCloudBytesMax << "bytes - discarding" << numberOfPointsToStore << "points.";
            }

            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }
    }

    /*qDebug() << "GlWidget::slotInsertLidarPoints(): created" << numberOfVbosCreated << "VBOs, inserted" << numberOfPointsSucessfullyStored << "points. Statistics follow:";

    QMapIterator<GLuint, unsigned int> it2(mVbosPointCloud);
    while(it2.hasNext())
    {
        it2.next();
        qDebug() << "GlWidget::slotInsertLidarPoints(): VBO" << it2.key() << "has size" << it2.value();
    }*/
}
