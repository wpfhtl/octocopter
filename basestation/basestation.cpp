#include "basestation.h"

#include "flightplannerbasic.h"
#include "flightplannerphysics.h"

BaseStation::BaseStation() : QMainWindow()

{
    qDebug() << "BaseStation::BaseStation()";
    mOctree = new Octree(
            QVector3D(-100, -100, -100), // min
            QVector3D(100, 100, 100),  // max
            1000);

    mOctree->setMinimumPointDistance(0.1);
    mOctree->setPointHandler(OpenGlUtilities::drawPoint);

    mIncomingDataBuffer.clear();

    mTcpSocket = new QTcpSocket(this);
    connect(mTcpSocket, SIGNAL(readyRead()), SLOT(slotReadSocket()));
    connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)), SLOT(slotSocketError(QAbstractSocket::SocketError)));
    connect(mTcpSocket, SIGNAL(connected()), SLOT(slotSocketConnected()));
    connect(mTcpSocket, SIGNAL(disconnected()), SLOT(slotSocketDisconnected()));

    mControlWidget = new ControlWidget(this);
    addDockWidget(Qt::RightDockWidgetArea, mControlWidget);
    connect(mControlWidget, SIGNAL(wayPointInsert(QString, int, const QList<WayPoint>&)), SLOT(slotWayPointInsert(QString, int, const QList<WayPoint>&)));
    connect(mControlWidget, SIGNAL(wayPointDelete(QString, int)), SLOT(slotWayPointDelete(QString, int)));

    mLogWidget = new LogWidget(this);
    addDockWidget(Qt::BottomDockWidgetArea, mLogWidget);
    menuBar()->addAction("Save Log", mLogWidget, SLOT(save()));

    mFlightPlanner = new FlightPlannerPhysics(&mVehiclePose, mOctree);
    mFlightPlanner->slotSetScanVolume(QVector3D(140, 60, 80), QVector3D(240, 120, 150));
    connect(mFlightPlanner, SIGNAL(newWayPointsReady(const QList<WayPoint>)), mControlWidget, SLOT(slotNewWayPoints(const QList<WayPoint>)));
    connect(mControlWidget, SIGNAL(setScanVolume(QVector3D,QVector3D)), mFlightPlanner, SLOT(slotSetScanVolume(QVector3D, QVector3D)));
    connect(mControlWidget, SIGNAL(generateWaypoints()), mFlightPlanner, SLOT(slotGenerateWaypoints()));

    menuBar()->addAction("Save Cloud", this, SLOT(slotExportCloud()));

    mGlWidget = new GlWidget(this, mOctree, mFlightPlanner);
    connect(mControlWidget, SIGNAL(setScanVolume(QVector3D,QVector3D)), mGlWidget, SLOT(update()));
    setCentralWidget(mGlWidget);

    connect(mGlWidget, SIGNAL(visualizeNow()), mFlightPlanner, SLOT(slotVisualize()));
    connect(mFlightPlanner, SIGNAL(suggestVisualization()), mGlWidget, SLOT(updateGL()));

    mTimerUpdateStatus = new QTimer();
    mTimerUpdateStatus->setInterval(500);
    connect(mTimerUpdateStatus, SIGNAL(timeout()), SLOT(slotGetStatus()));

    mPlotWidget = new PlotWidget(this);
    mPlotWidget->setAllowedAreas(Qt::AllDockWidgetAreas);
    addDockWidget(Qt::LeftDockWidgetArea, mPlotWidget);

    mPlotWidget->createCurve("cPitch");
    mPlotWidget->createCurve("cRoll");
    mPlotWidget->createCurve("cThrust");
    mPlotWidget->createCurve("cYaw");

    mPlotWidget->createCurve("pPitch");
    mPlotWidget->createCurve("pRoll");
    mPlotWidget->createCurve("pYaw");

    mLogWidget->log("startup finished, ready.");

    slotConnect();
}

BaseStation::~BaseStation()
{
}

void BaseStation::slotSocketDisconnected()
{
    qDebug() << "BaseStation::slotConnectionEnded()";

    mLogWidget->log("connection closed, retrying...");

    mTimerUpdateStatus->stop();

    mTcpSocket->connectToHost("localhost", 12345);
}

void BaseStation::slotSocketConnected()
{
    qDebug() << "BaseStation::slotSocketConnected()";

    mLogWidget->log("connection established.");

    mTimerUpdateStatus->start();
}

void BaseStation::slotExportCloud()
{
    const QString fileName = QFileDialog::getSaveFileName(this, "Save cloud to", QString(), "PLY files (*.ply)");

    if(fileName.isNull()) return;

    if(CloudExporter::savePly(this, mOctree, fileName))
      QMessageBox::information(this, "Cloud export", "Successfully wrote cloud to\n" + fileName, "OK");
    else
        QMessageBox::information(this, "Cloud export", "Failed saving cloud to file\n\n" + fileName, "OK");
}

void BaseStation::addRandomPoint()
{
    QVector3D scannerPosition(0, 0, 0);
    QVector3D point(rand()%199 - 100, rand()%199 - 100, rand()%199 - 100);
    mOctree->insertPoint(
            new LidarPoint(
                    point,
                    scannerPosition-point,
                    (rand()%20+1)
                    )
            );

    mLogWidget->log("added random point.");
}

void BaseStation::keyPressEvent(QKeyEvent* event)
{
    if(event->key() == Qt::Key_Space)
        addRandomPoint();
}

void BaseStation::slotReadSocket()
{
//    qDebug() << "BaseStation::slotReadSocket()";

    mIncomingDataBuffer.append(mTcpSocket->readAll());

    if(mIncomingDataBuffer.size() < 8) return;

//    mIncomingDataBuffer.remove(mIncomingDataBuffer.indexOf(QString("$KPROT").toAscii()), 6);

    QDataStream stream(mIncomingDataBuffer); // byteArray is const!
    quint32 packetLength;
    stream >> packetLength;

//    qDebug() << "BaseStation::slotReadSocket(): packetLength is" << packetLength << "buffersize is" << mIncomingDataBuffer.size();

    if(mIncomingDataBuffer.size() >= packetLength)
    {
        // strip the length from the beginning
        mIncomingDataBuffer.remove(0, sizeof(quint32));

        // pass the packet
        processPacket(mIncomingDataBuffer.left(packetLength - sizeof(quint32)));

        // now remove the packet from our incoming buffer
        mIncomingDataBuffer.remove(0, packetLength - sizeof(quint32));

        // see whether there's another packet lurking around in the array...
        slotReadSocket();
    }
}


void BaseStation::processPacket(QByteArray data)
{
//    qDebug() << "BaseStation::processPacket(): processing bytes:" << data.size();

    QDataStream stream(data);

    QString packetType;
    stream >> packetType;

    if(packetType == "lidarpoints")
    {
        QVector3D scannerPosition;
        QList<QVector3D> pointList;
//        quint32 numberOfHitsToReceive;
//        stream >> numberOfHitsToReceive;
        stream >> scannerPosition;
        stream >> pointList;

        mLogWidget->log(QString("received %1 points, inserting...").arg(pointList.size()));

        int i=0;
        foreach(const QVector3D &p, pointList)
        {
//            qDebug() << p;
            mOctree->insertPoint(new LidarPoint(p, (p-scannerPosition).normalized(), (p-scannerPosition).lengthSquared()));
            if(i%10 == 0)
                mFlightPlanner->insertPoint(new LidarPoint(p, (p-scannerPosition).normalized(), (p-scannerPosition).lengthSquared()));
            i++;
        }

        mGlWidget->update();

        mLogWidget->log(QString("%1 points using %2 MB, %3 nodes, %4 points added.").arg(mOctree->getNumberOfItems()).arg((mOctree->getNumberOfItems()*sizeof(LidarPoint))/1000000.0, 2, 'g').arg(mOctree->getNumberOfNodes()).arg(pointList.size()));

//        qDebug() << "appended" << pointList.size() << "points to octree.";
    }
    else if(packetType == "images")
    {
        QString cameraName;
        QVector3D cameraPosition;
        QQuaternion cameraOrientation;
        quint32 imageSize;
        QByteArray image;

        stream >> cameraName;
        stream >> cameraPosition;
        stream >> cameraOrientation;
        stream >> imageSize;

        image.resize(imageSize);

        qDebug() << "imageSize is" << imageSize << "actually read" << stream.readRawData(image.data(), imageSize) << "ba size" << image.size();

        qDebug() << "camera" << cameraName << "pos" << cameraPosition << "orientation" << cameraOrientation << "image bytearray size is" << image.size();

        if(!mCameraWindows.contains(cameraName))
            mCameraWindows.insert(cameraName, new CameraWindow(this, cameraName));

        CameraWindow* win = mCameraWindows.value(cameraName);

        win->slotSetPixmapData(image);
        win->show();
    }
    else if(packetType == "status")
    {
        QVector3D linearVelocity;
        quint32 simulationTime;
        double batteryVoltageMax, batteryVoltageCurrent;

        stream >> simulationTime;
        stream >> mVehiclePose;
        stream >> linearVelocity;
        stream >> batteryVoltageCurrent;
        stream >> batteryVoltageMax;

        mControlWidget->slotUpdateSimulationTime(simulationTime);
        mControlWidget->slotUpdatePose(mVehiclePose);
        mControlWidget->slotUpdateDynamics(linearVelocity);
        mControlWidget->slotUpdateBattery(batteryVoltageCurrent, batteryVoltageMax);
    }
    else if(packetType == "waypoints")
    {
        QList<WayPoint> wayPoints;

        stream >> wayPoints;

        mControlWidget->slotUpdateWayPoints(wayPoints);
    }
    else if(packetType == "waypointreached")
    {
        WayPoint wpt;
        stream >> wpt;

        mControlWidget->slotRoverReachedNextWayPoint();

        mLogWidget->log(QString("reached waypoint %1 %2 %3").arg(wpt.x()).arg(wpt.y()).arg(wpt.z()));
    }
    else if(packetType == "message")
    {
        QString text;
        stream >> text;

        mLogWidget->log(QString("rover: ") + text);
    }
    else if(packetType == "controllervalues")
    {
        Pose pose;
        quint8 thrust;
        qint8 pitch, roll, yaw, height;

        stream >> pose;
        stream >> thrust;
        stream >> pitch;
        stream >> roll;
        stream >> yaw;
        stream >> height;

        QVector<float> values;
        values << pitch << roll << thrust << yaw;
        values << pose.getPitchDegrees() << pose.getRollDegrees() << pose.getYawDegrees();

        mPlotWidget->slotAppendData(values);
    }
    else
    {
        qDebug() << "BaseStation::processPacket(): unknown packetType" << packetType;
        mLogWidget->log("unknown packetType: " + packetType);
        mIncomingDataBuffer.clear();
    }
}

void BaseStation::slotGetStatus()
{
//    mLogWidget->log("asking rover for status.");

    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("getstatus");

    slotSendData(data);
}

void BaseStation::slotWayPointInsert(QString hash, int index, const QList<WayPoint>& wayPoints)
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("waypointinsert");
    stream << hash;
    stream << (quint16)index;
    stream << wayPoints;

    mLogWidget->log(QString("inserting %1 waypoints").arg(wayPoints.size()));

    slotSendData(data);
}

void BaseStation::slotWayPointDelete(QString hash, int index)
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("waypointdelete");
    stream << hash;
    stream << (quint16)index;

    mLogWidget->log(QString("deleting waypoint at index %4").arg(index));

    slotSendData(data);
}

void BaseStation::slotSendData(const QByteArray &data)
{
//    qDebug() << "BaseStation::slotSendData():" << data << mTcpSocket->state();
//    qDebug() << "BaseStation::slotSendData():" << mTcpSocket->errorString();

    QByteArray dataToSend;
    QDataStream streamDataToSend(&dataToSend, QIODevice::WriteOnly);
//    streamDataToSend << QString("$KPROT").toAscii();
    streamDataToSend << (quint32)(data.length() + sizeof(quint32));

    dataToSend.append(data);

    mTcpSocket->write(dataToSend);
}

void BaseStation::slotConnect()
{
    mTcpSocket->connectToHost("localhost", 12345);
}

void BaseStation::slotSocketError(QAbstractSocket::SocketError socketError)
{
    qDebug() << "BaseStation::slotSocketError():" << socketError;

    if(socketError == QAbstractSocket::ConnectionRefusedError)
        QTimer::singleShot(2000, this, SLOT(slotConnect()));
}

const WayPoint BaseStation::getNextWayPoint(void) const
{
    return mControlWidget->getNextWayPoint();
}
