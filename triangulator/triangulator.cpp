#include "triangulator.h"

#include "flightplannerbasic.h"
#include "flightplannerphysics.h"

Triangulator::Triangulator() : QMainWindow()

{
    qDebug() << "Triangulator::Triangulator()";
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
    connect(mControlWidget, SIGNAL(wayPointInsert(QString, int, QVector3D)), SLOT(slotWayPointInsert(QString, int, QVector3D)));
    connect(mControlWidget, SIGNAL(wayPointDelete(QString, int)), SLOT(slotWayPointDelete(QString, int)));

    mLogWidget = new LogWidget(this);
    addDockWidget(Qt::BottomDockWidgetArea, mLogWidget);
    menuBar()->addAction("Save Log", mLogWidget, SLOT(save()));

    mFlightPlanner = new FlightPlannerPhysics(new QVector3D, new QQuaternion, mOctree);
    mFlightPlanner->slotSetScanVolume(QVector3D(80, 30, 20), QVector3D(280, 150, 220));
    connect(mFlightPlanner, SIGNAL(newWayPoint(const QVector3D)), mControlWidget, SLOT(slotNewWayPoint(const QVector3D)));
    connect(mControlWidget, SIGNAL(setScanVolume(QVector3D,QVector3D)), mFlightPlanner, SLOT(slotSetScanVolume(QVector3D, QVector3D)));
    connect(mControlWidget, SIGNAL(generateWaypoints()), mFlightPlanner, SLOT(slotGenerateWaypoints()));

    menuBar()->addAction("Save Cloud", this, SLOT(slotExportCloud()));

    mGlWidget = new GlWidget(this, mOctree, mFlightPlanner);
    connect(mControlWidget, SIGNAL(setScanVolume(QVector3D,QVector3D)), mGlWidget, SLOT(update()));
    setCentralWidget(mGlWidget);

    connect(mFlightPlanner, SIGNAL(suggestVisualization()), mGlWidget, SLOT(updateGL()));

    mTimerUpdateStatus = new QTimer();
    mTimerUpdateStatus->setInterval(500);
    connect(mTimerUpdateStatus, SIGNAL(timeout()), SLOT(slotGetStatus()));

    mLogWidget->log("startup finished, ready.");

    slotConnect();
}

Triangulator::~Triangulator()
{
}

void Triangulator::slotSocketDisconnected()
{
    qDebug() << "Triangulator::slotConnectionEnded()";

    mLogWidget->log("connection closed, retrying...");

    mTimerUpdateStatus->stop();

    mTcpSocket->connectToHost("localhost", 12345);
}

void Triangulator::slotSocketConnected()
{
    qDebug() << "Triangulator::slotSocketConnected()";

    mLogWidget->log("connection established.");

    mTimerUpdateStatus->start();
}

void Triangulator::slotExportCloud()
{
    const QString fileName = QFileDialog::getSaveFileName(this, "Save cloud to", QString(), "PLY files (*.ply)");

    if(fileName.isNull()) return;

    if(CloudExporter::savePly(this, mOctree, fileName))
      QMessageBox::information(this, "Cloud export", "Successfully wrote cloud to\n" + fileName, "OK");
    else
        QMessageBox::information(this, "Cloud export", "Failed saving cloud to file\n\n" + fileName, "OK");
}

void Triangulator::addRandomPoint()
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

void Triangulator::keyPressEvent(QKeyEvent* event)
{
    if(event->key() == Qt::Key_Space)
        addRandomPoint();
}

void Triangulator::slotReadSocket()
{
//    qDebug() << "Triangulator::slotReadSocket()";

    mIncomingDataBuffer.append(mTcpSocket->readAll());

    if(mIncomingDataBuffer.size() < 8) return;

    QDataStream stream(mIncomingDataBuffer); // byteArray is const!
    quint32 packetLength;
    stream >> packetLength;

//    qDebug() << "Triangulator::slotReadSocket(): packetLength is" << packetLength << "buffersize is" << mIncomingDataBuffer.size();

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


void Triangulator::processPacket(QByteArray data)
{
//    qDebug() << "Triangulator::processPacket(): processing bytes:" << data.size();

    QDataStream stream(data);

    QString packetType;
    stream >> packetType;

    if(packetType == "points")
    {
        QVector3D scannerPosition;
        QList<QVector3D> pointList;
        quint32 numberOfHitsToReceive;
        stream >> numberOfHitsToReceive;
        stream >> scannerPosition;
        stream >> pointList;

        mLogWidget->log(QString("received %1 points, inserting...").arg(numberOfHitsToReceive));

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

        mLogWidget->log(QString("%1 points, %2 nodes, %3 points added.").arg(mOctree->getNumberOfItems()).arg(mOctree->getNumberOfNodes()).arg(pointList.size()));

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
        QQuaternion rot;

        stream >> mCurrentVehiclePosition;
        stream >> rot;
        stream >> linearVelocity;

        mControlWidget->slotUpdatePose(mCurrentVehiclePosition, rot);
        mControlWidget->slotUpdateDynamics(linearVelocity);
    }
    else if(packetType == "waypoints")
    {
        QList<QVector3D> wayPoints;

        stream >> wayPoints;

        mControlWidget->slotUpdateWayPoints(wayPoints);
    }
    else if(packetType == "waypointreached")
    {
        QVector3D wpt;
        stream >> wpt;

        mLogWidget->log(QString("reached waypoint %1 %2 %3").arg(wpt.x()).arg(wpt.y()).arg(wpt.z()));
    }
    else if(packetType == "message")
    {
        QString text;
        stream >> text;

        mLogWidget->log(QString("rover: ") + text);
    }
    else
    {
        qDebug() << "Triangulator::processPacket(): unknown packetType" << packetType;
        mLogWidget->log("unknown packetType: " + packetType);
        mIncomingDataBuffer.clear();
    }
}

void Triangulator::slotGetStatus()
{
//    mLogWidget->log("asking rover for status.");

    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("getstatus");

    slotSendData(data);
}

void Triangulator::slotWayPointInsert(QString hash, int index, QVector3D wpt)
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("waypointinsert");
    stream << hash;
    stream << (quint32)index;
    stream << wpt;

    mLogWidget->log(QString("inserting waypoint %1 %2 %3 at position %4").arg(wpt.x()).arg(wpt.y()).arg(wpt.z()).arg(index));

    slotSendData(data);
}

void Triangulator::slotWayPointDelete(QString hash, int index)
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("waypointdelete");
    stream << hash;
    stream << (quint32)index;

    mLogWidget->log(QString("deleting waypoint at index %4").arg(index));

    slotSendData(data);
}

void Triangulator::slotSendData(const QByteArray &data)
{
//    qDebug() << "Triangulator::slotSendData():" << data << mTcpSocket->state();
//    qDebug() << "Triangulator::slotSendData():" << mTcpSocket->errorString();

    QByteArray lengthArray;
    QDataStream streamLength(&lengthArray, QIODevice::WriteOnly);
    streamLength << (quint32)(data.length() + sizeof(quint32));

    mTcpSocket->write(lengthArray);
    mTcpSocket->write(data);
}

void Triangulator::slotConnect()
{
    mTcpSocket->connectToHost("localhost", 12345);
}

void Triangulator::slotSocketError(QAbstractSocket::SocketError socketError)
{
    qDebug() << "Triangulator::slotSocketError():" << socketError;

    if(socketError == QAbstractSocket::ConnectionRefusedError)
        QTimer::singleShot(2000, this, SLOT(slotConnect()));
}

const QVector3D& Triangulator::getCurrentVehiclePosition(void) const
{
    return mCurrentVehiclePosition;
}

const QVector3D Triangulator::getNextWayPoint(void) const
{
    return mControlWidget->getNextWayPoint();
}
