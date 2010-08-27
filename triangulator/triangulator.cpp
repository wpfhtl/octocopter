#include "triangulator.h"

Triangulator::Triangulator() : QMainWindow()

{
    qDebug() << "Triangulator::Triangulator()";
    mOctree = new Octree(
            QVector3D(-100, -100, -100), // min
            QVector3D(100, 100, 100),  // max
            10);

    mUdpSocketPoints = new QUdpSocket(this);
    mUdpSocketPoints->bind(QHostAddress::Broadcast, 45454);
    connect(mUdpSocketPoints, SIGNAL(readyRead()), SLOT(slotReadSocketPoints()));
    connect(mUdpSocketPoints, SIGNAL(error(QAbstractSocket::SocketError)), SLOT(slotSocketPointsError(QAbstractSocket::SocketError)));

    mUdpSocketImages = new QUdpSocket(this);
    mUdpSocketImages->bind(11111, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint/*, QHostAddress::Broadcast, 11111*/);
    connect(mUdpSocketImages, SIGNAL(readyRead()), SLOT(slotReadSocketImages()));
    connect(mUdpSocketImages, SIGNAL(error(QAbstractSocket::SocketError)), SLOT(slotSocketImagesError(QAbstractSocket::SocketError)));

    mTcpServer = new QTcpServer(this);
    connect(mTcpServer, SIGNAL(newConnection()), SLOT(slotNewConnection()));
    mTcpServer->listen(QHostAddress::Any, 11111);

    menuBar()->addAction("Save Cloud", this, SLOT(slotExportCloud()));

    mGlWidget = new GlWidget(this, mOctree);
    setCentralWidget(mGlWidget);

    statusBar()->showMessage("ready.");
}

Triangulator::~Triangulator()
{
}

void Triangulator::slotExportCloud()
{
    const QString fileName = QFileDialog::getSaveFileName(this, "Save cloud to", QString(), "PLY files (*.ply)");

    if(fileName.isNull()) return;

    if(CloudExporter::savePly(mOctree, fileName))
      QMessageBox::information(this, "Cloud export", "", "Export successful");
    else
      QMessageBox::information(this, "Cloud export", "", "Export failed, couldn't write to " + fileName);
}

void Triangulator::slotNewConnection()
{
    mConnection = mTcpServer->nextPendingConnection();
    connect(mConnection, SIGNAL(disconnected()), mConnection, SLOT(deleteLater()));
    connect(mConnection, SIGNAL(readyRead()), SLOT(slotReadSocketImages()));

//    mConnection->write(block);
//    mConnection->disconnectFromHost();
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
}

void Triangulator::keyPressEvent(QKeyEvent* event)
{
    if(event->key() == Qt::Key_Space)
        addRandomPoint();
}

void Triangulator::slotReadSocketPoints()
{
    qDebug() << "Triangulator::slotReadSocketPoints()";

    while(mUdpSocketPoints->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(mUdpSocketPoints->pendingDatagramSize());

        mUdpSocketPoints->readDatagram(datagram.data(), datagram.size());
        mIncomingDataBufferPoints.append(datagram);
        qDebug() << "Triangulator::slotReadSocketPoints(): appending" << datagram.size() << "bytes to incoming-buffer, which now has" << mIncomingDataBufferPoints.size() << "bytes";
    }

    processIncomingPoints();
}


void Triangulator::processIncomingPoints()
{
    QDataStream stream(mIncomingDataBufferPoints);
    QVector3D scannerPosition;
    QList<QVector3D> pointList;
    qint32 numberOfHitsToReceive;
    stream >> numberOfHitsToReceive;
    stream >> scannerPosition;
    stream >> pointList;

    // Why does tha data have twice the calculated size?
    int numberOfBytesToReceive = 2 * (sizeof(qint32) + (numberOfHitsToReceive + 1) * sizeof(QVector3D));

//    Q_ASSERT(numberOfHitsToReceive == pointList.size());

    foreach(const QVector3D &p, pointList)
    {
        mOctree->insertPoint(new LidarPoint(p, (p-scannerPosition).normalized(), (p-scannerPosition).lengthSquared()));
    }

    mIncomingDataBufferPoints.remove(0, numberOfBytesToReceive);

    mGlWidget->update();

    statusBar()->showMessage(QString("%1 points, %2 nodes, %3 points added.").arg(mOctree->root()->mNumberOfItems).arg(mOctree->root()->mNumberOfNodes).arg(pointList.size()));

    qDebug() << "appended" << pointList.size() << "points to octree.";
}


void Triangulator::slotReadSocketImages()
{
    qDebug() << "Triangulator::slotReadSocketImages()";

    mIncomingDataBufferImages.append(mConnection->readAll());
    qDebug() << "Triangulator::slotReadSocketImages(): incoming-buffer now has" << mIncomingDataBufferImages.size() << "bytes";

    if(mIncomingDataBufferImages.size())
        processIncomingImages();
}

void Triangulator::processIncomingImages()
{
    QDataStream stream(mIncomingDataBufferImages); // byteArray is const!
    quint32 packetLength;
    stream >> packetLength;

    qDebug() << "ScanReceiver::processIncomingImages(): packetLength is" << packetLength << "buffersize is" << mIncomingDataBufferImages.size();

    if(mIncomingDataBufferImages.size() < packetLength)
    {
        qDebug() << "buffersize is" << mIncomingDataBufferImages.size() << "thats not enough.";
        return;
    }

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

    // remove the first packetLength bytes from the incoming buffer
    mIncomingDataBufferImages.remove(0, packetLength);

    // clear the image buffer array?

    slotReadSocketImages();
}

void Triangulator::slotSocketPointsError(QAbstractSocket::SocketError socketError)
{
    qDebug() << "Triangulator::slotSocketPointsError():" << socketError;
}

void Triangulator::slotSocketImagesError(QAbstractSocket::SocketError socketError)
{
    qDebug() << "Triangulator::slotSocketImagesError():" << socketError;
}
