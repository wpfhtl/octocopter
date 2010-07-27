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
    mUdpSocketImages->bind(QHostAddress::Broadcast, 11111);
    connect(mUdpSocketImages, SIGNAL(readyRead()), SLOT(slotReadSocketImages()));
    connect(mUdpSocketImages, SIGNAL(error(QAbstractSocket::SocketError)), SLOT(slotSocketImagesError(QAbstractSocket::SocketError)));

    mGlWidget = new GlWidget(this, mOctree);
    setCentralWidget(mGlWidget);
}

Triangulator::~Triangulator()
{
}

void Triangulator::addRandomPoint()
{
    QVector3D scannerPosition(0, 0, 0);
    QVector3D point(rand()%199 - 100, rand()%199 - 100, rand()%199 - 100);
    mOctree->insertPoint(
            new LidarPoint(
                    point,
                    (point-scannerPosition).normalized(),
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

    qDebug() << "appended" << pointList.size() << "points to octree.";
}


void Triangulator::slotReadSocketImages()
{
    qDebug() << "Triangulator::slotReadSocketImages()";

    while(mUdpSocketImages->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(mUdpSocketImages->pendingDatagramSize());

        mUdpSocketImages->readDatagram(datagram.data(), datagram.size());
        mIncomingDataBufferImages.append(datagram);
        qDebug() << "Triangulator::slotReadSocketImages(): appending" << datagram.size() << "bytes to incoming-buffer, which now has" << mIncomingDataBufferImages.size() << "bytes";
    }

    processIncomingImages();
}

void Triangulator::processIncomingImages()
{

    qDebug() << "ScanReceiver::processIncomingImages()";
    // Start streaming!
    QDataStream stream(mIncomingDataBufferImages);

    qint32 packetLength;
    QString cameraName;
    QVector3D cameraPosition;
    QQuaternion cameraOrientation;
    QByteArray image;

    stream >> packetLength;
    stream >> cameraName;
    stream >> cameraPosition;
    stream >> cameraOrientation;
    stream >> image;

    qDebug() << "image bytearray size is" << image.size();

    if(!mCameraWindows.contains(cameraName))
        mCameraWindows.insert(cameraName, new CameraWindow(this));

    CameraWindow* win = mCameraWindows.value(cameraName);

    win->slotSetPixmapData(image);
    win->show();

    // clear the image buffer array?

//    slotReadSocketImages();

/*  This is how we write into the stream
    QByteArray datagram;
    QDataStream stream(&datagram, QIODevice::WriteOnly);

    // Stream camera name
    stream << objectName();

    // Stream the pose
    stream << QVector3D(mCameraNode->_getDerivedPosition().x, mCameraNode->_getDerivedPosition().y, mCameraNode->_getDerivedPosition().z);
    stream << QQuaternion(mCameraNode->_getDerivedOrientation().w, mCameraNode->_getDerivedOrientation().x, mCameraNode->_getDerivedOrientation().y, mCameraNode->_getDerivedOrientation().z);

    // Stream the image
    QByteArray imageArray;
    QBuffer buffer(&imageArray);
    mImage->save(&buffer, "jpg", 90);
    datagram.append(imageArray);

    // Now set the length of the datagram
    QByteArray datagramLengthArray;
    QDataStream streamLength(&datagramLengthArray, QIODevice::WriteOnly);
    streamLength << (qint32)(datagram.length() + sizeof(qint32));
    datagram.prepend(datagramLengthArray);

    mUdpSocket->writeDatagram(datagram, QHostAddress::Broadcast, 11111);
    mUdpSocket->flush();
*/
}

void Triangulator::slotSocketPointsError(QAbstractSocket::SocketError socketError)
{
    qDebug() << "Triangulator::slotSocketPointsError():" << socketError;
}

void Triangulator::slotSocketImagesError(QAbstractSocket::SocketError socketError)
{
    qDebug() << "Triangulator::slotSocketImagesError():" << socketError;
}
