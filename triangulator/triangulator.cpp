#include "triangulator.h"

Triangulator::Triangulator() : QMainWindow()

{
    qDebug() << "Triangulator::Triangulator()";
    mOctree = new Octree(
            QVector3D(-100, -100, -100),
            QVector3D(100, 100, 100),
            10);

    mUdpSocket = new QUdpSocket(this);
    mUdpSocket->bind(QHostAddress::Broadcast, 45454);
//    qDebug() << "float:" << sizeof(float);
//    qDebug() << "double:" << sizeof(double);
//    qDebug() << "QVector3D:" << sizeof(QVector3D);
    qDebug() << "pointer:" << sizeof(QVector3D*);
    connect(mUdpSocket, SIGNAL(readyRead()), SLOT(slotReadSocket()));
    connect(mUdpSocket, SIGNAL(error(QAbstractSocket::SocketError)), SLOT(slotSocketError(QAbstractSocket::SocketError)));

    mGlWidget = new GlWidget(this, mOctree);
    setCentralWidget(mGlWidget);

    // testing coordinate conversion
//    Ogre::Vector3 a(10, 10, 10);
//    Ogre::Vector3 b = mCoordinateConverter.convert(mCoordinateConverter.convert(a));
//    qDebug() << a.x << a.y << a.z;
//    qDebug() << b.x << b.y << b.z;
//    exit(0);
    srand(65423321);
    /*
    QVector3D scannerPosition(0, 0, 0);
    for(int i=0;i<10;i++)
    {
        QVector3D point(rand()%199 - 100, rand()%199 - 100, rand()%199 - 100);
        mOctree->insertPoint(
                new LidarPoint(
                        point,
                        (point-scannerPosition).normalized(),
                        (rand()%20+1)
                        )
                );
    }
    */
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

void Triangulator::slotReadSocket()
{
    qDebug() << "Triangulator::slotReadSocket()";

    while(mUdpSocket->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(mUdpSocket->pendingDatagramSize());

        mUdpSocket->readDatagram(datagram.data(), datagram.size());
        mIncomingDataBuffer.append(datagram);
        qDebug() << "Triangulator::slotReadSocket(): appending" << datagram.size() << "bytes to incoming-buffer, which now has" << mIncomingDataBuffer.size() << "bytes";
    }

    processIncomingData();
}


void Triangulator::processIncomingData()
{
/*
    // Packet format: stream << (qint32)numberOfHits << QVector3D(ScannerPositionForAllHits) << mScanData (lots of QVector3D);
    bool conversionSucceeded;
    qint32 numberOfHitsToReceive = (qint32)mIncomingDataBuffer.left(sizeof(qint32)).toInt(&conversionSucceeded);
    int numberOfBytesToReceive = sizeof(qint32) + (numberOfHitsToReceive + 1) * sizeof(QVector3D);

    qDebug() << "Triangulator::slotReadSocket(): numberOfHitsToReceive" << numberOfHitsToReceive << "numberOfBytesToReceive" << numberOfBytesToReceive;

    if(mUdpSocket->bytesAvailable() < numberOfBytesToReceive || !conversionSucceeded)
    {
        // Not enough data ready.
        qDebug() << "Triangulator::slotReadSocket(): only" << mUdpSocket->bytesAvailable() << "bytes ready, but we need" << numberOfBytesToReceive;
        return;
    }

    // If we readDatagram() with a size restriction smaller than whats pending, the rest will be lost!!!
    Q_ASSERT(mUdpSocket->pendingDatagramSize() == numberOfBytesToReceive);

    QByteArray datagram(numberOfBytesToReceive, 0);
    mUdpSocket->readDatagram(datagram.data(), numberOfBytesToReceive);

    qDebug() << "Triangulator::slotReadSocket():" << datagram.size() << "content:" << datagram;

    QList<QVector3D> pointList;
    QVector3D scannerPosition;
    QDataStream stream(datagram);
    */

    QDataStream stream(mIncomingDataBuffer);
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

    mIncomingDataBuffer.remove(0, numberOfBytesToReceive);

    mGlWidget->update();

    qDebug() << "appended" << pointList.size() << "points to octree.";
}

void Triangulator::slotSocketError(QAbstractSocket::SocketError socketError)
{
    qDebug() << "Triangulator::slotSocketError():" << socketError;
}
