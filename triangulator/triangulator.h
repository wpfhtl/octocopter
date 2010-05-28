#ifndef TRIANGULATOR_H
#define TRIANGULATOR_H

#include <QtCore>
#include <QtGui>
#include <QtNetwork>
#include <QDebug>

#include <QUdpSocket>
#include <QMutex>

#include "glwidget.h"
#include "octree.h"
#include "lidarpoint.h"

//#include <coordinateconverter.h>

class Triangulator : public QMainWindow
{
    Q_OBJECT

private:

    QMutex mMutex;
    bool mContinue;

    QUdpSocket* mUdpSocket;
    QByteArray mIncomingDataBuffer;

    // a container for collected rays, or rather the world coordinates of where they ended
    Octree* mOctree;

//    CoordinateConverter mCoordinateConverter;

    // Cache the scanner position. If it hasn't changed, there's no need to scan again.
//    Ogre::Vector3 mScannerPosition, mScannerPositionPrevious;
//    Ogre::Quaternion mScannerOrientation, mScannerOrientationPrevious;

    GlWidget *mGlWidget;

    void addRandomPoint();
    void keyPressEvent(QKeyEvent* event);

    void processIncomingData();

private slots:
    void slotReadSocket();
    void slotSocketError(QAbstractSocket::SocketError socketError);

public:
    // Laser rotation is always CCW, angleStart < angleStop
    Triangulator(void);
    ~Triangulator();


signals:

public slots:
};

#endif
