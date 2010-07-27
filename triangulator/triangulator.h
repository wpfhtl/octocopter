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
#include "camerawindow.h"

class Triangulator : public QMainWindow
{
    Q_OBJECT

private:
    QUdpSocket* mUdpSocketPoints;
    QUdpSocket* mUdpSocketImages;

    QByteArray mIncomingDataBufferPoints, mIncomingDataBufferImages;

    // a container for collected rays, or rather the world coordinates of where they ended
    Octree* mOctree;

    GlWidget *mGlWidget;

    void addRandomPoint();
    void keyPressEvent(QKeyEvent* event);

    void processIncomingPoints();
    void processIncomingImages();

    QMap<QString, CameraWindow*> mCameraWindows;

private slots:
    void slotReadSocketPoints();
    void slotReadSocketImages();
    void slotSocketPointsError(QAbstractSocket::SocketError socketError);
    void slotSocketImagesError(QAbstractSocket::SocketError socketError);

public:
    // Laser rotation is always CCW, angleStart < angleStop
    Triangulator(void);
    ~Triangulator();


signals:

public slots:
};

#endif
