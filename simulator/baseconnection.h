#ifndef BASECONNECTION_H
#define BASECONNECTION_H

#include <QtCore>
#include <QtGui>
#include <QtNetwork>
#include <QDebug>
#include <QMutex>

#include "vehicle.h"
#include "simulator.h"
#include "flightcontroller.h"

class BaseConnection : public QObject
{
    Q_OBJECT

private:
    mutable QMutex mMutex;
    QTcpSocket* mTcpSocket;
    QTcpServer* mTcpServer;
    QByteArray mIncomingDataBuffer, mOutgoingDataBuffer;

    Simulator* mSimulator;
    Vehicle* mVehicle;
    FlightController* mFlightController;

    void processPacket(QByteArray packet);

private slots:
    void slotNewConnection(void);
    void slotConnectionEnded(void);
    void slotReadSocket(bool lockMutex = true);
    void slotSocketError(QAbstractSocket::SocketError socketError);

    // FlightController emits a signal calling this slot.
    void slotCurrentWayPointsChanged(QList<QVector3D> wayPoints);

public:
    BaseConnection(Simulator* simulator);
    ~BaseConnection();

    void setVehicle(Vehicle* vehicle);

signals:

public slots:
    // the internalCall parameter skips locking of the mutex, which would lead to a deadlock if
    // slotSendData was called from a class-internal method that already locked the mutex.
    void slotSendData(const QByteArray &data, bool lockMutex = true);
    void slotFlushWriteQueue(void);

    // called when a waypoint has been reached, notifies basestation
    void slotWayPointReached(QVector3D wpt);
};

#endif
