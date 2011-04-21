#ifndef BASECONNECTION_H
#define BASECONNECTION_H

#include <QtCore>
#include <QtGui>
#include <QtNetwork>
#include <QDebug>
#include <QMutex>

#include <waypoint.h>
#include <lidarpoint.h>
#include <pose.h>

class BaseConnection : public QObject
{
    Q_OBJECT

private:
    mutable QMutex mMutex;
    QTcpSocket* mTcpSocket;
    QTcpServer* mTcpServer;
    QByteArray mIncomingDataBuffer, mOutgoingDataBuffer;

    void processPacket(QByteArray packet);

private slots:
    void slotNewConnection(void);
    void slotConnectionEnded(void);
    void slotReadSocket(bool lockMutex = true);
    void slotSocketError(QAbstractSocket::SocketError socketError);

    // the internalCall parameter skips locking of the mutex, which would lead to a deadlock if
    // slotSendData was called from a class-internal method that already locked the mutex.
    void slotSendData(const QByteArray &data, bool lockMutex = true);
    void slotFlushWriteQueue(void);

public:
    BaseConnection(QObject* parent);
    ~BaseConnection();

    enum Importance
    {
        Log,
        Warning,
        Error,
        Desaster
    };

signals:
    // emitted when a new list of waypoints has arrived from the basestation
    void newWayPointListFromBase(QVector<WayPoint> wayPoints);

    // emitted when the basestation wants the kopter to freeze in mid-air
    // (i.e. hold the position, not stop the motors)
    void holdPosition();

public slots:
    // called when the rover has changed the waypoints list, will be sent to base
    void slotNewWayPointsFromRover(const QVector<WayPoint>& wayPoints);

    // called by rover when it has reached a waypoint, notifies basestation
    void slotWayPointReached(const WayPoint&wpt);

    // called by rover to send updated pose to basestation (called frequently)
    void slotPoseChanged(const Pose& pose);

    // called by rover to send lidarpoints to the basestation
    void slotNewLidarPoints(const QVector<LidarPoint>& points);

    // called by rover to send new vehicle status to basestation
    void slotNewVehicleStatus(
        const float& batteryVoltage,
        const float& barometricHeight,
        const float& wirelessRssi
        );

    // called by rover to send new gps status to basestation
    void slotNewGpsStatus(
        const quint8& mode,
        const quint8& info,
        const quint8& error,
        const quint8& numSatellitesTracked,
        const QString& status
        );

    // called by rover to send new log message to basestation
    void slotNewLogMessage(const QString& source, const BaseConnection::Importance& importance, const QString& text);
};

#endif
