#ifndef BASECONNECTION_H
#define BASECONNECTION_H

#include <QtCore>
#include <QtGui>
#include <QtNetwork>
#include <QDebug>
#include <QMutex>

#include <common.h>
#include <waypoint.h>
#include <lidarpoint.h>
#include <wirelessdevice.h>
#include <pose.h>

class Simulator;

class BaseConnection : public QObject
{
    Q_OBJECT

private:
    mutable QMutex mMutex;
//    int mSockfd; // socket for reading RSSI
//    QString mInterface; // used only for RSSI reading in case of WLAN connection
    QTcpSocket* mTcpSocket;
    QTcpServer* mTcpServer;
    QByteArray mIncomingDataBuffer, mOutgoingDataBuffer;
    WirelessDevice* mWirelessDevice;

    void processPacket(QByteArray packet);

//    qint8 getRssi();

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
    BaseConnection(const QString& interface);
    ~BaseConnection();

signals:
    // emitted when a new list of waypoints has arrived from the basestation
//    void newWayPointListFromBase(QVector<WayPoint> wayPoints);
    void wayPointInsert(const QString& hash, quint16 index, const QList<WayPoint>& wayPoints);
    void wayPointDelete(const QString& hash, quint16 index);

    // emitted when the basestation wants the kopter to freeze in mid-air
    // (i.e. hold the position, not stop the motors)
    void holdPosition();

    void enableScanning(const bool& enable);

    // emitted to indicate saturation of connection. Idea is to send less
    // lidar-data when link is saturated. TODO: combine with RSSI?
    void networkSaturationChanged(const quint8& percentage);

    void rtkDataReady(const QByteArray&);

    // emitted when the base is requesting status information. Please collect
    // information and call the slotNewVehicleStatus(...) slot.
    // UNUSED, we will alays send statu, no need to request it.
//    void statusRequested();

public slots:
    // called when the rover has changed the waypoints list, will be sent to base
    void slotNewWayPointsFromRover(const QVector<WayPoint>& wayPoints);

    // called by rover when it has reached a waypoint, notifies basestation
    void slotWayPointReached(const WayPoint&wpt);

    // called by rover to send updated pose to basestation (called frequently)
    void slotPoseChanged(const Pose& pose);

    // called by rover to send lidarpoints to the basestation
    void slotNewLidarPoints(const QVector3D& scanPosition, const QVector<QVector3D>& points);

    // called by rover to send new vehicle status to basestation
    void slotNewVehicleStatus(
        const quint32& missionRunTime, // milliseconds
        const qint16& barometricHeight,
        const float& batteryVoltage
        );

    // called by rover to send new gps status to basestation
    void slotNewGpsStatus(
        const quint8& gnssMode,
        const quint8& integrationMode,
        const quint8& info,
        const quint8& error,
        const quint8& numSatellitesTracked,
        const quint8& lastPvtAge,
        const QString& status
        );

    // called by flightcontroller to send its output to basestation for debugging purposes
    void slotNewControllerDebugValues(const Pose& pose, const quint8& thrust, const qint8& pitch, const qint8& roll, const qint8& yaw, const qint8& height);

    // called by rover to send new log message to basestation
    void slotNewLogMessage(const LogImportance& importance, const QString& source, const QString& text);
};

#endif
