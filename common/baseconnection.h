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
#include <gnssstatusinformation.h>
#include <pose.h>

class Simulator;
class MotionCommand;

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
//    WirelessDevice* mWirelessDevice;

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
    void wayPointInsert(quint16 index, const WayPoint& wayPoint);
    void wayPointDelete(const quint16& index);
    void wayPoints(const QList<WayPoint>& wayPoints);

    // emitted when the basestation wants the kopter to freeze in mid-air
    // (i.e. hold the position, not stop the motors)
    void holdPosition();

    // emitted when the basestation wants to send direct ExternalControl motion commands,
    // which is only used for testing.
    void motion(const quint8& thrust, const qint8& pitch, const qint8& roll, const qint8& yaw, const qint8& height);

    void enableScanning(const bool& enable);

    // emitted to indicate saturation of connection. Idea is to send less
    // lidar-data when link is saturated. TODO: combine with RSSI?
    void networkSaturationChanged(const quint8& percentage);

    void rtkDataReady(const QByteArray&);

    // can be used to send a newly connected basestation information that would otherwise
    // take a long time to come in, e.g. flightState
    void newConnection();

public slots:
    // called when the rover has appended a waypoint, will be sent to base
    void slotRoverWayPointInserted(const quint16& index, const WayPoint& wayPoint);

    // called by rover when it has reached a waypoint, notifies basestation
    void slotWayPointReached(const WayPoint&wpt);

    // called by flightcontroller when waypoints are changed (by basestation, just to compare afterwards)
    void slotFlightControllerWayPointsChanged(const QList<WayPoint>&);

    // called by rover to send updated pose to basestation (called frequently)
    void slotNewVehiclePose(const Pose& pose);

    // called by rover when the flightstate changes
    void slotFlightStateChanged(FlightState);

    // called by rover to send lidarpoints to the basestation
    void slotNewScannedPoints(const QVector<QVector3D>& points, const QVector3D& scanPosition);

    // called by rover to send new vehicle status to basestation
    void slotNewVehicleStatus(
        const quint32& missionRunTime, // milliseconds
        const qint16& barometricHeight,
        const float& batteryVoltage
        );

    // called by rover to send new gnss status to basestation
    void slotNewGnssStatus(const GnssStatusInformation::GnssStatus&);

    // called by flightcontroller to send its output to basestation for debugging purposes
    void slotNewFlightControllerValues(const MotionCommand& mc, const Pose& pose, const WayPoint& wpt);

    // called by rover to send new image to basestation
    void slotNewCameraImage(const QString& name, const QSize& imageSize, const Pose& pose, const QByteArray* image);

    // called by rover to send new log message to basestation
    void slotNewLogMessage(const LogImportance& importance, const QString& source, const QString& text);
};

#endif
