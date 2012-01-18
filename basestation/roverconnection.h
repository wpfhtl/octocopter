#ifndef ROVERCONNECTION_H
#define ROVERCONNECTION_H

#include <QObject>
#include <QtNetwork>
#include <common.h>
#include <gpsstatusinformation.h>
#include <waypoint.h>
#include <pose.h>

class RoverConnection : public QObject
{
    Q_OBJECT
private:
    QString mHostName;
    quint16 mPort;
    QTcpSocket* mTcpSocket;
    QByteArray mIncomingDataBuffer;

    // When we get a packet indicating that the connection is alive, we re-start this timer,
    // which will switch to failure after no packet arrived for some seconds
    QTimer mTimerConnectionWatchdog;

    void processPacket(QByteArray data);

public:
    RoverConnection(const QString& hostName, const quint16& port, QObject* parent = 0);

signals:
    void vehiclePose(Pose);
    void connectionStatusRover(const bool& connected);

    void message(const LogImportance& importance, const QString& source, const QString& message);

    void scanData(const QVector3D& scannerPosition, const QList<QVector3D>& pointList);
    void image(const QString& cameraName, const QSize& imageSize, const Pose& cameraPose, const QByteArray& imageData);
    void vehicleStatus(const quint32& missionRunTime, const float& batteryVoltage, const qint16& barometricHeight, const qint8& wirelessRssi);
    void gpsStatus(GpsStatusInformation::GpsStatus);
    void controllerValues(QVector<float>);

    void wayPointsHashFromRover(const QString& hash);
    void wayPointReachedByRover(const WayPoint& wpt);
    void wayPointInsertedByRover(const quint16& index, const WayPoint& wpt);

public slots:
    void slotConnectToRover(void);

    void slotRoverWayPointInsert(const quint16&, const WayPoint&);
    void slotRoverWayPointDelete(const quint16&);
    void slotRoverWayPointsSet(const QList<WayPoint>&);

    void slotSendRtkDataToRover(const QByteArray& rtkData);

private slots:
    void slotSocketConnected(void);
    void slotSocketDisconnected(void);
    void slotEmitConnectionTimedOut(void);
    void slotReadSocket(void);
    void slotSocketError(QAbstractSocket::SocketError socketError);
    void slotSendData(const QByteArray &data);

};

#endif // ROVERCONNECTION_H
