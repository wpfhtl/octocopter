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
#include <vehiclestatus.h>
#include <gnssstatus.h>
#include <flightcontrollervalues.h>
#include <pose.h>

class Simulator;
class MotionCommand;
class FlightStateRestriction;

class BaseConnection : public QObject
{
    Q_OBJECT

private:
    mutable QMutex mMutex;

    QTcpSocket* mTcpSocket;
    QTcpServer* mTcpServer;
    QByteArray mIncomingDataBuffer, mOutgoingDataBuffer;

    // Data is sent from base into these members, then pointers to them are emitted
    MotionCommand mMotionCommand;
    QString mControllerName;
    QMap<QChar,float> mControllerWeights;
    QByteArray mDifferentialCorrections;

    void processPacket(QByteArray packet);

private slots:
    void slotNewConnection(void);
    void slotConnectionEnded(void);
    void slotReadSocket(bool lockMutex = true);
    void slotSocketError(QAbstractSocket::SocketError socketError);

    void slotSendPingReply();

    // the internalCall parameter skips locking of the mutex, which would lead to a deadlock if
    // slotSendData was called from a class-internal method that already locked the mutex.
    void slotSendData(const QByteArray &data, bool lockMutex = true);
    void slotFlushWriteQueue(void);

public:
    BaseConnection(const QString& interface);
    ~BaseConnection();

signals:
    // emitted when a new list of waypoints has arrived from the basestation
    void wayPoints(const QList<WayPoint>& wayPoints, const WayPointListSource);

    // emitted when the basestation wants to send direct ExternalControl motion commands, which is only used for testing.
    void motion(const MotionCommand* const mc);

    // enable/disable laserscanners
    void scannerState(const bool enable);

    // emitted to indicate saturation of connection. Idea is to send less
    // lidar-data when link is saturated. TODO: combine with RSSI?
    void networkSaturationChanged(const quint8& percentage);

    void differentialCorrections(const QByteArray* const diffcorr);

    void controllerWeights(const QString* const name, const QMap<QChar,float>* const weights);

    // can be used to send a newly connected basestation information that would otherwise
    // take a long time to come in, e.g. flightState
    void newConnection();

public slots:
    // called by rover when it has reached a waypoint, notifies basestation
    void slotSendWayPointReached(const WayPoint&wpt);

    // called by flightcontroller when waypoints are changed
    void slotSendWayPoints(const QList<WayPoint> *const, const WayPointListSource source);

    // called by rover to send updated pose to basestation (called frequently)
    void slotSendVehiclePose(const Pose* const pose);

    // called by rover when the flightstate changes
    void slotSendFlightState(const FlightState* const fs);

    // called by rover when the flightstaterestriction changes
    void slotSendFlightStateRestriction(const FlightStateRestriction* const fsr);

    // called by rover when flightcontroller's pidcontroller-weights have been changed (due to a request from basestation)
    void slotSendFlightControllerWeights();

    // called by rover to send lidarpoints (float4!) to the basestation
    // in the simulator, send the data instead, because the laserscanner lives in another thread
    void slotSendScanFused(const QVector<QVector4D>& points, const QVector3D& scannerPosition);
    // for the rover, its fine to send a pointer, because sender and receiver live in the same thread
    void slotSendScanFused(const float* const points, const quint32 numPoints, const QVector3D* const scannerPosition);

    // called by rover to send new vehicle status to basestation
    void slotSendVehicleStatus(const VehicleStatus* const vs);

    // called by rover to send new gnss status to basestation
    void slotSendGnssStatus(const GnssStatus* const gs);

    // called by flightcontroller to send its output to basestation for debugging purposes
    void slotSendFlightControllerValues(const FlightControllerValues* const fcv);

    // called by rover to send new image to basestation
    void slotSendCameraImage(const QString& name, const QSize& imageSize, const Pose& pose, const QByteArray* image);

    // called by rover to send new log message to basestation
    void slotSendLogMessage(const LogImportance& importance, const QString& source, const QString& text);
};

#endif
