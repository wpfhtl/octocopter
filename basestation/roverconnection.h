#ifndef ROVERCONNECTION_H
#define ROVERCONNECTION_H

#include <QObject>
#include <QtNetwork>
#include <common.h>
#include <flightstate.h>
#include <flightstaterestriction.h>
#include <gnssstatus.h>
#include <flightcontrollervalues.h>
#include <waypoint.h>
#include <pose.h>
#include <motioncommand.h>
#include <vehiclestatus.h>

class RoverConnection : public QObject
{
    Q_OBJECT
private:
    QString mHostName;
    quint16 mPort;
    QTcpSocket* mTcpSocket;
    QByteArray mIncomingDataBuffer;

    // When we haven't received a packet for quite a while (usual e.g. in simulator),
    // we send a ping request to confirm network is still alive. This is for bookkeeping.
    bool mCurrentlyWaitingForPingReply;

    // When we get a packet indicating that the connection is alive, we re-start this timer,
    // which will switch to failure after no packet arrived for some seconds
    QTimer mTimerConnectionWatchdog;
    QTime mTimeOfLastPacket;

    // This class keeps instances of objects that are updated from the rover. After they are,
    // we simply emit pointers to this data.
    GnssStatus mGnssStatus;
    FlightControllerValues mFlightControllerValues;
    FlightState mFlightState;
    FlightStateRestriction mFlightStateRestriction;
    Pose mPose;
    QList<WayPoint> mWayPointList;
    float* mRegisteredPointsFloat;
    QVector3D mScannerPosition;
    VehicleStatus mVehicleStatus;

    void processPacket(QByteArray data);

public:
    RoverConnection(const QString& hostName, const quint16& port, QObject* parent = 0);
    ~RoverConnection();

    const FlightControllerValues* const getFlightControllerValues() {return &mFlightControllerValues;}

signals:
    void vehiclePose(const Pose* const);
    void connectionStatusRover(const bool& connected);

    void message(const LogImportance& importance, const QString& source, const QString& message);

    void scanData(const float* const points, const quint32& count, const QVector3D* const scannerPosition);
    void image(const QString& cameraName, const QSize& imageSize, const Pose& cameraPose, const QByteArray& imageData);
    void vehicleStatus(const VehicleStatus* const);
    void gnssStatus(const GnssStatus* const);
    void flightControllerValues(const FlightControllerValues* const fcv);
    void flightControllerWeightsChanged();
    void flightState(const FlightState* const);
    void flightStateRestriction(const FlightStateRestriction* const);

    // When the rover has reached a waypoint
    void wayPointReachedByRover(const WayPoint& wpt);

    // When the rover has updated the list by itself
    void wayPoints(const QList<WayPoint>* const, const WayPointListSource);

public slots:
    void slotConnectToRover(void);

    void slotSendControllerWeights(QString name, QMap<QChar,float> weights);

    void slotSendScannerState(const bool enabled);

    // To set waypoints on rover
    void slotSetWayPoints(const QList<WayPoint>* const wayPointList, const WayPointListSource source);

    void slotSendDiffCorrToRover(const QByteArray& diffcorr);

    void slotSendMotionToKopter(const MotionCommand* const mc);
    void slotSendPingRequest();

private slots:
    void slotSocketConnected(void);
    void slotSocketDisconnected(void);
    void slotWatchdogTimerFired(void);
    void slotReadSocket(void);
    void slotSocketError(QAbstractSocket::SocketError socketError);
    void slotSendData(const QByteArray &data);

};

#endif // ROVERCONNECTION_H
