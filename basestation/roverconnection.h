#ifndef ROVERCONNECTION_H
#define ROVERCONNECTION_H

#include <QObject>
#include <QtNetwork>
#include <common.h>
#include <flightstate.h>
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

    // When we get a packet indicating that the connection is alive, we re-start this timer,
    // which will switch to failure after no packet arrived for some seconds
    QTimer mTimerConnectionWatchdog;

    // This class keeps instances of objects that are updated from the rover. After they are,
    // we simply emit pointers to this data.
    GnssStatus mGnssStatus;
    FlightControllerValues mFlightControllerValues;
    FlightState mFlightState;
    Pose mPose;
    QVector<QVector3D> mRegisteredPoints;
    QVector3D mScannerPosition;
    VehicleStatus mVehicleStatus;

    void processPacket(QByteArray data);

public:
    RoverConnection(const QString& hostName, const quint16& port, QObject* parent = 0);

    const FlightControllerValues* const getFlightControllerValues() {return &mFlightControllerValues;}

signals:
    void vehiclePose(const Pose* const);
    void connectionStatusRover(const bool& connected);

    void message(const LogImportance& importance, const QString& source, const QString& message);

    void scanData(const QVector<QVector3D>* const pointList, const QVector3D* const scannerPosition);
    void image(const QString& cameraName, const QSize& imageSize, const Pose& cameraPose, const QByteArray& imageData);
    void vehicleStatus(const VehicleStatus* const);
    void gnssStatus(const GnssStatus* const);
    void flightControllerValues(const FlightControllerValues* const fcv);
    void flightControllerWeightsChanged();
    void flightState(const FlightState* const);

    void wayPointsHashFromRover(const QString& hash);
    void wayPointReachedByRover(const WayPoint& wpt);
    void wayPointInsertedByRover(const quint16& index, const WayPoint& wpt);

public slots:
    void slotConnectToRover(void);

    void slotSendControllerWeights(QString name, QMap<QString,float> weights);

    void slotRoverWayPointInsert(const quint16&, const WayPoint&);
    void slotRoverWayPointDelete(const quint16&);
    void slotRoverWayPointsSet(const QList<WayPoint> *const);

    void slotSendDiffCorrToRover(const QByteArray& diffcorr);

    void slotSendMotionToKopter(const MotionCommand* const mc);

private slots:
    void slotSocketConnected(void);
    void slotSocketDisconnected(void);
    void slotEmitConnectionTimedOut(void);
    void slotReadSocket(void);
    void slotSocketError(QAbstractSocket::SocketError socketError);
    void slotSendData(const QByteArray &data);

};

#endif // ROVERCONNECTION_H
