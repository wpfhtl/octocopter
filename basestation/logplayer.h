#ifndef LOGPLAYER_H
#define LOGPLAYER_H

#include <QDockWidget>
#include <QMessageBox>
#include <QTimer>
#include <QFileDialog>
#include <QString>
#include <QByteArray>

#include <sbfparser.h>
#include <gnssstatus.h>
#include <vehiclestatus.h>
#include <flightcontrollervalues.h>
#include <sensorfuser.h>

namespace Ui {
    class LogPlayer;
}

class LogPlayer : public QDockWidget
{
    Q_OBJECT

public:
    LogPlayer(QWidget *parent = 0);
    ~LogPlayer();

private:

    enum DataSource
    {
        Source_Sbf,
        Source_Laser,
        Source_FlightController,
        Source_Invalid
    };

    Ui::LogPlayer *ui;
    SbfParser* mSbfParser;
    SensorFuser* mSensorFuser;
    QByteArray mDataSbf, mDataLaser, mDataSbfCopy, mDataFlightController; // We don't copy the laser array, as it can be several hundred megabytes in size!
    qint32 mIndexLaser; // points to either 1) the beginning, 2) a byte after a \n or 3) behind the QByteArray's last byte (which should equal 2))
    qint32 mIndexFlightController; // points to either 1) the beginning, 2) a byte after a \n or 3) behind the QByteArray's last byte (which should equal 2))

    // Stuff needed for realtime-playback
    QTimer* mTimerAnimation;
    QTime mTimePlaybackStartReal;
    qint32 mTimePlaybackStartTow;

    // Retrieves the next valid packet from the private data, or an empty packet if that
    QByteArray getNextPacket(const DataSource& source);

    // Returns the TOW of the next Laser-Packet/Line, or -1 when there's no more packet available.
    qint32 getNextTow(const DataSource& source);
    qint32 getLastTow(const DataSource& source);

    DataSource getNextDataSource(qint32* tow = 0);

    qint32 getLastTowSbf();

    void processPacket(const LogPlayer::DataSource& source, const QByteArray& packetLaser);

    // This class keeps instances of objects that are updated from the logfiles. After they are,
    // we simply emit pointers to this data.
    GnssStatus mGnssStatus;
    FlightControllerValues mFlightControllerValues;
    Pose mPose; // is also within FlightControllerValues!?
    QVector<QVector3D> mRegisteredPoints;
    QVector3D mScannerPosition;
    VehicleStatus mVehicleStatus;

private slots:
    void slotLaserScannerRelativePoseChanged();
    void slotNewSbfTime(QByteArray,qint32 tow);

    bool slotOpenLogFiles();
    bool slotStepForward();
    void slotRewind();
    void slotPlay();

signals:
    void message(const LogImportance& importance, const QString& source, const QString& message);

    void vehiclePose(const Pose* const);
    void scanData(const QVector<QVector3D>* const, const QVector3D* const);
    void vehicleStatus(const VehicleStatus* const);
    void gnssStatus(const GnssStatus* const);
    void flightControllerValues(const FlightControllerValues* const);
};

#endif // LOGPLAYER_H
