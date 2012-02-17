#ifndef LOGPLAYER_H
#define LOGPLAYER_H

#include <QDockWidget>
#include <QMessageBox>
#include <QTimer>
#include <QFileDialog>
#include <QString>
#include <QByteArray>

#include "sbfparser.h"
#include "sensorfuser.h"

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
    //enum Direction { Direction_Forward, Direction_Backward };
    enum DataSource {DataSource_SBF, DataSource_Laser };

    Ui::LogPlayer *ui;
    SbfParser* mSbfParser;
    SensorFuser* mSensorFuser;
    QByteArray mDataSbf, mDataLaser;
    qint32 mIndexSbf; // always points to 1) a "$" from the "$@" sbf-sync marker or 2) behind the QByteArray's last byte
    qint32 mIndexLaser; // points to either 1) the beginning, 2) a byte after a \n or 3) behind the QByteArray's last byte (which should equal 2))
    QTimer* mTimerAnimation;

    qint32 getEarliestValidTow(const qint32& towA, const qint32& towB) const;

    // Retrieves the next valid packet from the private data, or an empty packet if that
    QByteArray getPacket(const LogPlayer::DataSource& source);
    qint32 getPacketTow(const LogPlayer::DataSource& source);
    void processLaserData(const QByteArray& packetLaser);

private slots:
    void slotLaserScannerRelativePoseChanged();

    bool slotOpenLogFiles();
    bool slotStepForward();
    void slotRewind();
    void slotPlay();
    void slotPause();

signals:
    void message(const LogImportance& importance, const QString& source, const QString& message);

    void vehiclePose(Pose);
    void scanData(QVector<QVector3D>,QVector3D);
    void vehicleStatus(quint32,float,qint16,qint8);
    void gpsStatus(GpsStatusInformation::GpsStatus);
};

#endif // LOGPLAYER_H
