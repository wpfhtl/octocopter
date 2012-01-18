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
    Ui::LogPlayer *ui;
    SbfParser* mSbfParser;
    SensorFuser* mSensorFuser;
    QByteArray mDataSbf, mDataLaser;
    qint32 mIndexSbf, mIndexLaser; // our seek positions in the two bytearrays above
    QTimer* mTimerAnimation;

private slots:
    bool slotOpenLogFiles();
    void slotStepForward();
    void slotStepBack();
    void slotPlay();
    void slotPause();

signals:
    void message(const LogImportance& importance, const QString& source, const QString& message);

    void vehiclePose(Pose);
    void scanData(QList<QVector3D>, QVector3D);
    void vehicleStatus(quint32,float,qint16,qint8);
    void gpsStatus(GpsStatusInformation::GpsStatus);
};

#endif // LOGPLAYER_H
