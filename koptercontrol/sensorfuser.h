#ifndef SENSORFUSER_H
#define SENSORFUSER_H

#include <QtCore>
#include "pose.h"

class SensorFuser : public QObject
{
    Q_OBJECT
private:
    QFile* mLogFileScanData;  // poses and scanned points in global frame (processed)

    // The scanner's pose relative to the vehicle frame
    const Pose mRelativeScannerPose;

    QVector<quint32> mScanTimestampsFromGps;
    QList<Pose> mSavedPoses;
    QMap<quint32, std::vector<long> > mSavedScans; // these are timestamps from the scanner

    void transformScanData();

public:
    // The pose specifies translation from vehicle frame to the laser source, so the scanner's
    // physical dimensions are ignored completely. Further down, this class receives high-
    // frequency updates of the vehicle's poses. Using the vehicle-frame poses and its static
    // offset defined in this constructor, it can emit scanpoints in world-coordinates.
    SensorFuser(const Pose &pose);

signals:
    void newScannedPoints(const QVector3D& scanPosition, const QVector<QVector3D>&);

private slots:
    void slotLogScannedPoints(const QVector3D& vehiclePosition, const QVector<QVector3D>& points);

public slots:

    // The pose also contains a timestamp (receiver-time) of when that pose was recorded.
    void slotNewVehiclePose(const Pose& pose);

    // When a laserscan is finished, the lidar changes the electrical level on the
    // event-pin of the gps-receiver-board, which then notifies the PC together with
    // the receiver-time (TOW) when this event happened.
    void slotScanFinished(const quint32& timestamp);

    // Used to feed data from the laserscanner
    void slotNewScanData(const quint32& timestampScanner, const std::vector<long>& points);
};

#endif // SENSORFUSER_H
