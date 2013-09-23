#ifndef ALMANAC_H
#define ALMANAC_H

#include <QObject>
#include <QString>
#include <QStringList>
#include <QNetworkAccessManager>
#include "gnsstime.h"
#include "satellite.h"
#include "common.h"

/*
 * I use this class to get the angles (azimuth and elevation) to satellites from the
 * vehicle's current position. This is useful e.g. for mission planning, where the
 * pointcloud is used to build a local horizon. From this, e.g. DOP can be computed.
*/

class Almanac : public QObject
{
    Q_OBJECT
public:


    PositionGeodetic mReceiverPosition;

    explicit Almanac(QObject *parent = 0);

    bool addTleAlmanac(const QString& twoLineElmentsUrl, const GnssConstellation& constellation);

    // Specify time of week in seconds,
    void setTime(const float tow) {mTow = tow;}
    // Specify receiver position
    void setReceiverPosition(const PositionGeodetic& pos)
    {
        mReceiverPosition = pos;
        slotComputeOrbits();
    }

    PositionGeodetic getReceiverPosition() const {return mReceiverPosition;}

    QList<Satellite>* getSatellites() {return &mSatellites;}

protected:
    QNetworkAccessManager *mNetworkAccessManager;

    float mTow;
    QList<Satellite> mSatellites;

signals:
    void dataChanged();

public slots:
    void slotComputeOrbits();
    void slotClear();

private slots:
    void slotAlmanacReceived(QNetworkReply* reply);
};

#endif // ALMANAC_H
