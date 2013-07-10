#ifndef RAWSCAN_H
#define RAWSCAN_H

#include <QtCore>
#include <QMetaType>

#include "pose.h"
#include "logfile.h"

struct RawScan
{
    QSharedPointer<quint16> distancesSharedPointer;
    quint16* distances; // points to sharedPinter.data() for convenient access using distances[i]
    quint16 numberOfDistances;
    quint16 firstUsableDistance; // At which index was the first suable distance? Oftentimes, the first ~100 distances have a length of 1mm.
    qint32 timeStampScanMiddleGnss;
    qint32 timeStampScanMiddleScanner;
    const QMatrix4x4* relativeScannerPose; // we're NOT owner of this pose!

    RawScan();
    RawScan(const RawScan& other);

    ~RawScan();

    void setDistances(const std::vector<long> &distances);
    void setDistances(const quint16* distances, const quint16 firstUsableDistance, const quint16 lastUsableDistance);

    void log(LogFile* const logFile);

    QString toString() const;
};

Q_DECLARE_METATYPE(RawScan)

#endif // RAWSCAN_H
