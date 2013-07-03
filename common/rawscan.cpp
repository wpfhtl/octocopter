#include "rawscan.h"

RawScan::RawScan()
{
    relativeScannerPose = 0;
    distances = 0;
    numberOfDistances = 0;
    firstUsableDistance = 0;
    timeStampScanMiddleGnss = 0;
    timeStampScanMiddleScanner = 0;
}

RawScan::RawScan(const RawScan& other)
{
    relativeScannerPose = other.relativeScannerPose;
    distances = other.distances;
    distancesSharedPointer = other.distancesSharedPointer;
    numberOfDistances = other.numberOfDistances;
    firstUsableDistance = other.firstUsableDistance;
    timeStampScanMiddleGnss = other.timeStampScanMiddleGnss;
    timeStampScanMiddleScanner = other.timeStampScanMiddleScanner;
}

RawScan::~RawScan()
{
    // don't delete distances, the shared pointer will do it!
}

QString RawScan::toString() const
{
    QString result = QString("Scan TOW LIDAR %1, GNSS %2, firstUsableDistance %3, numberOfDistances %4, distances:\n")
        .arg(timeStampScanMiddleScanner)
	.arg(timeStampScanMiddleGnss)
	.arg(firstUsableDistance)
	.arg(numberOfDistances);

    for(int i=0;i<numberOfDistances;i++)
    {
        result.append(QString::number(distances[i]) + " ");
    }
    
    return result;
}

void RawScan::setDistances(const std::vector<long>* distances, const quint16 firstUsableDistance, const quint16 lastUsableDistance)
{
    this->firstUsableDistance = firstUsableDistance;
    this->distancesSharedPointer = QSharedPointer<quint16>(new quint16[(lastUsableDistance - firstUsableDistance) + 1]);
    this->numberOfDistances = (lastUsableDistance - firstUsableDistance) + 1;
    this->distances = distancesSharedPointer.data();

    // We have to loop, because we're also converting from long to quint16
    quint16 targetIndex = 0;
    for(int i=firstUsableDistance;i<=lastUsableDistance;i++)
    {
        this->distances[targetIndex++] = distances->at(i);
    }
}

