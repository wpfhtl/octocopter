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

void RawScan::setDistances(const std::vector<long>& distances)
{
    // A usual dataset contains 200 1's at the beginning and 200 1's at the end.
    // We RLE-compress the leading 1s and drop the trailing 1s

    // We currently do not write the relativesensorpose every time, because that'd be wasteful.

    quint16 indexFirst = 0;
    while(distances[indexFirst] == 1)
        indexFirst++;

    quint16 indexLast = distances.size()-1;
    while(distances[indexLast] == 1)
        indexLast--;

    this->firstUsableDistance = indexFirst;
    this->numberOfDistances = (indexLast - indexFirst) + 1;
    this->distancesSharedPointer = QSharedPointer<quint16>(new quint16[numberOfDistances]);
    this->distances = distancesSharedPointer.data();

    // We have to loop, because we're also converting from long to quint16
    quint16 targetIndex = 0;
    for(int i=indexFirst;i<=indexLast;i++)
    {
        this->distances[targetIndex++] = distances[i];
    }
}

void RawScan::setDistances(const quint16* distances, const quint16 firstUsableDistance, const quint16 lastUsableDistance)
{
    this->numberOfDistances = (lastUsableDistance - firstUsableDistance) + 1;
    this->firstUsableDistance = firstUsableDistance;
    this->distancesSharedPointer = QSharedPointer<quint16>(new quint16[numberOfDistances]);
    this->distances = distancesSharedPointer.data();

    // We can just copy...
    memcpy(
                (void*)this->distances,
                (void*)distances,
                sizeof(quint16) * numberOfDistances);
}


void RawScan::log(LogFile* const logFile)
{
    // Always write log data in binary format for later replay. Format is:
    //
    // LASER PacketLengthInBytes(quint16) TOW-LIDAR(qint32) TOW-GNSS(qint32) IndexFirst(quint16) N-DISTANCES(quint16)
    //
    // PacketLengthInBytes is ALL bytes of this packet
    // indexFirst denotes the start of usable data (not 1s)

    if(
            numberOfDistances > 1080
            || firstUsableDistance > 1080
            || timeStampScanMiddleGnss < 0
            || timeStampScanMiddleGnss > 604800000
            || timeStampScanMiddleScanner < 0
            || timeStampScanMiddleScanner > 604800000
            )
    {
        qDebug() << __PRETTY_FUNCTION__ << "error, not logging corrupt raw scan:"
                 << "numberOfDistances" << numberOfDistances
                 << "firstUsableDistance" << firstUsableDistance
                 << "timeStampScanMiddleGnss" << timeStampScanMiddleGnss
                 << "timeStampScanMiddleScanner" << timeStampScanMiddleScanner;
        return;
    }

    // Write the total amount of bytes of this scan into the stream
    const quint16 length = 5 // LASER
            + sizeof(quint16) // length at beginning
            + sizeof(qint32)  // RawScan::timeStampScanMiddleScanner
            + sizeof(qint32)  // RawScan::timeStampScanMiddleGnss. Why? Scanner puts the same time here if its NOT connected to Event-Pin!
            + sizeof(quint16) // indexFirst
            + numberOfDistances * sizeof(quint16); // number of bytes for the distance-data

    const QByteArray magic("LASER");

    logFile->write(magic.constData(), magic.size());
    logFile->write((const char*)&length, sizeof(quint16));
    logFile->write((const char*)&timeStampScanMiddleScanner, sizeof(qint32));
    logFile->write((const char*)&timeStampScanMiddleGnss, sizeof(qint32));
    logFile->write((const char*)&firstUsableDistance, sizeof(quint16));

    // Instead of looping through the indices, lets write everything at once.
    logFile->write((const char*)distances, sizeof(quint16) * numberOfDistances);
}
