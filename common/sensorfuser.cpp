#include "sensorfuser.h"
#include <unistd.h> // usleep
#include <profiler.h>

SensorFuser::SensorFuser(const quint8& stridePoint, const quint8& strideScan) : QObject()
{
    mStridePoint = stridePoint;
    mStrideScan = strideScan; // TODO: implement

    mFlushRemainingData = false;
    mNewestDataTime = 0;
    mMaximumFusableRayLength = 10.0f;
    mStatsScansDiscarded = 0;
    mLastScanMiddleGnssTow = 0;
    mNumberOfScansWithMissingGnssTimestamps = 0;
    mMaximumTimeOffsetBetweenScannerAndGnss = 12;

    mRegisteredPoints = new float[1080 * 4];
    mNumberOfPointsFusedInThisScan = 0;

    mBestInterpolationMethodToUse = InterpolationMethod::Linear;

    // We need to register one of them, so we can pass pointers in queued signals and slots
    qRegisterMetaType<RawScan>("RawScan");
    qRegisterMetaType<RawScan*>("RawScanPointer");

    mPoseDynamicsLogFile = new QFile("/tmp/posedynamics.dat");
    if(!mPoseDynamicsLogFile->open(QIODevice::WriteOnly | QIODevice::Truncate))
        qFatal("cannot open pose dynamics logfile for writing!");

    mPoseDynamicsStream = new QTextStream(mPoseDynamicsLogFile);
    mPoseDynamicsStream->setRealNumberPrecision(3);
    mPoseDynamicsStream->setRealNumberNotation(QTextStream::FixedNotation);
    *(mPoseDynamicsStream) << "pointIndex\tvehicle-velocity (cm/s)\tvehicle-acceleration (cm/s^2)\tvehicle-rotation (deg/s)\n";
}

SensorFuser::~SensorFuser()
{
    qDebug() << "SensorFuser::~SensorFuser(): total scans fused:"
             << "cubic:"  << mStatsScansFused[InterpolationMethod::Cubic]
             << "linear:" << mStatsScansFused[InterpolationMethod::Linear]
             << "nearestneighbor:" << mStatsScansFused[InterpolationMethod::NearestNeighbor]
             << "discarded:"<< mStatsScansDiscarded
             << "missing gnss stamps:" << mNumberOfScansWithMissingGnssTimestamps;
    qDebug() << "SensorFuser::~SensorFuser(): leftover scanInfos:"  << mRawScans.size() << "scanGnss:" << mGnssTimeStamps.size() << "poses:"<< mPoses.size();

    slotClearData();

    delete mRegisteredPoints;
    mPoseDynamicsStream->flush();
    delete mPoseDynamicsStream;
    mPoseDynamicsLogFile->close();
    delete mPoseDynamicsLogFile;
}

void SensorFuser::slotClearData(const qint32 maximumDataAge)
{
    // Remove all poses older than maximumDataAge
    QMutableListIterator<Pose*> i(mPoses);
    while(i.hasNext())
    {
        const qint32 timestamp = i.next()->timestamp;
        if(timestamp + maximumDataAge < mNewestDataTime || maximumDataAge < 0)
        {
            delete i.value();
            i.remove();
        }
    }

    // Remove all gnss timestamps older than maximumAge
    QMutableListIterator<qint32> j(mGnssTimeStamps);
    while(j.hasNext())
    {
        const qint32 timestamp = j.next();
        if(timestamp + maximumDataAge < mNewestDataTime || maximumDataAge < 0)
        {
            j.remove();
        }
    }

    // Remove all gnss timestamps older than maximumAge
    QMutableListIterator<RawScan*> k(mRawScans);
    while(k.hasNext())
    {
        RawScan* rs = k.next();
        // We check the scanner timestamp, thats precise enough
        if(rs->timeStampScanMiddleScanner + maximumDataAge < mNewestDataTime || maximumDataAge < 0)
        {
            qDebug() << "SensorFuser::slotClearData(): removing old unfused scan from lasertime" << rs->timeStampScanMiddleScanner << "gnsstime" << rs->timeStampScanMiddleGnss;
            delete rs;
            k.remove();
        }
    }

    if(maximumDataAge < 0)
    {
        // Clear everything, not just old data
        mNewestDataTime = 0;
        mLastScanMiddleGnssTow = 0;
    }
}

// TODO: make this method fuse N successive rays
void SensorFuser::fuseRayWithLastInterpolatedPose(const qint16 index, const float& distance)
{
    // Version using QMatrix4x4
    const QVector3D vectorScannerToPoint(
                sin(-0.0043633231299858238686f * (index - 540)) * distance, // X in meters
                0.0f,                                                       // Y always 0
                -cos(0.0043633231299858238686f * (index - 540)) * distance);// Z in meters




    const QVector3D p = mLastInterpolatedPose.getMatrixRef() * vectorScannerToPoint;

    mRegisteredPoints[mNumberOfPointsFusedInThisScan * 4 + 0] = p.x();
    mRegisteredPoints[mNumberOfPointsFusedInThisScan * 4 + 1] = p.y();
    mRegisteredPoints[mNumberOfPointsFusedInThisScan * 4 + 2] = p.z();
    mRegisteredPoints[mNumberOfPointsFusedInThisScan * 4 + 3] = vectorScannerToPoint.lengthSquared(); // squared distance to point
    mNumberOfPointsFusedInThisScan++;
}

// According to mInterpolationmethod, we set each ScanInfo' object 's poses-list to be the 1(nn) / 2(linear) / 3(linear) / 4(cubic) / 5(cubic) best poses to interpolate
void SensorFuser::fuseScans()
{
    Profiler p(__PRETTY_FUNCTION__);

    // Process every scan...
    QMutableListIterator<RawScan*> iteratorRawScans(mRawScans);
    while(iteratorRawScans.hasNext())
    {
        RawScan* rawScan = iteratorRawScans.next();
        //qDebug() << __PRETTY_FUNCTION__ << "now processing raw scan from scanner time" << rawScan->timeStampScanMiddleScanner << "gnss time" << rawScan->timeStampScanMiddleGnss;

        // scanInfo can still contain scans with zero timeStampScanMiddleGnss values because they weren't matched/
        // populated from the mGnssTimeStamps vector. This can have two reasons:
        //
        // a)   the ExtEvent-packet wasn't received yet, so slotScanFinished() wasn't called and thus mGnssTimeStamps
        //      doesn't contain any timestamp that could be matched in matchTimestamps(). In this case, we skip this
        //      scanInfo and try again later.
        //
        // b)   In some logdata, there are no ExtEvents due to misconfiguration or hardware failure (EventA is broken).
        //      This means slotScanFinished() will never be called, so ScanInformation.timeStampScanMiddleGnss will
        //      remain 0 and data fusion cannot work.
        //
        // To fix b), we DO process scans with a timeStampScanMiddleGnss of 0 IF they are so old that we can trust that
        // their respective timeStampGnss will not come in anymore. In this case, we simply use the scanner's timestamp.
        if(rawScan->timeStampScanMiddleGnss == 0)
        {
            if(mNewestDataTime - rawScan->timeStampScanMiddleScanner > 1000 || mFlushRemainingData)
            {
                // the scanInfo is relatively old, we give up waiting for a gnssTimestamp and use the scanner's
                rawScan->timeStampScanMiddleGnss = rawScan->timeStampScanMiddleScanner;
                mNumberOfScansWithMissingGnssTimestamps++;
                //qDebug() << __PRETTY_FUNCTION__ << "scan is old, no gnss time, using scanner time";
            }
            else
            {
                // the scan is still young, skip processing and hope the gnss timestamp still comes.
                //qDebug() << __PRETTY_FUNCTION__ << "scan is young, no gnss time, skipping for now to wait for gnss time";
                continue;
            }
        }

        //qDebug() << "SensorFuser::fuseScans(): trying to fuse scan from" << rawScan->timeStampScanMiddleGnss;

        // Which pose(-index) fits best to this scan?
        qint32 bestFitPoseIndex = -1;
        qint32 bestFitPoseTimeDifference = std::numeric_limits<qint32>::max();
        for(int currentPoseIndex = 0; currentPoseIndex < mPoses.size(); currentPoseIndex++)
        {
            const Pose* const currentPose = mPoses[currentPoseIndex];
            if(abs(currentPose->timestamp - rawScan->timeStampScanMiddleGnss) < abs(bestFitPoseTimeDifference))
            {
                bestFitPoseIndex = currentPoseIndex;

                // negative value means the pose was before the scan
                bestFitPoseTimeDifference = currentPose->timestamp - rawScan->timeStampScanMiddleGnss;
            }
            else
            {
                // the time difference has INcreased, so lets break this loop.
                break;
            }
        }

        //qDebug() << __PRETTY_FUNCTION__ << "best fit pose index" << bestFitPoseIndex << "timediff" << bestFitPoseTimeDifference;

        // If no pose was found, try the next scan.
        if(bestFitPoseTimeDifference == std::numeric_limits<qint32>::max()) continue;

        // Even when interpolating e.g. cubically, the bestFit pose (not the other ones surrounding
        // the scan) should be as close as required for nearest neighbor. If not, skip this scan.
        // TODO: check this!
        if(abs(bestFitPoseTimeDifference) > MaximumFusionTimeOffset::NearestNeighbor)
        {
            //qDebug() << __PRETTY_FUNCTION__ << "scan<->pose time diff is more than" << MaximumFusionTimeOffset::NearestNeighbor << "- skipping";
            continue;
        }

        // If mPoses[bestFitPoseIndex] is the last element, it means that the next element (coming
        // in the future) might be better. So, skip fusion in that case and try again next time.
        if(bestFitPoseIndex >= mPoses.size() - 1 && ! mFlushRemainingData)
        {
            //qDebug() << __PRETTY_FUNCTION__ << "scan fits last pose best. Maybe next pose is better? skipping.";
            continue;
        }

        // Prepare for fusion :)
        mNumberOfPointsFusedInThisScan = 0;

        // Scans are stored with the times their ray was in front, so for linear/cubic interpolation, we need
        // one/two poses before and after each *ray*. Hence, we need one/two poses before scanStartRay (t-9.375ms)
        // and one/two poses after scanEndRay (t+9.375ms). Depending on the temporal shift between scans and
        // poses, we'll need 2-3/4-5 poses for all of a scan's rays. This makes life a lot harder for us.

        // We do not use switch/case or if/else here, because when we find that a scan cannot ever be fused using a better method
        // requiring more poses, we can downgrade the InterpolationMethod and still fuse this scan linearly of even using nn.
        InterpolationMethod im = mBestInterpolationMethodToUse;

        if(im == InterpolationMethod::Cubic)
        {
            // The indexes of the poses to be used for cubic interpolation
            QVector<qint16> poseIndicesToUse;

            // We have the index of the best-fitting pose, but for cubic interpolation, we need 4 or 5 poses.
            // Assemble the indices of the other required poses
            if(bestFitPoseTimeDifference < 0)
            {
                // The best-fitting pose is from BEFORE the scan
                if(bestFitPoseTimeDifference >= -9) // 9ms is duration of half a scan
                {
                    // the scan's first ray was before mPose[bestFitPoseIndex]
                    poseIndicesToUse.append(bestFitPoseIndex - 2);
                    poseIndicesToUse.append(bestFitPoseIndex - 1);
                    poseIndicesToUse.append(bestFitPoseIndex);
                    poseIndicesToUse.append(bestFitPoseIndex + 1);
                    poseIndicesToUse.append(bestFitPoseIndex + 2);
                }
                else
                {
                    // the scan's first ray was after mPose[bestFitPoseIndex]
                    poseIndicesToUse.append(bestFitPoseIndex - 1);
                    poseIndicesToUse.append(bestFitPoseIndex);
                    poseIndicesToUse.append(bestFitPoseIndex + 1);
                    poseIndicesToUse.append(bestFitPoseIndex + 2);
                }
            }
            else // this branch is ok for a value of exactly 0!
            {
                // The best-fitting pose is from AFTER the scan
                if(bestFitPoseTimeDifference <= 9) // 9ms is duration of half a scan
                {
                    // the scan's last ray was after mPose[bestFitPoseIndex]
                    poseIndicesToUse.append(bestFitPoseIndex - 2);
                    poseIndicesToUse.append(bestFitPoseIndex - 1);
                    poseIndicesToUse.append(bestFitPoseIndex);
                    poseIndicesToUse.append(bestFitPoseIndex + 1);
                    poseIndicesToUse.append(bestFitPoseIndex + 2);
                }
                else
                {
                    // the scan's last ray was before mPose[bestFitPoseIndex]
                    poseIndicesToUse.append(bestFitPoseIndex - 2);
                    poseIndicesToUse.append(bestFitPoseIndex - 1);
                    poseIndicesToUse.append(bestFitPoseIndex);
                    poseIndicesToUse.append(bestFitPoseIndex + 1);
                }
            }

            bool allPosesPresent = true;

            // Check for existance and usability of poses until one fails
            for(int i=0;i<poseIndicesToUse.size() && allPosesPresent;i++)
            {
                const qint16& poseIndex = poseIndicesToUse[i];
                if(poseIndex < 0)
                {
                    // The current scan does have a well-fitting pose, but for cubic interpolation, we require one that happened
                    // before the first/oldest pose in mPoses. Thus, we will never be able to fuse this scan cubically.
                    // Downgrade the InterpolationMethod to Linear and see if that works out.
                    allPosesPresent = false;
                    im = InterpolationMethod::Linear;
                }
                else if(poseIndex >= mPoses.size())
                {
                    // We need a pose from the future: just wait for it to come in!
                    allPosesPresent = false;
                    if(mFlushRemainingData) im = InterpolationMethod::Linear;
                }
                else if(abs(mPoses[poseIndex]->timestamp - rawScan->timeStampScanMiddleGnss) > MaximumFusionTimeOffset::Cubic)
                {
                    // mPoses[poseIndex] exists, but it is too far away time-wise. This means there are missing poses,
                    // so we should try degrading the interpolation-method
                    allPosesPresent = false;
                    im = InterpolationMethod::Linear;
                }
            }

            if(allPosesPresent)
            {
                // We finally have all poses required to fuse this scan. Go!
                for(qint16 index=0; index < rawScan->numberOfDistances; index++)
                {
                    // Only process every mStridePoint'th point
                    if(index % mStridePoint != 0) continue;

                    // Skip reflections on vehicle (=closer than 50cm) and long ones (bad platform orientation accuracy)
                    if(rawScan->distances[index] < 700 || rawScan->distances[index] > mMaximumFusableRayLength * 1000.0f) continue;

                    // Convert millimeters to meters.
                    const float distance = rawScan->distances[index] / 1000.0f;

                    const qint32 timeOfCurrentRay = rawScan->timeStampScanMiddleGnss + (qint32)(-9.375f + (((float)index) * 0.01736f));

                    if(mLastInterpolatedPose.timestamp != timeOfCurrentRay)
                    {
                        // TODO: chek this form 20 and 50 Hz poses using yellow paper!
                        if(mPoses[poseIndicesToUse[2]]->timestamp > timeOfCurrentRay)
                        {
                            mLastInterpolatedPose = Pose::interpolateCubic(
                                        mPoses[poseIndicesToUse[0]],
                                    mPoses[poseIndicesToUse[1]],
                                    mPoses[poseIndicesToUse[2]],
                                    mPoses[poseIndicesToUse[3]],
                                    timeOfCurrentRay);
                        }
                        else
                        {
                            mLastInterpolatedPose = Pose::interpolateCubic(
                                        mPoses[poseIndicesToUse[1]],
                                    mPoses[poseIndicesToUse[2]],
                                    mPoses[poseIndicesToUse[3]],
                                    mPoses[poseIndicesToUse[4]],
                                    timeOfCurrentRay);
                        }

                        mLastInterpolatedPose.transform(rawScan->relativeScannerPose);
                        emitLastInterpolatedPose();
                    }

                    fuseRayWithLastInterpolatedPose(index + rawScan->firstUsableDistance, distance);
                    // Skip angles when distance is close
                    if(distance < 1.0f) index += 3; else if(distance < 2.0f) index += 2; else if(distance < 3.0f) index += 1;
                }

                // This scan was successfully fused. Remove it from our vector
                iteratorRawScans.remove();

                mStatsScansFused[InterpolationMethod::Cubic]++;

                mLastScannerPosition = mPoses[poseIndicesToUse[2]]->getPosition();
                emit scanFused(mRegisteredPoints, mNumberOfPointsFusedInThisScan, &mLastScannerPosition);
            }
        }

        if(im == InterpolationMethod::Linear)
        {
            //qDebug() << __PRETTY_FUNCTION__ << "trying to fuse scan linearly...";
            // The indexes of the poses to be used for linear interpolation
            QVector<qint16> poseIndicesToUse;

            // We have the index of the best-fitting pose, but for linear interpolation, we need 2 or 3 poses.
            // Assemble the indices of the other required poses
            if(bestFitPoseTimeDifference < 0)
            {
                // The best-fitting pose is from BEFORE the scan
                if(bestFitPoseTimeDifference >= -9) // 9ms is duration of half a scan
                {
                    // the scan's first ray was before mPose[bestFitPoseIndex]
                    poseIndicesToUse.append(bestFitPoseIndex - 1);
                    poseIndicesToUse.append(bestFitPoseIndex);
                    poseIndicesToUse.append(bestFitPoseIndex + 1);
                }
                else
                {
                    // the scan's first ray was after mPose[bestFitPoseIndex]
                    poseIndicesToUse.append(bestFitPoseIndex);
                    poseIndicesToUse.append(bestFitPoseIndex + 1);
                }
            }
            else // this branch is ok for a value of exactly 0!
            {
                // The best-fitting pose is from AFTER the scan
                if(bestFitPoseTimeDifference <= 9) // 9ms is duration of half a scan
                {
                    // the scan's last ray was after mPose[bestFitPoseIndex]
                    poseIndicesToUse.append(bestFitPoseIndex - 1);
                    poseIndicesToUse.append(bestFitPoseIndex);
                    poseIndicesToUse.append(bestFitPoseIndex + 1);
                }
                else
                {
                    // the scan's last ray was before mPose[bestFitPoseIndex]
                    poseIndicesToUse.append(bestFitPoseIndex - 1);
                    poseIndicesToUse.append(bestFitPoseIndex);
                }
            }

            bool allPosesPresent = true;

            // Check for existance and usability of poses until one fails
            for(int i=0;i<poseIndicesToUse.size() && allPosesPresent;i++)
            {
                const qint16 poseIndex = poseIndicesToUse[i];
                if(poseIndex < 0)
                {
                    // The current scan does have a well-fitting pose, but for linear interpolation, we require one that happened
                    // before the first/oldest pose in mPoses. Thus, we will never be able to fuse this scan linearly.
                    // Downgrade the InterpolationMethod to NearestNeighbor and see if that works out.
                    allPosesPresent = false;
                    im = InterpolationMethod::NearestNeighbor;
                }
                else if(poseIndex >= mPoses.size())
                {
                    // We need a pose from the future: just wait for it to come in!
                    allPosesPresent = false;
                    if(mFlushRemainingData) im = InterpolationMethod::NearestNeighbor;
                }
                else if(abs(mPoses[poseIndex]->timestamp - rawScan->timeStampScanMiddleGnss) > MaximumFusionTimeOffset::Linear)
                {
                    // mPoses[poseIndex] exists, but it is too far away time-wise. This means there are missing poses,
                    // so we should try degrading the interpolation-method
                    allPosesPresent = false;
                    im = InterpolationMethod::NearestNeighbor;
                }
            }

            if(allPosesPresent)
            {
                //qDebug() << __PRETTY_FUNCTION__ << "all poses present, fusing" << rawScan->numberOfDistances << "rays";
                // We finally have all poses required to fuse this scan. Go!
                for(qint16 index=0; index < rawScan->numberOfDistances; index++)
                {
                    // Only process every mStridePoint'th point
                    if(index % mStridePoint != 0) continue;

                    // Skip reflections on vehicle (=closer than 50cm) and long ones (bad platform orientation accuracy)
                    if(rawScan->distances[index] < 700 || rawScan->distances[index] > mMaximumFusableRayLength * 1000.0f) continue;

                    // Convert millimeters to meters.
                    const float distance = rawScan->distances[index] / 1000.0f;

                    const qint32 timeOfCurrentRay = rawScan->timeStampScanMiddleGnss + (qint32)(-9.375f + (((float)index) * 0.01736f));

                    if(mLastInterpolatedPose.timestamp != timeOfCurrentRay)
                    {
                        if(mPoses[poseIndicesToUse[1]]->timestamp > timeOfCurrentRay)
                        {
                            mLastInterpolatedPose = Pose::interpolateLinear(
                                        mPoses[poseIndicesToUse[0]],
                                    mPoses[poseIndicesToUse[1]],
                                    timeOfCurrentRay);
                        }
                        else
                        {
                            mLastInterpolatedPose = Pose::interpolateLinear(
                                        mPoses[poseIndicesToUse[1]],
                                    mPoses[poseIndicesToUse[2]],
                                    timeOfCurrentRay);
                        }

                        emitLastInterpolatedPose();
                        //qDebug() << "pose source" << mPoses[poseIndicesToUse[1]]->toString();
                        mLastInterpolatedPose.transform(rawScan->relativeScannerPose);
                        //qDebug() << "pose done: " << mLastInterpolatedPose.toString();
                    }

                    fuseRayWithLastInterpolatedPose(index + rawScan->firstUsableDistance, distance);
                    // Skip angles when distance is close
                    if(distance < 1.0f) index += 3; else if(distance < 2.0f) index += 2; else if(distance < 3.0f) index += 1;
                }

                // This scan was successfully fused. Remove it from our vector
                iteratorRawScans.remove();

                mStatsScansFused[InterpolationMethod::Linear]++;

                mLastScannerPosition = mPoses[poseIndicesToUse[1]]->getPosition();
                //qDebug() << __PRETTY_FUNCTION__ << "now emitting" << mNumberOfPointsFusedInThisScan << "points";
                emit scanFused(mRegisteredPoints, mNumberOfPointsFusedInThisScan, &mLastScannerPosition);
            }
        }

        if(im == InterpolationMethod::NearestNeighbor)
        {
            // Skip this scan if no good pose was found
            if(abs(bestFitPoseTimeDifference) > MaximumFusionTimeOffset::NearestNeighbor) continue;

            mLastInterpolatedPose = *mPoses[bestFitPoseIndex];
            mLastInterpolatedPose.transform(rawScan->relativeScannerPose);

            emitLastInterpolatedPose();

            for(qint16 index=0; index < rawScan->numberOfDistances; index++)
            {
                // Only process every mStridePoint'th point
                if(index % mStridePoint != 0) continue;

                // Skip reflections on vehicle (=closer than 50cm) and long ones (bad platform orientation accuracy)
                if(rawScan->distances[index] < 700 || rawScan->distances[index] > mMaximumFusableRayLength * 1000.0f) continue;

                // Convert millimeters to meters.
                const float distance = rawScan->distances[index] / 1000.0f;

                fuseRayWithLastInterpolatedPose(index + rawScan->firstUsableDistance, distance);

                // Skip angles when distance is close
                if(distance < 1.0f) index += 3; else if(distance < 2.0f) index += 2; else if(distance < 3.0f) index += 1;
            }

            // This scan was successfully fused. Remove it from our vector
            iteratorRawScans.remove();

            mStatsScansFused[InterpolationMethod::NearestNeighbor]++;

            mLastScannerPosition = mPoses[bestFitPoseIndex]->getPosition();
            emit scanFused(mRegisteredPoints, mNumberOfPointsFusedInThisScan, &mLastScannerPosition);
        }
    }
}

qint8 SensorFuser::matchTimestamps()
{
    qint16 scansMatched = 0;
    qint16 scansUnmatched = 0;

    //qDebug() << "SensorFuser::matchTimestamps(): scnr stamps:" << getTimeStamps(mScansTimestampScanner).join(",");
    //qDebug() << "SensorFuser::matchTimestamps(): gnss stamps:" << getTimeStamps(mScansTimestampGnss).join(",");

    // Iterate through the list of gnss-timestamps and try to find fitting laserscanner-timestamps - as long as there are scanner scans
    QMutableListIterator<qint32> iteratorGnssTimeStamps(mGnssTimeStamps);
    while(iteratorGnssTimeStamps.hasNext())
    {
        const qint32 timestampGnss = iteratorGnssTimeStamps.next();

        // Find the timestamp from laserscanner closest to this one.
        //qDebug() << "SensorFuser::matchTimestamps(): looking for a matching scanner timestamp for gnss timestamp" << timestampGnss;

        // To which ScanInformation does the currently processed GNSS timestamp fit best? The index in the ScanInformation-Vector is recorded here
        qint32 bestFitTimestampScannerIndex = -1;
        qint32 smallestTimeDifference = 2147483647; // qint32 MAX, will always be an absolute value

        // Iterate through ScanInformation list and find temporally closest entry
        for(int currentIndexScanInformation=0;currentIndexScanInformation<mRawScans.size();currentIndexScanInformation++)
        {
            // Skip RawScans that already have a GNSS timestamps (includes those that
            // were set by hokuyos that are not connected to event-pins)
            if(mRawScans[currentIndexScanInformation]->timeStampScanMiddleGnss != 0)
                continue;

            const qint32 timestampScanner = mRawScans[currentIndexScanInformation]->timeStampScanMiddleScanner;
            if(abs(timestampScanner - timestampGnss) < smallestTimeDifference)
            {
                //qDebug() << "SensorFuser::matchTimestamps(): improved match between gnss" <<  timestampGnss << "and scanner timestamp" << timestampScanner << "to" << abs(timestampScanner - timestampGnss) << "msecs";
                smallestTimeDifference = abs(timestampScanner - timestampGnss);
                bestFitTimestampScannerIndex = currentIndexScanInformation;
            }
            else
            {
                // The time difference between our gnss_scan and the scanner_scan has INcreased, so we're on the wrong path. STOP!
                //qDebug() << "SensorFuser::matchTimestamps(): comparison to next scanner timestamp shows worse results than last time, error now:" << abs(timestampScanner - timestampGnss) << "msecs, breaking.";
                break;
            }
        }

        // Assign the gnss timestamp G to the scan S if:
        // - S and G are at least as close as mMaximumTimeOffsetBetweenScannerAndGnss
        // - S is not the last scan in the list. Because if it IS, the next scan might fit even better to G.
        if(smallestTimeDifference <= mMaximumTimeOffsetBetweenScannerAndGnss && (bestFitTimestampScannerIndex < mRawScans.size() - 1))
        {
            //            qDebug() << "SensorFuser::matchTimestamps(): assigning gnss timestamp" << timestampGnss << "to scanner time" << mScanInformation[bestFitTimestampScannerIndex].timeStampScanMiddleScanner << "- time difference:" << smallestTimeDifference;

            // fill ScanInfo with more precise gnss timestamp
            mRawScans[bestFitTimestampScannerIndex]->timeStampScanMiddleGnss = timestampGnss;

            // remove the gnss timestamp from the vector...
            iteratorGnssTimeStamps.remove();

            scansMatched++;
        }
        else
        {
            //qDebug() << "SensorFuser::matchTimestamps(): couldn't match scan from" << timestampGnss << "with any gnss timestamp, smallestTimeDifference was" << smallestTimeDifference;
            scansUnmatched++;
        }
    }

    //qDebug() << "SensorFuser::matchTimestamps(): gnss scans: matched" << scansMatched << "unmatched" << scansUnmatched << "total" << mScansTimestampGnss.size();
    return scansMatched;
}

void SensorFuser::slotNewVehiclePose(const Pose* const pose)
{
    if(!pose->isSufficientlyPreciseForSensorFusion())
    {
        qDebug() << "SensorFuser::slotNewVehiclePose(): received pose is not precise enough for fusing:" << pose->toString(true);
        return;
    }

    //Profiler p(__PRETTY_FUNCTION__);

    //qDebug() << "SensorFuser::slotNewVehiclePose(): received a usable" << *pose;

    // Make sure we receive data in order
    if(mPoses.size() && mPoses.last()->timestamp > pose->timestamp)
    {
        qDebug() << __PRETTY_FUNCTION__ << "last known pose t" << mPoses.last()->timestamp << "new pose t" << pose->timestamp;
        return;
    }

    // Append pose to our list
    mPoses.append(new Pose(pose));
    //qDebug() << __PRETTY_FUNCTION__ << "appended new pose from" << pose->timestamp;

    mNewestDataTime = std::max(mNewestDataTime, pose->timestamp);

    // Fuse pose and scans if it was possible to augment at least one lasertimestamp with a gnss timestamp
    // Sometimes, there are no gnss timestamps due to hardware/config problems; in this case, we still try
    // to match using scanner-timestamps only.
    if(matchTimestamps() || (mRawScans.size() > 2 && mPoses.size() > 2))
    {
        fuseScans();
    }

    // clear all data older than 2 seconds
    if(mPoses.size() > 10) slotClearData(2000);
}

void SensorFuser::slotFlushData()
{
    mFlushRemainingData = true;

    matchTimestamps();
    fuseScans();

    mFlushRemainingData = false;
}


void SensorFuser::slotScanFinished(const quint32 &timestampScanGnss)
{
    /*
      This method is called as part of the effort to synchronize LIDAR scanning with poses. The laserscanner
      pulls a SYNC-signal low for 1ms whenever a scan is finished. The next 24ms, the scanner does nothing to
      the pin, so it floats. To fix this, the pin is connected to VCC (3.34V) using a 2.7K pull-up resistor.
      This yields a signal like this: ---------(24ms)---------_---------(24ms)---------_---------(24ms)---...

      Previously, this signal was directly connected to the GNSS board's Event-B pin, which was configured to
      listen to (and timestamp) falling edges. The resulting SBF packet would be parsed by SbfParser and cause
      this method to be called with the given time (minus 12.5 milliseconds, which then equals the center-time
      of each scan-sweep (ray index 540 of 1080).

      This configuration worked well, but caused high CPU load in the GNSS receiver. To fix this, I inserted a
      74HC4020 linear logic IC which counts the number of falling edges on its CLOCK pin. This pin is now con-
      nected to the laserscanner's SYNC signal, so in effect, it counts the number of finished scans using 14
      bits. The GNSS receiver's Event-B pin is now connected to the 74HC4020's Q5-pin (the 5th bit), meaning
      that the signal on Event-B toggles between HIGH and LOW every 32 scans. Since the GNSS receiver acts on
      only EITHER the falling XOR the rising edge, we'll get a packet every 64 scans, which should be 64*25ms=
      1600ms.

      If this is too long (because drift/skew is too high for these intervals), I'll move the pin from Q4 to Q3,
      doubling the frequency. But in general, we want a long interval to keep GNSS receiver load low.

      To keep the following code from being hardcoded to this mechanism (or even a specific Q*-pin), we compute
      the interval from the last packet we received and derive the ratio of lidar-sync to gnss-sync from that.
      We also compute the drift and cause an alarm if it becomes too high, neccessitating a shorter ratio.
      */

    //qDebug() << "SensorFuser::slotScanFinished(): gnss receiver says scanner finished a scan at time" << timestampScanGnss;

    // Make sure we receive data in order
    if(mGnssTimeStamps.size() && mGnssTimeStamps.last() >= timestampScanGnss)
    {
        qDebug() << __PRETTY_FUNCTION__ << "last timestamp is" << mGnssTimeStamps.last() << "new timestamp is" << timestampScanGnss << "- this should never happen! Returning.";
        return;
    }

    // If we subscribe to ExtEvent and Support in different streams, we'll get the same ExtEvent from both
    // subscriptions. Make sure to only process an ExtEvent once, even if it comes in N times.
    if(mLastScanMiddleGnssTow >= timestampScanGnss) return;

    // Do not store data that we cannot fuse anyway, because the newest pose is very old (no gnss reception)
    if(!mPoses.size() || mPoses.last()->timestamp < (timestampScanGnss - 1000))
    {
        //qDebug() << "SensorFuser::slotScanFinished(): " << mPoses.size() << "/old poses, ignoring scanfinished gnss signal for scantime" << timestampScanGnss;
        return;
    }

    // Our gnss board tells us that a scan is finished. The scan data itself might already be saved in mSavedScans.
    const qint32 intervalBetweenSyncPackets = timestampScanGnss - mLastScanMiddleGnssTow;
    if(intervalBetweenSyncPackets > 3000)
    {
        //qDebug() << "SensorFuser::slotScanFinished(): interval between sync packets is" << intervalBetweenSyncPackets << "- this should ony happen during intialization.";
    }
    else
    {
        const quint8 ratio = (quint8)round(intervalBetweenSyncPackets / 25.0f);
        const qint8 driftMs = intervalBetweenSyncPackets - (ratio * 25);
        //qDebug() << "SensorFuser::slotScanFinished(): last sync packet" << mLastScanMiddleTow << "now" << timestampScanGnss << "interval" << intervalBetweenSyncPackets << "ms, ratio is" << ratio << "and drift is" << driftMs;

        // For every $ratio (=2^X) scans, we get a packet. Insert the correct scantimes into our data structure
        // For ratio 32, insert from -16*
        for(int i=-ratio/2; i<ceil(ratio/2.0f); i++)
        {
            const qint32 scanMiddleTow = timestampScanGnss + i*25;
            //qDebug() << "SensorFuser::slotScanFinished(): inserting scanMiddleTow" << i << ":" << scanMiddleTow;
            mGnssTimeStamps.append(scanMiddleTow);

            // update the latest data time
            if(i == ceil(ratio/2.0f) - 1) mNewestDataTime = std::max(mNewestDataTime, scanMiddleTow);
        }
    }
    mLastScanMiddleGnssTow = timestampScanGnss;
}

void SensorFuser::slotNewScanRaw(RawScan *scan)
{
    //qDebug() << "SensorFuser::slotNewRawScan(): received" << scan->numberOfDistances << "distance values from scannertime" << scan->timeStampScanMiddleScanner;

    //Profiler p(__PRETTY_FUNCTION__);

    // Do not store data that we cannot fuse anyway, because there is no pose or its very old (no gnss reception)
    if(!mPoses.size() || mPoses.last()->timestamp < (scan->timeStampScanMiddleScanner - MaximumFusionTimeOffset::Cubic))
    {
        // We cannot ignore the scan, as it was new()d somewhere!
        qDebug() << __PRETTY_FUNCTION__ << "no or old poses only, deleting new raw scan from" << scan->timeStampScanMiddleScanner;
        delete scan;
        return;
    }

    // Make sure we receive data in order. With multiple scanners, that's not so easy.
    if(mRawScans.size())
    {
        quint32 indexToInsert = mRawScans.size();
        while(indexToInsert > 0 && mRawScans.at(indexToInsert-1)->timeStampScanMiddleScanner > scan->timeStampScanMiddleScanner)
        {
            qDebug() << __PRETTY_FUNCTION__ << "oops, new raw scan from" << scan->timeStampScanMiddleScanner << "is older than rawscan at index" << indexToInsert << "from time" << mRawScans.at(indexToInsert-1)->timeStampScanMiddleScanner;
            indexToInsert--;
        }
        //qDebug() << __PRETTY_FUNCTION__ << "inserting scan from" << scan->timeStampScanMiddleScanner << "to index" << indexToInsert;
        mRawScans.insert(indexToInsert, scan);
    }
    else
    {
        mRawScans.append(scan);
    }


    mNewestDataTime = std::max(mNewestDataTime, scan->timeStampScanMiddleScanner);
}

void SensorFuser::emitLastInterpolatedPose()
{
    //    emit vehiclePose(&mLastInterpolatedPose);
    //    QCoreApplication::processEvents();
    //    usleep(100000);
}
