#include "sensorfuser.h"

SensorFuser::SensorFuser(const quint8& stridePoint, const quint8& strideScan) : QObject()
{
    mStridePoint = stridePoint;
    mStrideScan = strideScan; // TODO: implement

    mNewestDataTime = 0;
    mMaximumFusableRayLength = 200.0;
    mStatsScansDiscarded = 0;
    mLastScanMiddleGnssTow = 0;

    mBestInterpolationMethodToUse = InterpolationMethod::Cubic;

    mMaximumTimeOffsetBetweenFusedPoseAndScanMsec = 81; // 2*poseInterval+1
    mMaximumTimeOffsetBetweenScannerAndGnss = 12; // msecs maximum clock offset between scanner and gnss device. Smaller means less data, more means worse data.
}

SensorFuser::~SensorFuser()
{
    qDebug() << "SensorFuser::~SensorFuser(): total scans fused:"
             << "cubic:"  << mStatsScansFused[InterpolationMethod::Cubic]
             << "linear:" << mStatsScansFused[InterpolationMethod::Linear]
             << "nearestneighbor:" << mStatsScansFused[InterpolationMethod::NearestNeighbor]
             << "- discarded:"<< mStatsScansDiscarded;
    qDebug() << "SensorFuser::~SensorFuser(): leftover scanInfos:"  << mScanInformation.size() << "scanGnss:" << mGnssTimeStamps.size() << "poses:"<< mPoses.size();

    // delete leftover scaninfos including their range-data
    QMutableVectorIterator<ScanInformation> k(mScanInformation);
    while(k.hasNext())
    {
        ScanInformation& si = k.next();
        delete si.ranges;
        k.remove();
    }
}

void SensorFuser::cleanUnusableData()
{
    // Simply clean all data older than X ms!
    const qint32 maximumDataAge = 2000;

    // Remove all poses older than maximumDataAge
    QMutableVectorIterator<Pose> i(mPoses);
    while(i.hasNext())
    {
        if(i.next().timestamp + maximumDataAge < mNewestDataTime)
        {
            i.remove();
        }
    }

    // Remove all gnss timestamps older than maximumAge
    QMutableVectorIterator<qint32> j(mGnssTimeStamps);
    while(j.hasNext())
    {
        if(j.next() + maximumDataAge < mNewestDataTime)
        {
            j.remove();
        }
    }

    // Remove all gnss timestamps older than maximumAge
    QMutableVectorIterator<ScanInformation> k(mScanInformation);
    while(k.hasNext())
    {
        ScanInformation& si = k.next();
        // We check the scanner timestamp, thats precise enough
        if(si.timeStampScanMiddleScanner + maximumDataAge < mNewestDataTime)
        {
            delete si.ranges;
            k.remove();
        }
    }
}

void SensorFuser::fuseRay(const Pose& pose, const qint16 index, const float& distance)
{
    // Version using QMatrix4x4
    const QVector3D vectorScannerToPoint(
                sin(-0.0043633231299858238686f * (index - 540)) * distance,  // X in meters
                0.0,                                                                // Y always 0
                -cos(0.0043633231299858238686f * (index - 540)) * distance); // Z in meters

    mRegisteredPoints.append(pose * vectorScannerToPoint);
}

// According to mInterpolationmethod, we set each ScanInfo' object 's poses-list to be the 1(nn) / 2(linear) / 3(linear) / 4(cubic) / 5(cubic) best poses to interpolate
void SensorFuser::fuseScans()
{
    // Process every scan...
    QMutableVectorIterator<ScanInformation> iteratorScanInformation(mScanInformation);
    while(iteratorScanInformation.hasNext())
    {
        ScanInformation& scanInfo = iteratorScanInformation.next();

        // scanInfo can still contain scans with empty timeStampGnss values (value is 0) because they
        // weren't populated with matched scans-with-laserscanner-timestamps yet. Don't try to process those.
        if(scanInfo.timeStampScanMiddleGnss == 0)
        {
            continue;
        }

        const qint32 timestampMiddleOfScan = scanInfo.timeStampScanMiddleGnss;
        //qDebug() << "SensorFuser::fuseScans(): trying to fuse scan from" << timestampMiddleOfScan;

        // Which pose(-index) fits best to this scan?
        qint32 bestFitPoseIndex = -1;
        qint32 bestFitPoseTimeDifference = qint32_max;
        for(int currentIndexPose = 0; currentIndexPose < mPoses.size(); currentIndexPose++)
        {
            if(abs(mPoses[currentIndexPose].timestamp - timestampMiddleOfScan) < abs(bestFitPoseTimeDifference))
            {
                bestFitPoseIndex = currentIndexPose;

                // negative value means the pose was before the scan
                bestFitPoseTimeDifference = mPoses[currentIndexPose].timestamp - timestampMiddleOfScan;
            }
            else
            {
                // the time difference has INcreased, so lets break this loop.
                break;
            }
        }

        // If no pose was found, try the next scan.
        if(bestFitPoseTimeDifference == qint32_max) continue;

        // Even when interpolating e.g. cubically, the bestFit pose (not the other ones surrounding
        // the scan) should be as close as required for nearest neighbor. If not, skip this scan.
        if(abs(bestFitPoseTimeDifference) > MaximumFusionTimeOffset::NearestNeighbor) continue;

        // If mPoses[bestFitPoseIndex] is the last element, it means that the next element (coming
        // in the future) might be better. So, skip fusion in that case and try again next time.
        if(bestFitPoseIndex >= mPoses.size() - 1) continue;

        // Prepare for fusion :) Do not reserve full length, will be less poins due to reflections on the vehicle being filtered
        mRegisteredPoints.clear();
        mRegisteredPoints.reserve(800);

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
                }
                else if(abs(mPoses[poseIndex].timestamp - timestampMiddleOfScan) > MaximumFusionTimeOffset::Cubic)
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
                std::vector<long>* scanDistances = scanInfo.ranges;

                Pose pose;

                for(qint16 index=0; index < scanDistances->size(); index++)
                {
                    // Only process every mStridePoint'th point
                    if(index % mStridePoint != 0) continue;

                    // Skip reflections on vehicle (=closer than 50cm) and long ones (bad platform orientation accuracy)
                    if((*scanDistances)[index] < 500 || (*scanDistances)[index] > mMaximumFusableRayLength * 1000.0f) continue;

                    // Convert millimeters to meters.
                    const float distance = (*scanDistances)[index] / 1000.0f;

                    const qint32 timeOfCurrentRay = timestampMiddleOfScan + (qint32)(-9.375f + (((float)index) * 0.01736f));

                    if(pose.timestamp != timeOfCurrentRay)
                    {
                        // TODO: chek this form 20 and 50 Hz poses using yellow paper!
                        if(mPoses[poseIndicesToUse[2]].timestamp > timeOfCurrentRay)
                        {
                            pose = Pose::interpolateCubic(
                                        &mPoses[poseIndicesToUse[0]],
                                        &mPoses[poseIndicesToUse[1]],
                                        &mPoses[poseIndicesToUse[2]],
                                        &mPoses[poseIndicesToUse[3]],
                                        timeOfCurrentRay);
                        }
                        else
                        {
                            pose = Pose::interpolateCubic(
                                        &mPoses[poseIndicesToUse[1]],
                                        &mPoses[poseIndicesToUse[2]],
                                        &mPoses[poseIndicesToUse[3]],
                                        &mPoses[poseIndicesToUse[4]],
                                        timeOfCurrentRay);
                        }
                    }

                    fuseRay(pose, index, distance);
                }

                // This scan was successfully fused. Remove it from our vector
                iteratorScanInformation.remove();

                mStatsScansFused[InterpolationMethod::Cubic]++;

                mLastScannerPosition = mPoses[poseIndicesToUse[2]].getPosition();
                emit newScannedPoints(&mRegisteredPoints, &mLastScannerPosition);
            }
        }

        if(im == InterpolationMethod::Linear)
        {
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
                }
                else if(abs(mPoses[poseIndex].timestamp - timestampMiddleOfScan) > MaximumFusionTimeOffset::Linear)
                {
                    // mPoses[poseIndex] exists, but it is too far away time-wise. This means there are missing poses,
                    // so we should try degrading the interpolation-method
                    allPosesPresent = false;
                    im = InterpolationMethod::NearestNeighbor;
                }
            }

            if(allPosesPresent)
            {
                // We finally have all poses required to fuse this scan. Go!
                std::vector<long>* scanDistances = scanInfo.ranges;

                Pose pose;

                for(qint16 index=0; index < scanDistances->size(); index++)
                {
                    // Only process every mStridePoint'th point
                    if(index % mStridePoint != 0) continue;

                    // Skip reflections on vehicle (=closer than 50cm) and long ones (bad platform orientation accuracy)
                    if((*scanDistances)[index] < 500 || (*scanDistances)[index] > mMaximumFusableRayLength * 1000.0f) continue;

                    // Convert millimeters to meters.
                    const float distance = (*scanDistances)[index] / 1000.0f;

                    const qint32 timeOfCurrentRay = timestampMiddleOfScan + (qint32)(-9.375f + (((float)index) * 0.01736f));

                    if(pose.timestamp != timeOfCurrentRay)
                    {
                        if(mPoses[poseIndicesToUse[1]].timestamp > timeOfCurrentRay)
                        {
                            pose = Pose::interpolateLinear(
                                        mPoses[poseIndicesToUse[0]],
                                        mPoses[poseIndicesToUse[1]],
                                        timeOfCurrentRay);
                        }
                        else
                        {
                            pose = Pose::interpolateLinear(
                                        mPoses[poseIndicesToUse[1]],
                                        mPoses[poseIndicesToUse[2]],
                                        timeOfCurrentRay);
                        }
                    }

                    fuseRay(pose, index, distance);
                }

                // This scan was successfully fused. Remove it from our vector
                iteratorScanInformation.remove();

                mStatsScansFused[InterpolationMethod::Linear]++;

                mLastScannerPosition = mPoses[poseIndicesToUse[1]].getPosition();
                emit newScannedPoints(&mRegisteredPoints, &mLastScannerPosition);
            }
        }

        if(im == InterpolationMethod::NearestNeighbor)
        {
            // Skip this scan if no good pose was found
            if(abs(bestFitPoseTimeDifference) > MaximumFusionTimeOffset::NearestNeighbor) continue;

            Pose& pose = mPoses[bestFitPoseIndex];

            std::vector<long>* scanDistances = scanInfo.ranges;

            for(qint16 index=0; index < scanDistances->size(); index++)
            {
                // Only process every mStridePoint'th point
                if(index % mStridePoint != 0) continue;

                // Skip reflections on vehicle (=closer than 50cm) and long ones (bad platform orientation accuracy)
                if((*scanDistances)[index] < 500 || (*scanDistances)[index] > mMaximumFusableRayLength * 1000.0f) continue;

                // Convert millimeters to meters.
                const float distance = (*scanDistances)[index] / 1000.0f;

                fuseRay(pose, index, distance);
            }

            // This scan was successfully fused. Remove it from our vector
            iteratorScanInformation.remove();

            mStatsScansFused[InterpolationMethod::NearestNeighbor]++;

            mLastScannerPosition = mPoses[bestFitPoseIndex].getPosition();
            emit newScannedPoints(&mRegisteredPoints, &mLastScannerPosition);
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
    QMutableVectorIterator<qint32> iteratorGnssTimeStamps(mGnssTimeStamps);
    while(iteratorGnssTimeStamps.hasNext())
    {
        const qint32 timestampGnss = iteratorGnssTimeStamps.next();

        // Find the timestamp from laserscanner closest to this one.
        //qDebug() << "SensorFuser::matchTimestamps(): looking for a matching scanner timestamp for gnss timestamp" << timestampGnss;

        // To which ScanInformation does the currently processed GNSS timestamp fit best? The index in the ScanInformation-Vector is recorded here
        qint32 bestFitTimestampScannerIndex = -1;
        qint32 smallestTimeDifference = 2147483647; // qint32 MAX, will always be an absolute value

        // Iterate through ScanInformation list and find temporally closest entry
        for(int currentIndexScanInformation=0;currentIndexScanInformation<mScanInformation.size();currentIndexScanInformation++)
        {
            const qint32 timestampScanner = mScanInformation[currentIndexScanInformation].timeStampScanMiddleScanner;
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

        // If we found a close-enough match, use it.
        if(smallestTimeDifference <= mMaximumTimeOffsetBetweenScannerAndGnss)
        {
            //qDebug() << "SensorFuser::matchTimestamps(): using data from scanner timestamp" << bestFittingScannerTime << "to populate gnss timestamp" << timestampGnss << "time difference" << smallestTimeDifference;

            // fill ScanInfo with more precise gnss timestamp
            mScanInformation[bestFitTimestampScannerIndex].timeStampScanMiddleGnss = timestampGnss;

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
    if(!(
            pose->precision & Pose::AttitudeAvailable &&
            pose->precision & Pose::RtkFixed &&
            pose->precision & Pose::CorrectionAgeLow &&
            pose->precision & Pose::HeadingFixed &&
            //pose->precision & Pose::ModeIntegrated &&
            pose->covariances < Pose::maximumUsableCovariance
            ))
    {
//        qDebug() << t() << "SensorFuser::slotNewVehiclePose(): received pose is not precise enough for fusing, ignoring it";
        return;
    }

//    qDebug() << t() << "SensorFuser::slotNewVehiclePose(): received a " << pose;

    // Append pose to our list
    mPoses.append((*pose) * mLaserScannerRelativePose);

    //qDebug() << "SensorFuser::slotNewVehiclePose(): vehicle" << pose << "relative scanner" << mLaserScannerRelativePose << "result" << mPoses.last();

    mNewestDataTime = std::max(mNewestDataTime, pose->timestamp);

    // Fuse pose and scans if it was possible to augment at least one lasertimestamp with a gnss timestamp
    if(matchTimestamps())
    {
//        transformScanDataNearestNeighbor();
        fuseScans();
    }

    cleanUnusableData();
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

    // If we subscribe to ExtEvent and Support in different streams, we'll get the same ExtEvent from both
    // subscriptions. Make sure to only process an ExtEvent once, even if it comes in N times.
    if(mLastScanMiddleGnssTow >= timestampScanGnss) return;

    //qDebug() << t() << "SensorFuser::slotScanFinished(): gnss receiver says scanner finished a scan at time" << timestampScanGnss;

    // Do not store data that we cannot fuse anyway, because the newest pose is very old (no gnss reception)
    if(!mPoses.size() || mPoses.last().timestamp < (timestampScanGnss - 1000))
    {
        //qDebug() << t() << "SensorFuser::slotScanFinished(): " << mPoses.size() << "/old poses, ignoring scanfinished gnss signal for scantime" << timestampScanGnss;
        return;
    }

    // Our gnss board tells us that a scan is finished. The scan data itself might already be saved in mSavedScans.
    const qint32 intervalBetweenSyncPackets = timestampScanGnss - mLastScanMiddleGnssTow;
    if(intervalBetweenSyncPackets > 3000)
    {
//        qDebug() << t() << "SensorFuser::slotScanFinished(): interval between sync packets is" << intervalBetweenSyncPackets << "- this should ony happen during intialization.";
    }
    else
    {
        const quint8 ratio = (quint8)round(intervalBetweenSyncPackets / 25.0f);
        const qint8 driftMs = intervalBetweenSyncPackets - (ratio * 25);
//        qDebug() << t() << "SensorFuser::slotScanFinished(): last sync packet" << mLastScanMiddleTow << "now" << timestampScanGnss << "interval" << intervalBetweenSyncPackets << "ms, ratio is" << ratio << "and drift is" << driftMs;

        // For every $ratio (=2^X) scans, we get a packet. Insert the correct scantimes into our data structure
        // For ratio 32, insert from -16*
        for(int i=-ratio/2; i<ceil(ratio/2.0f); i++)
        {
            const qint32 scanMiddleTow = timestampScanGnss + i*25;
            //qDebug() << t() << "SensorFuser::slotScanFinished(): inserting scanMiddleTow" << i << ":" << scanMiddleTow;
            mGnssTimeStamps.append(scanMiddleTow);

            // update the latest data time
            if(i == ceil(ratio/2.0f) - 1) mNewestDataTime = std::max(mNewestDataTime, scanMiddleTow);
        }
    }
    mLastScanMiddleGnssTow = timestampScanGnss;
}

void SensorFuser::slotNewScanData(const qint32& timestampScanScanner, std::vector<long> * const distances)
{
//    qDebug() << t() << "SensorFuser::slotNewScanData(): received" << distances->size() << "distance values from scannertime" << timestampScanScanner;

    // We need this only when the Event-Pins don't work, so we create our own fake events
    //slotScanFinished(timestampScanScanner);

    // Do not store data that we cannot fuse anyway, because the newest pose is very old (no gnss reception)
    if(!mPoses.size() || mPoses.last().timestamp < (timestampScanScanner - mMaximumTimeOffsetBetweenFusedPoseAndScanMsec - 13))
    {
//        qDebug() << t() << "SensorFuser::slotNewScanData(): " << mPoses.size() << "/old poses, ignoring scandata at time" << timestampScanScanner;
        // We cannot ignore the scandata, we must at least delete() it, because it was new()ed in LaserScanner and we are now the owner.
        delete distances;
        return;
    }

    mScanInformation.append(ScanInformation());
    mScanInformation.last().timeStampScanMiddleScanner = timestampScanScanner;
    mScanInformation.last().ranges = distances;

    mNewestDataTime = std::max(mNewestDataTime, timestampScanScanner);
}
