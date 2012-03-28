#include "pose.h"

Pose::Pose(const QVector3D &position, const QQuaternion &orientation, const qint32& timestamp)
{
    this->position = position;

    mYaw = keepWithinRangeRadians(getYawRadians(orientation, true));
    mPitch = keepWithinRangeRadians(getPitchRadians(orientation, true));
    mRoll = keepWithinRangeRadians(getRollRadians(orientation, true));

    this->timestamp = timestamp;

    precision = 0;
    covariances = 100.0f;

    if(timestamp == 0) this->timestamp = getCurrentGpsTowTime();
}

Pose::Pose(const QVector3D &position, const float &yawDegrees, const float &pitchDegrees, const float &rollDegrees, const qint32& timestamp)
{
    this->position = position;

    mYaw = keepWithinRangeRadians(DEG2RAD(yawDegrees));
    mPitch = keepWithinRangeRadians(DEG2RAD(pitchDegrees));
    mRoll = keepWithinRangeRadians(DEG2RAD(rollDegrees));

    this->timestamp = timestamp;

    precision = 0;
    covariances = 100.0f;

    if(timestamp == 0)
    {
        this->timestamp = getCurrentGpsTowTime();
    }
}

Pose::Pose()
{
    mYaw = 0.0f;
    mPitch = 0.0f;
    mRoll = 0.0f;

    precision = 0;
    covariances = 100.0f;
    this->timestamp = 0;
}

Pose Pose::interpolateLinear(const Pose &before, const Pose &after, const float &mu)
{
    Q_ASSERT(mu <= 0.0 && mu <= 1.0);

    Pose p(
                before.position * (1.0 - mu) + after.position * mu,
                RAD2DEG(before.mYaw * (1.0 - mu) + after.mYaw * mu),
                RAD2DEG(before.mPitch * (1.0 - mu) + after.mPitch * mu),
                RAD2DEG(before.mRoll * (1.0 - mu) + after.mRoll * mu),
                before.timestamp * (1.0 - mu) + after.timestamp * mu
                );

    p.covariances = before.covariances * (1.0 - mu) + after.covariances * mu;
    p.precision = before.precision & after.precision;

    return p;
}

// http://paulbourke.net/miscellaneous/interpolation/

// TODO: This code doesn't really use the timestamps in the poses, that seems stupid
Pose Pose::interpolateCubic(const Pose * const first, const Pose * const before, const Pose * const after, const Pose * const last, const float &mu)
{//                                             y0                        y1                         y2                        y3
    Q_ASSERT(mu >= 0.0 && mu <= 1.0);

    const double mu2 = mu*mu;

    // position
    const QVector3D po0 = last->position - after->position - first->position + before->position;
    const QVector3D po1 = first->position - before->position - po0;
    const QVector3D po2 = after->position - first->position;
    const QVector3D po3 = before->position;

    QVector3D resultPosition = po0*mu*mu2+po1*mu2+po2*mu+po3;

    // yaw - what happens with values around +-180?!r
    const float y0 = last->mYaw - after->mYaw - first->mYaw + before->mYaw;
    const float y1 = first->mYaw - before->mYaw - y0;
    const float y2 = after->mYaw - first->mYaw;
    const float y3 = before->mYaw;

    const float yaw = y0*mu*mu2+y1*mu2+y2*mu+y3;

    // pitch
    const float p0 = last->mPitch - after->mPitch - first->mPitch + before->mPitch;
    const float p1 = first->mPitch - before->mPitch - p0;
    const float p2 = after->mPitch - first->mPitch;
    const float p3 = before->mPitch;

    const float pitch = p0*mu*mu2+p1*mu2+p2*mu+p3;

    // roll
    const float r0 = last->mRoll - after->mRoll - first->mRoll + before->mRoll;
    const float r1 = first->mRoll - before->mRoll - r0;
    const float r2 = after->mRoll - first->mRoll;
    const float r3 = before->mRoll;

    const float roll = r0*mu*mu2+r1*mu2+r2*mu+r3;

    // roll
//    float  t0, t1, t2, t3;
//    const float t0 = last->timestamp - after->timestamp - first->timestamp + before->timestamp;
//    const float t1 = first->timestamp - before->timestamp - t0;
//    const float t2 = after->timestamp - first->timestamp;
//    const float t3 = before->timestamp;

    // too expensive.
//    const float timestamp = t0*mu*mu2+t1*mu2+t2*mu+t3;
//    const qint32 timestamp = before->timestamp + (qint32)(mu * ((float)(after->timestamp - before->timestamp)));

    Pose p(resultPosition, RAD2DEG(yaw), RAD2DEG(pitch), RAD2DEG(roll), 0);

    p.covariances = (before->covariances + after->covariances) / 2.0f;
    p.precision = before->precision & after->precision;

    return p;
}

Pose Pose::interpolateCubic(const Pose * const first, const Pose * const before, const Pose * const after, const Pose * const last, const qint32& time)
{//                                             y0                        y1                         y2                        y3

    // Check parameters.
    Q_ASSERT(first->timestamp < before->timestamp && "Pose::interpolateCubic(): first < before didn't pass");

    if(!(before->timestamp <= time)) qDebug() << "Pose::interpolateCubic(): before" << before->timestamp << "<= raytime" << time <<  "didn't pass";
    if(!(after->timestamp >= time)) qDebug() << "Pose::interpolateCubic(): after" << after->timestamp << ">= raytime" << time <<  "didn't pass";

    Q_ASSERT(last->timestamp > after->timestamp && "Pose::interpolateCubic(): last > after didn't pass");

    // recreate mu from time argument
    const float mu = (((float)(time - before->timestamp)) / ((float)(after->timestamp - before->timestamp)));
    Pose p = interpolateCubic(first, before, after, last, mu);
    p.timestamp = time;
    return p;
}


float Pose::getShortestTurnRadians(float angle)
{
    return DEG2RAD(getShortestTurnDegrees(RAD2DEG(angle)));
}

float Pose::getShortestTurnDegrees(float angle)
{
    while(angle <= -180.0) angle += 360.0;
    while(angle > 180.0) angle -= 360.0;
    return angle;
}

float Pose::keepWithinRangeDegrees(float angleDegrees)
{
    // When two angles are added, e.g. 270 + 270 deg, we arrive at 540,
    // which can be simplified to 180 degrees.
    while(angleDegrees > 180.0f) angleDegrees -= 360.0f;
    while(angleDegrees < -180.0f) angleDegrees += 360.0f;

    return angleDegrees;
}

float Pose::keepWithinRangeRadians(float angleRadians)
{
    return DEG2RAD(keepWithinRangeDegrees(RAD2DEG(angleRadians)));
}

QVector2D Pose::getPlanarPosition() const
{
    return QVector2D(position.x(), position.z());
}

QVector2D Pose::getPlanarDirection() const
{
    const float y = -cos(getYawRadians());
    const float x = -sin(getYawRadians());

    const QVector2D result = QVector2D(x, y).normalized();
    qDebug() << "Pose::getPlanarDirection(): angle" << getYawDegrees() << "result" << result;
    return result;
}

// No idea whether the order of orientation is correct
const QQuaternion Pose::getOrientation() const
{
    // FIXME: This is buggy! It was used for buggy laserscanner-fusion, too.
    return
            QQuaternion::fromAxisAndAngle(QVector3D(0,1,0), getYawDegrees())
            * QQuaternion::fromAxisAndAngle(QVector3D(1,0,0), getPitchDegrees())
            * QQuaternion::fromAxisAndAngle(QVector3D(0,0,1), getRollDegrees());
}
/*
QMatrix4x4 Pose::getMatrix()
{
    QMatrix4x4 matrix;
    matrix.translate(position);

    matrix.rotate(getOrientation());


    matrix.rotate(getYawDegrees(), QVector3D(0,1,0));

    matrix.rotate(getPitchDegrees(), QVector3D(1,0,0));

    // The more we pitch, the more our roll should happen on the yaw axis. Whee.
    matrix.rotate(
                getRollDegrees(),
                QVector3D(
                    0,
                    1,//cos(scannerPose.getPitchRadians()),
                    0)//sin(scannerPose.getPitchRadians())
                );
}*/

Pose Pose::operator+(const Pose &p) const
{



    Pose a(
                position + getOrientation().rotatedVector(p.position),
                RAD2DEG(mYaw + p.getYawRadians()),
                RAD2DEG(mPitch + p.getPitchRadians()),
                RAD2DEG(mRoll + p.getRollRadians()),
                // use the latest timestamp, needed in LaserScanner::slotNewVehiclePose(const Pose& pose)
                std::max(timestamp,p.timestamp)
                );

    a.covariances = std::max(covariances, p.covariances);
    a.precision = precision & p.precision;

    return a;
}

QDebug operator<<(QDebug dbg, const Pose &pose)
{
    dbg.nospace() << pose.toString();

    return dbg.space();
}

const QString Pose::toString() const
{
    return QString()
            .append("pose t").append(QString::number(timestamp))
            .append(" (").append(QString::number(position.x(), 'f', 2))
            .append("/").append(QString::number(position.y(), 'f', 2))
            .append("/").append(QString::number(position.z(), 'f', 2))
            .append(") YPR (").append(QString::number(getYawDegrees(), 'f', 2))
            .append("/").append(QString::number(getPitchDegrees(), 'f', 2))
            .append("/").append(QString::number(getRollDegrees(), 'f', 2)).append(")")
            .append(" pr ").append(QString::number(precision))
            .append(" co ").append(QString::number(covariances, 'f', 2));
}

const QString Pose::getFlagsString() const
{
    QString flags;
    if(precision & Pose::AttitudeAvailable) flags.append("+ATT"); else flags.append("-ATT");
    if(precision & Pose::CorrectionAgeLow) flags.append(" +CRL"); else flags.append(" -CRL");
    if(precision & Pose::RtkFixed) flags.append(" +RTK"); else flags.append(" -RTK");
    if(precision & Pose::ModeIntegrated) flags.append(" +INT"); else flags.append(" -INT");
    return flags;
}

const QString Pose::toStringVerbose() const
{

    return QString()
            .append("pose t").append(QString::number(timestamp))
            .append(" (").append(QString::number(position.x(), 'f', 2))
            .append("/").append(QString::number(position.y(), 'f', 2))
            .append("/").append(QString::number(position.z(), 'f', 2))
            .append(") YPR (").append(QString::number(getYawDegrees(), 'f', 2))
            .append("/").append(QString::number(getPitchDegrees(), 'f', 2))
            .append("/").append(QString::number(getRollDegrees(), 'f', 2)).append(")")
            .append(" pr ").append(getFlagsString())
            .append(" co ").append(QString::number(covariances, 'f', 2));
}

// Must be able to process what operator<< writes above, for example:
// pose t501171350 (-30.49/51.84/140.01) YPR (155.27/2.92/-1.03) pr 17 co 2.34
Pose::Pose(const QString& poseString)
{
    QStringList tokens = poseString.split(' ');
    if(tokens.size() != 9) qDebug() << "Pose::Pose(QString): token stringlist size is not 5!";
    bool success = false;

    // set time
    const qint32 timestamp = tokens.value(1).remove(0, 1).toInt(&success, 10);
    Q_ASSERT(success && "Pose::Pose(QString): couldn't convert time to int.");
    this->timestamp = timestamp;

    // set positions
    QString positionString = tokens.value(2).remove(0, 1);
    positionString.chop(1);
    const QStringList positions = positionString.split('/');
    position.setX(positions.at(0).toFloat(&success));
    if(!success) qDebug() << "Pose::Pose(QString): couldn't convert X to float.";
    position.setY(positions.at(1).toFloat(&success));
    if(!success) qDebug() << "Pose::Pose(QString): couldn't convert Y to float.";
    position.setZ(positions.at(2).toFloat(&success));
    if(!success) qDebug() << "Pose::Pose(QString): couldn't convert Z to float.";

    // set orientations
    QString orientationString = tokens.value(4).remove(0, 1);
    orientationString.chop(1);
    const QStringList orientations = orientationString.split('/');
    setYawDegrees(orientations.at(0).toFloat(&success));
    if(!success) qDebug() << "Pose::Pose(QString): couldn't convert yaw to float.";
    setPitchDegrees(orientations.at(1).toFloat(&success));
    if(!success) qDebug() << "Pose::Pose(QString): couldn't convert pitch to float.";
    setRollDegrees(orientations.at(2).toFloat(&success));
    if(!success) qDebug() << "Pose::Pose(QString): couldn't convert roll to float.";

    // set precision and covariances
    precision = tokens.value(6).toInt(&success);
    if(!success) qDebug() << "Pose::Pose(QString): couldn't convert precision to int.";

    covariances = tokens.value(8).toFloat(&success);
    if(!success) qDebug() << "Pose::Pose(QString): couldn't convert covariances to float.";

    if(poseString != toString())
        qDebug() << "Pose::Pose(QString): parsing failed: original:" << poseString << "reconstructed" << toString();
}

QDataStream& operator<<(QDataStream &out, const Pose &pose)
{
    out << pose.position << pose.getYawRadians() << pose.getPitchRadians() << pose.getRollRadians() << pose.timestamp << pose.precision << pose.covariances;
    return out;
}

QDataStream& operator>>(QDataStream &in, Pose &pose)
{
    float yaw, pitch, roll;
    in >> pose.position;
    in >> yaw;
    in >> pitch;
    in >> roll;
    in >> pose.timestamp;
    in >> pose.precision;
    in >> pose.covariances;

//    qDebug() << "reconstructing pose with YPR:" << yaw << pitch << roll;

    pose.setYawRadians(yaw);
    pose.setPitchRadians(pitch);
    pose.setRollRadians(roll);

    return in;
}

float Pose::getRollRadians(const QQuaternion& orientation, bool reprojectAxis)
{
    if(reprojectAxis)
    {
        // ben: this was Real in ogre, so it might be better to use double
        // roll = atan2(localx.y, localx.x)
        // pick parts of xAxis() implementation that we need
        float fTx  = 2.0 * orientation.x();
        float fTy  = 2.0 * orientation.y();
        float fTz  = 2.0 * orientation.z();
        float fTwz = fTz * orientation.scalar();
        float fTxy = fTy * orientation.x();
        float fTyy = fTy * orientation.y();
        float fTzz = fTz * orientation.z();

        // Vector3(1.0-(fTyy+fTzz), fTxy+fTwz, fTxz-fTwy);
        return atan2(fTxy+fTwz, 1.0-(fTyy+fTzz));
    }
    else
    {
        return atan2(2*(orientation.x() * orientation.y() + orientation.scalar() * orientation.z() ), orientation.scalar() * orientation.scalar() + orientation.x() * orientation.x() - orientation.y() * orientation.y() - orientation.z() * orientation.z());
    }
}

float Pose::getPitchRadians(const QQuaternion& orientation, bool reprojectAxis)
{
    if(reprojectAxis)
    {
        // pitch = atan2(localy.z, localy.y)
        // pick parts of yAxis() implementation that we need
        float fTx  = 2.0 * orientation.x();
        float fTy  = 2.0 * orientation.y();
        float fTz  = 2.0 * orientation.z();
        float fTwx = fTx * orientation.scalar();
        float fTxx = fTx * orientation.x();
        float fTyz = fTz * orientation.y();
        float fTzz = fTz * orientation.z();

        // Vector3(fTxy-fTwz, 1.0-(fTxx+fTzz), fTyz+fTwx);
        return atan2(fTyz+fTwx, 1.0-(fTxx+fTzz));
    }
    else
    {
        // internal version
        return atan2(2*(orientation.y() * orientation.z() + orientation.scalar() * orientation.x() ), orientation.scalar() * orientation.scalar() - orientation.x() * orientation.x() - orientation.y() * orientation.y() + orientation.z() * orientation.z());
    }
}

float Pose::getYawRadians(const QQuaternion& orientation, bool reprojectAxis)
{
    if(reprojectAxis)
    {
        // yaw = atan2(localz.x, localz.z)
        // pick parts of zAxis() implementation that we need
        float fTx  = 2.0 * orientation.x();
        float fTy  = 2.0 * orientation.y();
        float fTz  = 2.0 * orientation.z();
        float fTwy = fTy * orientation.scalar();
        float fTxx = fTx * orientation.x();
        float fTxz = fTz * orientation.x();
        float fTyy = fTy * orientation.y();

        // Vector3(fTxz+fTwy, fTyz-fTwx, 1.0-(fTxx+fTyy));
        return atan2(fTxz+fTwy, 1.0-(fTxx+fTyy));
    }
    else
    {
        // internal version
        return asin(-2*(orientation.x() * orientation.z() - orientation.scalar() * orientation.y()));
    }
}
