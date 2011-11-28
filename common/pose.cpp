#include "pose.h"

Pose::Pose(const QVector3D &position, const QQuaternion &orientation, const qint32& timestamp)
{
    this->position = position;

    mYaw = getYawRadians(orientation, true);
    mPitch = getPitchRadians(orientation, true);
    mRoll = getRollRadians(orientation, true);

    this->timestamp = timestamp;

    if(timestamp == 0) this->timestamp = getCurrentGpsTowTime();
}

Pose::Pose(const QVector3D &position, const float &yawDegrees, const float &pitchDegrees, const float &rollDegrees, const qint32& timestamp)
{
    this->position = position;

    mYaw = DEG2RAD(yawDegrees);
    mPitch = DEG2RAD(pitchDegrees);
    mRoll = DEG2RAD(rollDegrees);

    this->timestamp = timestamp;

    if(timestamp == 0)
    {
        this->timestamp = getCurrentGpsTowTime();
    }
}

Pose::Pose()
{
    this->timestamp = 0;
}

Pose Pose::interpolateLinear(const Pose &before, const Pose &after, const float &mu)
{
    Q_ASSERT(mu <= 0.0 && mu <= 1.0);

    return Pose(
                before.position * (1.0 - mu) + after.position * mu,
                before.mYaw * (1.0 - mu) + after.mYaw * mu,
                before.mPitch * (1.0 - mu) + after.mPitch * mu,
                before.mRoll * (1.0 - mu) + after.mRoll * mu,
                before.timestamp * (1.0 - mu) + after.timestamp * mu
                );
}

// http://paulbourke.net/miscellaneous/interpolation/

// TODO: This code doesn't really use the timestamps in the poses, that seems stupid
Pose Pose::interpolateCubic(const Pose * const first, const Pose * const before, const Pose * const after, const Pose * const last, const float &mu)
{//                                             y0                        y1                         y2                        y3
    Q_ASSERT(mu >= 0.0 && mu <= 1.0);

    const double mu2 = mu*mu;

    // position
//    QVector3D po0, po1, po2, po3;
    const QVector3D po0 = last->position - after->position - first->position + before->position;
    const QVector3D po1 = first->position - before->position - po0;
    const QVector3D po2 = after->position - first->position;
    const QVector3D po3 = before->position;

    QVector3D resultPosition = po0*mu*mu2+po1*mu2+po2*mu+po3;

    // yaw
//    float  y0, y1, y2, y3;
    const float y0 = last->mYaw - after->mYaw - first->mYaw + before->mYaw;
    const float y1 = first->mYaw - before->mYaw - y0;
    const float y2 = after->mYaw - first->mYaw;
    const float y3 = before->mYaw;

    const float yaw = y0*mu*mu2+y1*mu2+y2*mu+y3;

    // pitch
//    float  p0, p1, p2, p3;
    const float p0 = last->mPitch - after->mPitch - first->mPitch + before->mPitch;
    const float p1 = first->mPitch - before->mPitch - p0;
    const float p2 = after->mPitch - first->mPitch;
    const float p3 = before->mPitch;

    const float pitch = p0*mu*mu2+p1*mu2+p2*mu+p3;

    // roll
//    float  r0, r1, r2, r3;
    const float r0 = last->mRoll - after->mRoll - first->mRoll + before->mRoll;
    const float r1 = first->mRoll - before->mRoll - r0;
    const float r2 = after->mRoll - first->mRoll;
    const float r3 = before->mRoll;

    const float roll = r0*mu*mu2+r1*mu2+r2*mu+r3;

    // roll
//    float  t0, t1, t2, t3;
    const float t0 = last->timestamp - after->timestamp - first->timestamp + before->timestamp;
    const float t1 = first->timestamp - before->timestamp - t0;
    const float t2 = after->timestamp - first->timestamp;
    const float t3 = before->timestamp;

    const float timestamp = t0*mu*mu2+t1*mu2+t2*mu+t3;

    return Pose(resultPosition, RAD2DEG(yaw), RAD2DEG(pitch), RAD2DEG(roll), timestamp);
}

Pose Pose::interpolateCubic(const Pose * const first, const Pose * const before, const Pose * const after, const Pose * const last, const qint32& time)
{//                                             y0                        y1                         y2                        y3

    // Check parameters.
    Q_ASSERT(first->timestamp < before->timestamp && "Pose::interpolateCubic(): first < before didn't pass");
    Q_ASSERT(before->timestamp < time && "Pose::interpolateCubic(): before < raytime didn't pass");
    Q_ASSERT(after->timestamp > time && "Pose::interpolateCubic(): after > raytime didn't pass");
    Q_ASSERT(last->timestamp > after->timestamp && "Pose::interpolateCubic(): last > after didn't pass");

    // recreate mu from time argument
    const float mu = (time - before->timestamp) / (after->timestamp - before->timestamp);
    return interpolateCubic(first, before, after, last, mu);
}


float Pose::getShortestTurnRadians(const float& angle)
{
    return DEG2RAD(getShortestTurnDegrees(RAD2DEG(angle)));
}

float Pose::getShortestTurnDegrees(const float& angle)
{
    float angleNew = angle;
    while(angleNew <= -180.0) angleNew += 360.0;
    while(angleNew > 180.0) angleNew -= 360.0;
    return angleNew;
}

float Pose::keepWithinRangeRadians(float angleRadians)
{
    // When two angles are added, e.g. 270 + 270 deg, we arrive at 540,
    // which can be simplified to 180 degrees.
    // This is DIFFERENT from normalizeAngle, please see its description
    while(angleRadians > DEG2RAD(360.0)) angleRadians -= DEG2RAD(360.0);
    while(angleRadians < DEG2RAD(-360.0)) angleRadians += DEG2RAD(360.0);

    return angleRadians;
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

/* unused
Pose Pose::operator*(const float &factor)
{
    return Pose(...);
}*/

// No idea whether the order of orientation is correct
const QQuaternion Pose::getOrientation() const
{
    return
            QQuaternion::fromAxisAndAngle(QVector3D(0,1,0), getYawDegrees())
            * QQuaternion::fromAxisAndAngle(QVector3D(1,0,0), getPitchDegrees())
            * QQuaternion::fromAxisAndAngle(QVector3D(0,0,1), getRollDegrees());
}

Pose Pose::operator+(const Pose &p) const
{
    // The following two should be the same
    return Pose(
                position + getOrientation().rotatedVector(p.position),
                RAD2DEG(keepWithinRangeRadians(mYaw + p.getYawRadians())),
                RAD2DEG(keepWithinRangeRadians(mPitch + p.getPitchRadians())),
                RAD2DEG(keepWithinRangeRadians(mRoll + p.getRollRadians())),
                // use the latest timestamp, needed in LaserScanner::slotNewVehiclePose(const Pose& pose)
                std::max(timestamp,p.timestamp)
                );

    /*return Pose(
                position + getOrientation().rotatedVector(p.position),
                p.getOrientation() * getOrientation(),
                // use the latest timestamp, needed in LaserScanner::slotNewVehiclePose(const Pose& pose)
                std::max(timestamp,p.timestamp)
                );*/
}

/*const Pose Pose::operator-(const Pose &p) const
{
    Q_ASSERT(false);
}*/


QDebug operator<<(QDebug dbg, const Pose &pose)
{
    dbg.nospace() << "pose t" << pose.timestamp
                  << " (" << Q(QString::number(pose.position.x(), 'f', 2))
                  << "/" << Q(QString::number(pose.position.y(), 'f', 2))
                  << "/" << Q(QString::number(pose.position.z(), 'f', 2))
                  << ") YPR (" << Q(QString::number(pose.getYawDegrees(), 'f', 2))
                  << "/" << Q(QString::number(pose.getPitchDegrees(), 'f', 2))
                  << "/" << Q(QString::number(pose.getRollDegrees(), 'f', 2)) << ")";

    return dbg.space();
}

QDataStream& operator<<(QDataStream &out, const Pose &pose)
{
    out << pose.position << pose.getYawRadians() << pose.getPitchRadians() << pose.getRollRadians() << pose.timestamp;
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
