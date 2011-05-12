#include "pose.h"

Pose::Pose(const QVector3D &position, const QQuaternion &orientation, const quint32& timestamp)
{
    this->position = position;

//    setYawRadians(getYawRadians(orientation, true));
//    setPitchRadians(getPitchRadians(orientation, true));
//    setRollRadians(getRollRadians(orientation, true));

    mYaw = getYawRadians(orientation, true);
    mPitch = getPitchRadians(orientation, true);
    mRoll = getRollRadians(orientation, true);

    this->timestamp = timestamp;

    if(timestamp == 0) this->timestamp = getCurrentGpsTowTime();
}

Pose::Pose(const QVector3D &position, const float &yaw, const float &pitch, const float &roll, const quint32& timestamp)
{
    this->position = position;
//    setYawRadians(yaw);
//    setPitchRadians(pitch);
//    setRollRadians(roll);

    mYaw = yaw;
    mPitch = pitch;
    mRoll = roll;

    this->timestamp = timestamp;

    if(timestamp == 0) this->timestamp = getCurrentGpsTowTime();
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
Pose Pose::interpolateCubic(const Pose * const first, const Pose * const before, const Pose * const after, const Pose * const last, const float &mu)
{ //                                      y0                 y1                 y2                 y3
    Q_ASSERT(mu <= 0.0 && mu <= 1.0);

    const double mu2 = mu*mu;

    // position
    QVector3D po0, po1, po2, po3;
    po0 = last->position - after->position - first->position + before->position;
    po1 = first->position - before->position - po0;
    po2 = after->position - first->position;
    po3 = before->position;

    QVector3D resultPosition = po0*mu*mu2+po1*mu2+po2*mu+po3;

    // yaw
    float  y0, y1, y2, y3;
    y0 = last->mYaw - after->mYaw - first->mYaw + before->mYaw;
    y1 = first->mYaw - before->mYaw - y0;
    y2 = after->mYaw - first->mYaw;
    y3 = before->mYaw;

    const float yaw = y0*mu*mu2+y1*mu2+y2*mu+y3;

    // pitch
    float  p0, p1, p2, p3;
    p0 = last->mPitch - after->mPitch - first->mPitch + before->mPitch;
    p1 = first->mPitch - before->mPitch - p0;
    p2 = after->mPitch - first->mPitch;
    p3 = before->mPitch;

    const float pitch = p0*mu*mu2+p1*mu2+p2*mu+p3;

    // roll
    float  r0, r1, r2, r3;
    r0 = last->mRoll - after->mRoll - first->mRoll + before->mRoll;
    r1 = first->mRoll - before->mRoll - r0;
    r2 = after->mRoll - first->mRoll;
    r3 = before->mRoll;

    const float roll = r0*mu*mu2+r1*mu2+r2*mu+r3;

    // roll
    float  t0, t1, t2, t3;
    t0 = last->timestamp - after->timestamp - first->timestamp + before->timestamp;
    t1 = first->timestamp - before->timestamp - t0;
    t2 = after->timestamp - first->timestamp;
    t3 = before->timestamp;

    const float timestamp = t0*mu*mu2+t1*mu2+t2*mu+t3;

    return Pose(resultPosition, yaw, pitch, roll, timestamp);
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

    QVector2D result(x, y);
    qDebug() << "Pose::getPlanarDirection(): angle" << getYawDegrees() << "result" << result;
    return result;
}

// unused
Pose Pose::operator*(const float &factor)
{
    return Pose(/*position * factor, orientation * factor*/);
}

// No idea whether the order of orientation is correct
const QQuaternion Pose::getOrientation() const
{
    return
            QQuaternion::fromAxisAndAngle(QVector3D(1,0,0), getPitchDegrees())
            * QQuaternion::fromAxisAndAngle(QVector3D(0,0,1), getRollDegrees())
            * QQuaternion::fromAxisAndAngle(QVector3D(0,1,0), getYawDegrees());
}

Pose Pose::operator+(const Pose &p) const
{
    return Pose(
                position + p.position,
                keepWithinRangeRadians(mYaw + p.getYawRadians()),
                keepWithinRangeRadians(mPitch + p.getPitchRadians()),
                keepWithinRangeRadians(mRoll + p.getRollRadians()),
                (timestamp + p.timestamp)/2
                );
}

/*const Pose Pose::operator+(const Pose &q1, const Pose &q2)
{
    return Pose();
}*/


QDebug operator<<(QDebug dbg, const Pose &pose)
{
    dbg.nospace() << "Position:" << pose.position << "yaw" << pose.getYawDegrees() << "pitch" << pose.getPitchDegrees() << "roll" << pose.getRollDegrees();
    return dbg.maybeSpace();
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
