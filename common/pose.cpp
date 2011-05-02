#include "pose.h"

Pose::Pose(const QVector3D &position, const QQuaternion &orientation, const quint32& timestamp)
{
    this->orientation = orientation;
    this->position = position;

    this->timestamp = timestamp;

    if(timestamp == 0) this->timestamp = getCurrentGpsTowTime();
}

Pose::Pose(const QVector3D &position, const float &yaw, const float &pitch, const float &roll, const quint32& timestamp)
{
    this->position = position;
    this->yaw = yaw;
    this->pitch = pitch;
    this->roll = roll;

    this->timestamp = timestamp;

    if(timestamp == 0) this->timestamp = getCurrentGpsTowTime();
}

Pose::Pose()
{
    this->timestamp = 0;
}

quint32 Pose::getCurrentGpsTowTime()
{
    const QDate today = QDate::currentDate();
    QDateTime beginningOfWeek(today.addDays(-(today.dayOfWeek() % 7)), QTime(0, 0, 0, 0));

    qDebug() << beginningOfWeek.toString("ddd hh:mm:ss:zzz");
    Q_ASSERT(beginningOfWeek.date().dayOfWeek() == Qt::Sunday && beginningOfWeek.toString("hh:mm:ss:zzz") == QString("00:00:00:000"));

    return beginningOfWeek.msecsTo(QDateTime::currentDateTime());
}

Pose Pose::interpolateLinear(const Pose &before, const Pose &after, const float &mu)
{
    Q_ASSERT(mu <= 0.0 && mu <= 1.0);

    return Pose(
                before.position * (1.0 - mu) + after.position * mu,
                before.orientation * (1.0 - mu) + after.orientation * mu
                );
}

// http://paulbourke.net/miscellaneous/interpolation/
Pose Pose::interpolateCubic(const Pose * const first, const Pose * const before, const Pose * const after, const Pose * const last, const float &mu)
{ //                                      y0                 y1                 y2                 y3
    Q_ASSERT(mu <= 0.0 && mu <= 1.0);

    const double mu2 = mu*mu;

    // position
    QVector3D p0, p1, p2, p3;
    p0 = last->position - after->position - first->position + before->position;
    p1 = first->position - before->position - p0;
    p2 = after->position - first->position;
    p3 = before->position;

    QVector3D resultPosition = p0*mu*mu2+p1*mu2+p2*mu+p3;

    // orientation
    QQuaternion  o0, o1, o2, o3;
    o0 = last->orientation - after->orientation - first->orientation + before->orientation;
    o1 = first->orientation - before->orientation - o0;
    o2 = after->orientation - first->orientation;
    o3 = before->orientation;

    QQuaternion resultOrientation = o0*mu*mu2+o1*mu2+o2*mu+o3;

    return Pose(resultPosition, resultOrientation);
}

// unused
Pose Pose::operator*(const float &factor)
{
    return Pose(position * factor, orientation * factor);
}


//const Pose Pose::operator+(const Pose & p)
//{
//    return Pose(position + p.position, orientation + p.orientation);
//}

Pose* Pose::operator+(const Pose &p)
{
    position += p.position;
    orientation += p.orientation;
    return this;
}

/*const Pose Pose::operator+(const Pose &q1, const Pose &q2)
{
    return Pose();
}*/


QDebug operator<<(QDebug dbg, const Pose &pose)
{
    dbg.nospace() << "Pose: Position:" << pose.position << "Orientation:" << pose.orientation;
    return dbg.maybeSpace();
}

/*
QQuaternion Pose::getOrientation(void) const
{
    return orientation;
}

QVector3D Pose::getPosition(void) const
{
    return position;
}*/

QDataStream& operator<<(QDataStream &out, const Pose &pose)
{
    out << pose.position << pose.orientation << pose.yaw << pose.pitch << pose.roll << pose.timestamp;
    return out;
}

QDataStream& operator>>(QDataStream &in, Pose &pose)
{
    in >> pose.position;
    in >> pose.orientation;
    in >> pose.yaw;
    in >> pose.pitch;
    in >> pose.roll;
    in >> pose.timestamp;
    return in;
}

float Pose::getRollRadians(bool reprojectAxis) const
{
    if (reprojectAxis)
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

float Pose::getPitchRadians(bool reprojectAxis) const
{
    if (reprojectAxis)
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

float Pose::getYawRadians(bool reprojectAxis) const
{
    if (reprojectAxis)
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
