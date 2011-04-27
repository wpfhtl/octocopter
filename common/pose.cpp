#include "pose.h"

Pose::Pose(const QVector3D &position, const QQuaternion &orientation, const quint32& timestamp)
{
    this->orientation = orientation;
    this->position = position;

    this->timestamp = timestamp;
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
    out << pose.position << pose.orientation << pose.timestamp;
    return out;
}

QDataStream& operator>>(QDataStream &in, Pose &pose)
{
    in >> pose.position;
    in >> pose.orientation;
    in >> pose.timestamp;
    return in;
}
