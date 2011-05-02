#ifndef POSE_H
#define POSE_H

#include <QVector3D>
#include <QQuaternion>
#include <QTime>
#include <QDebug>

#include <math.h>

class Pose// : public QObject
{
//    Q_OBJECT
private:

public:
    Pose(const QVector3D &position, const QQuaternion &orientation, const quint32& timestamp = 0);
    Pose(const QVector3D &position, const float &yaw, const float &pitch, const float &roll, const quint32& timestamp = 0);
    Pose();


    QVector3D position;
    QQuaternion orientation;
    float yaw, pitch, roll;

    // This is the GPS Time-Of-Week, specified in milliseconds since last Sunday, 00:00:00 AM (midnight)
    quint32 timestamp;

    static Pose interpolateLinear(const Pose &before, const Pose &after, const float &mu);

    // Returns a pose between @before and @after, also needs @first and @last, as its bicubic
    static Pose interpolateCubic(const Pose * const first, const Pose * const before, const Pose * const after, const Pose * const last, const float &mu);

    Pose operator*(const float &factor);
//    Pose operator*(const float &factor);
//    const Pose operator+(const Pose & p);

    Pose* operator+(const Pose &p);
//    friend inline const Pose operator+(const Pose &q1, const Pose &q2);

//    QQuaternion getOrientation(void) const;
//    QVector3D getPosition(void) const;

    static quint32 getCurrentGpsTowTime();

    float getRollRadians(bool reprojectAxis) const;
    float getPitchRadians(bool reprojectAxis) const;
    float getYawRadians(bool reprojectAxis) const;

    float getRollDegrees(bool reprojectAxis) const {return 180.0 * getRollRadians(reprojectAxis) / M_PI;};
    float getPitchDegrees(bool reprojectAxis) const {return 180.0 * getPitchRadians(reprojectAxis) / M_PI;};
    float getYawDegrees(bool reprojectAxis) const {return 180.0 * getYawRadians(reprojectAxis) / M_PI;};

signals:

public slots:

};

// for using qDebug() << myPose;
QDebug operator<<(QDebug dbg, const Pose &pose);

// for streaming
QDataStream& operator<<(QDataStream &out, const Pose &pose);
QDataStream& operator>>(QDataStream &in, Pose &pose);

#endif // POSE_H
