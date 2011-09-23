#ifndef POSE_H
#define POSE_H

#include <QVector3D>
#include <QQuaternion>
#include <QTime>
#include <QDebug>

#ifdef BASESTATION
  #include <bullet/btBulletDynamicsCommon.h>
#endif

#include <common.h>
#include <math.h>

class Pose// : public QObject
{
//    Q_OBJECT
private:
    float mYaw, mPitch, mRoll; // angle in RADIANS!

public:
    Pose(const QVector3D &position, const QQuaternion &orientation, const quint32& timestamp = 0);
    Pose(const QVector3D &position, const float &yaw, const float &pitch, const float &roll, const quint32& timestamp = 0);
    Pose();

    QVector3D position;
    const QQuaternion getOrientation() const;

    // This is the GPS Time-Of-Week, specified in milliseconds since last Sunday, 00:00:00 AM (midnight)
    quint32 timestamp;

    static Pose interpolateLinear(const Pose &before, const Pose &after, const float &mu);

    // Returns a pose between @before and @after, also needs @first and @last, as its bicubic. @mu specifies the position between @before and @after.
    static Pose interpolateCubic(const Pose * const first, const Pose * const before, const Pose * const after, const Pose * const last, const float &mu);

    // Returns a pose between @before and @after, also needs @first and @last, as its bicubic.
    // This method takes a time argument instead of a float. @time must be between the times of @before and @after poses
    static Pose interpolateCubic(const Pose * const first, const Pose * const before, const Pose * const after, const Pose * const last, const quint32& time);

    // Returns the shortest turn to an angle. For example:
    // 359 => -1
    // 181 => -179
    // -270 => 90
    static float getShortestTurnRadians(const float& angle);
    static float getShortestTurnDegrees(const float& angle);
    static float keepWithinRangeRadians(float angleRadians);

    Pose operator*(const float &factor);
//    Pose operator*(const float &factor);
    const Pose operator-(const Pose &p) const;

    Pose operator+(const Pose &p) const;
//    friend inline const Pose operator+(const Pose &q1, const Pose &q2);

    QVector2D getPlanarPosition() const;

    // Assuming that:
    // - 000 deg == NEGATIVE_Z
    // - 090 deg == NEGATIVE_X
    // - 180 deg == POSITIVE_Z
    // - 270 deg == POSITIVE_X
    QVector2D getPlanarDirection() const;






    static float getPitchRadians(const QQuaternion& orientation, bool reprojectAxis);
    static float getRollRadians(const QQuaternion& orientation, bool reprojectAxis);
    static float getYawRadians(const QQuaternion& orientation, bool reprojectAxis);

    float getPitchRadians() const {return mPitch;}
    float getRollRadians() const {return mRoll;}
    float getYawRadians() const {return mYaw;}

    float getPitchDegrees() const {return RAD2DEG(mPitch);}
    float getRollDegrees() const {return RAD2DEG(mRoll);}
    float getYawDegrees() const {return RAD2DEG(mYaw);}

    void setPitchRadians(const float& pitch) {mPitch = /*normalizeAngleRadians*/(pitch);}
    void setRollRadians(const float& roll) {mRoll = /*normalizeAngleRadians*/(roll);}
    void setYawRadians(const float& yaw) {mYaw = /*normalizeAngleRadians*/(yaw);}

#ifdef BASESTATION
    btTransform getTransform() const {return btTransform(btQuaternion(mYaw, mPitch, mRoll), btVector3(position.x(), position.y(), position.z()));}
#endif


signals:

public slots:

};

// for using qDebug() << myPose;
QDebug operator<<(QDebug dbg, const Pose &pose);

// for streaming
QDataStream& operator<<(QDataStream &out, const Pose &pose);
QDataStream& operator>>(QDataStream &in, Pose &pose);

#endif // POSE_H
