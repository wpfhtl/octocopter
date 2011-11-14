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

/**
  A pose represents a position in 3D space (3 DOF) and an orientation (also 3 DOF: yaw, pitch, roll).
  What exactly a pose is relative to is not relevant. This class uses a right-handed coordinate system
  like OpenGL, so for our vehicle this means:

    - positive yaw turns CCW as seen from the top.
    - positive pitch turns CCW as seen from the right / pulls up.
    - positive roll turns CCW as seen from the back / turns left

  As a pose also represents a transformation (from origin to current position and orientation,
  we need to agree that translation always comes first, then rotation.

  Also, when we yaw, pitch, roll, we interpret these operations like a pilot of the vehicle, that is
  we pitch on the transformed/yawed x axis, and we roll on the twice-transformed (yawed and pitched)
  z axis. This *should* match the readings of our IMUs, who obviously sense local transformation data.

  A Pose also contains an unsigned int32 timestamp which usually identifies the time at which some
  sensor reading was taken.
*/

class Pose
{

private:
    float mYaw, mPitch, mRoll;  // Angles in RADIANS!

public:
    Pose(const QVector3D &position, const QQuaternion &orientation, const quint32& timestamp = 0);
    Pose(const QVector3D &position, const float &yawDegrees, const float &pitchDegrees, const float &rollDegrees, const quint32& timestamp = 0);
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

//    Pose operator*(const float &factor);
//    const Pose operator-(const Pose &p) const;
    Pose operator+(const Pose &p) const;

    QVector2D getPlanarPosition() const;

    // Assuming that we rotate on Y CCW (right-handed coordinate system):
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

    void setPitchDegrees(const float& pitch) {mPitch = DEG2RAD(pitch);}
    void setRollDegrees(const float& roll) {mRoll = DEG2RAD(roll);}
    void setYawDegrees(const float& yaw) {mYaw = DEG2RAD(yaw);}

    void setPitchRadians(const float& pitch) {mPitch = /*normalizeAngleRadians*/(pitch);}
    void setRollRadians(const float& roll) {mRoll = /*normalizeAngleRadians*/(roll);}
    void setYawRadians(const float& yaw) {mYaw = /*normalizeAngleRadians*/(yaw);}

#ifdef BASESTATION
    btTransform getTransform() const {return btTransform(btQuaternion(mYaw, mPitch, mRoll), btVector3(position.x(), position.y(), position.z()));}
#endif

};

// For establishing a total order on poses. Needed for storage in QMap. WRONG, only needed for key, pose is a value.
//inline bool operator<(const Pose &p1, const Pose &p2) {return p1.timestamp < p2.timestamp;}

// for using qDebug() << myPose;
QDebug operator<<(QDebug dbg, const Pose &pose);

// for streaming
QDataStream& operator<<(QDataStream &out, const Pose &pose);
QDataStream& operator>>(QDataStream &in, Pose &pose);

#endif // POSE_H
