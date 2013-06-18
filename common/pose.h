#ifndef POSE_H
#define POSE_H

#include <QVector3D>
#include <QVector2D>
#include <QQuaternion>
#include <QMatrix4x4>
#include <QTime>
#include <QStringList>
#include <QDebug>

#ifdef COMPILED_WITH_BULLET
  #include <bullet/btBulletDynamicsCommon.h>
#endif

#include <gnsstime.h>
#include <common.h>
#include <math.h>
#include <float.h>

/**
  A pose represents a position in 3D space (3 DOF) and an orientation (also 3 DOF: yaw, pitch, roll).
  What exactly a pose is relative to is not relevant. This class uses a right-handed coordinate system
  like OpenGL, so for our vehicle this means:

    - positive yaw turns CCW as seen from the top.
    - positive pitch turns CCW as seen from the right / pilot pulls up.
    - positive roll turns CCW as seen from the back / pilot rotates left wing hanging down

  As a pose also represents a transformation (from origin to current position and orientation,
  we need to agree that translation always comes first, then rotation.

  Also, when we yaw, pitch, roll, we interpret these operations like a pilot of the vehicle, that is
  we pitch on the transformed/yawed x axis, and we roll on the twice-transformed (yawed and pitched)
  z axis. This *should* match the readings of our IMUs, which obviously senses local transformation data.

  A Pose also contains a signed int32 timestamp which usually identifies the time at which some
  sensor reading was taken. Its signed so we can compare times and get negative values indicating one
  time was before the other. The readings themselves will always be positive values.

  Assuming that we rotate on Y CCW (right-handed coordinate system):
   - 000 deg == NEGATIVE_Z
   - 090 deg == NEGATIVE_X
   - 180 deg == POSITIVE_Z
   - 270 deg == POSITIVE_X

*/

class Pose
{
private:
    QMatrix4x4 mTransform;
    QVector3D mVelocity; // m/s

public:
    Pose(const QVector3D &position, const float &yawDegrees, const float &pitchDegrees, const float &rollDegrees, const qint32& timestamp = 0);
    Pose(const QMatrix4x4& matrix, const qint32& timestamp = 0);
    Pose(const QString& poseString);
    Pose();

    // untested!
    static float getAngleBetweenDegrees(const Pose& p1, const Pose& p2);
    static float getAngleBetweenDegrees(const QQuaternion& p1, const QQuaternion& p2);

    // This is the GPS Time-Of-Week, specified in milliseconds since last Sunday, 00:00:00 AM (midnight)
    // Changed this from quint32 to qint32 for comparison operations.
    qint32 timestamp;
    float covariances;
    quint8 precision;

    // These scalar members were added to debug/test interpolation accuracies and their applciability to pointcloud registration
    float acceleration; // m/s^2
    float rotation; // deg/s

    // Usually, maximum covariances converge to about 0.02.
    static constexpr float maximumUsableCovariance = 0.05f;

    void setVelocity(const QVector3D& velocity)
    {
        mVelocity = velocity;
//        qDebug() << "Pose::setVelocity():" << mVelocity;
    }
    const QVector3D& getVelocity() const {return mVelocity;}


    enum Precision
    {
        RtkFixed = 1,
        HeadingFixed = 2,
        ModeIntegrated = 4,
        AttitudeAvailable = 8,
        CorrectionAgeLow = 16
    };

    const QString getFlagsString() const;

    // How old os this pose in milliseconds? Requires a synchronized system clock.
    qint32 getAge() const { return GnssTime::currentTow() - timestamp; }

    const QString toString(bool verbose = false) const; // verbose includes flags-string

    const QVector3D getPosition() const;
    const QQuaternion getOrientation() const;

//    static Pose extrapolateLinear(const Pose &first, const Pose &second, const quint32 &timeAfter);

    static Pose extrapolateLinear(const Pose &p1, const Pose &p2, const qint32 &timeInFuture);

    static Pose interpolateLinear(const Pose &p0, const Pose &p1, const qint32 &time);

    // Returns a pose between @before and @after, also needs @first and @last, as its bicubic. @mu specifies the position between @before and @after.
//    static Pose interpolateCubic(const Pose * const p0, const Pose * const p1, const Pose * const p2, const Pose * const p3, const float &mu);
    static QVector3D interpolateCubic(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2, const QVector3D& p3, const float mu);

    // Returns a pose between @before and @after, also needs @first and @last, as its bicubic.
    // This method takes a time argument instead of a float. @time must be between the times of @before and @after poses
    static Pose interpolateCubic(const Pose * const p0, const Pose * const p1, const Pose * const p2, const Pose * const p3, const qint32& time);

    // Returns the shortest turn to an angle by transforming any input to the range [-180,180].
    // For example: 359 => -1, 181 => -179, -270 => 90
    static float getShortestTurnRadians(float angle);
    static float getShortestTurnDegrees(float angle);

    bool isSufficientlyPreciseForFlightControl() const;

    void getEulerAnglesRadians(float& yaw, float &pitch, float &roll) const;
    void getEulerAnglesDegrees(float& yaw, float &pitch, float &roll) const;

    Pose operator*(const Pose &p) const
    {
        const QMatrix4x4 newTransform = mTransform * p.getMatrixConst();
        Pose r(newTransform, std::max(timestamp, p.timestamp));

        // just for testing, remove in the future!
        r.rotation = rotation;
        r.acceleration = acceleration;
        r.mVelocity = mVelocity;
        r.covariances = covariances;
        r.precision = precision;

        return r;
    }

    static void rotationToAngleAxis(const QQuaternion &q, float& angleDegrees, QVector3D& axis);

    static QQuaternion inverse(const QQuaternion& q);

    QVector3D operator*(const QVector3D &v) const
    {
        return mTransform.map(v);
    }

    // a.k.a. squad
    static QQuaternion interpolateCubic(const QQuaternion& rkP, const QQuaternion& rkA, const QQuaternion& rkB, const QQuaternion& rkQ, const float mu)
    {
        float fSlerpT = 2.0f*mu*(1.0f-mu);
        QQuaternion kSlerpP = QQuaternion::slerp(rkP, rkQ, mu);
        QQuaternion kSlerpQ = QQuaternion::slerp(rkA, rkB, mu);
        return QQuaternion::slerp(kSlerpP ,kSlerpQ, fSlerpT);
    }

    bool operator<(const Pose& p) const { return this->timestamp < p.timestamp; }

    QVector2D getPlanarPosition() const;

    const QMatrix4x4& getMatrixConst() const {return mTransform;}
    QMatrix4x4& getMatrixRef() {return mTransform;}
    QMatrix4x4 getMatrixCopy() const {return mTransform;}

    void setMatrix(const QMatrix4x4& transform) {mTransform = transform;}

    static QVector2D getPlanarPosition(const Pose& p) { return QVector2D(p.getPosition().x(), p.getPosition().z()); }
    static QVector2D getPlanarPosition(const QVector3D& p) { return QVector2D(p.x(), p.z()); }

    float getYawRadians() const
    {
        float yaw, pitch, roll;
        getEulerAnglesRadians(yaw, pitch, roll);
        return yaw;
    }
    float getPitchRadians() const
    {
        float yaw, pitch, roll;
        getEulerAnglesRadians(yaw, pitch, roll);
        return pitch;
    }
    float getRollRadians() const
    {
        float yaw, pitch, roll;
        getEulerAnglesRadians(yaw, pitch, roll);
        return roll;
    }
    float getYawDegrees() const {return RAD2DEG(getYawRadians());}
    float getPitchDegrees() const {return RAD2DEG(getPitchRadians());}
    float getRollDegrees() const {return RAD2DEG(getRollRadians());}

#ifdef BASESTATION
    btTransform getTransform() const
    {
        float yaw, pitch, roll;
        getEulerAnglesRadians(yaw, pitch, roll);
        const QVector3D position = getPosition();
        return btTransform(btQuaternion(yaw, pitch, roll), btVector3(position.x(), position.y(), position.z()));
    }
#endif
};

// for using qDebug()
QDebug operator<<(QDebug dbg, const Pose &pose);

// for streaming
QDataStream& operator<<(QDataStream &out, const Pose &pose);
QDataStream& operator>>(QDataStream &in, Pose &pose);

#endif // POSE_H
