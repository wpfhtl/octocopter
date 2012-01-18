#ifndef POSE_H
#define POSE_H

#include <QVector3D>
#include <QVector2D>
#include <QQuaternion>
#include <QMatrix4x4>
#include <QTime>
#include <QStringList>
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

  A Pose also contains a signed int32 timestamp which usually identifies the time at which some
  sensor reading was taken. Its signed so we can compare times and get negative values indicating one
  time was before the other. Teh readings themselves will always be positive values.
*/

class Pose
{

private:
    float mYaw, mPitch, mRoll;  // Angles in RADIANS!

public:
    Pose(const QVector3D &position, const QQuaternion &orientation, const qint32& timestamp = 0);
    Pose(const QVector3D &position, const float &yawDegrees, const float &pitchDegrees, const float &rollDegrees, const qint32& timestamp = 0);
    Pose(const QString& poseString);
    Pose();

    const QString toString() const;

    QVector3D position;
    const QQuaternion getOrientation() const;

    // This is the GPS Time-Of-Week, specified in milliseconds since last Sunday, 00:00:00 AM (midnight)
    // Ben: changed this from quint32 to qint32 for comparison operations.
    qint32 timestamp;

    static Pose interpolateLinear(const Pose &before, const Pose &after, const float &mu);

    // Returns a pose between @before and @after, also needs @first and @last, as its bicubic. @mu specifies the position between @before and @after.
    static Pose interpolateCubic(const Pose * const first, const Pose * const before, const Pose * const after, const Pose * const last, const float &mu);

    // Returns a pose between @before and @after, also needs @first and @last, as its bicubic.
    // This method takes a time argument instead of a float. @time must be between the times of @before and @after poses
    static Pose interpolateCubic(const Pose * const first, const Pose * const before, const Pose * const after, const Pose * const last, const qint32& time);

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

// for streaming poses
QDataStream& operator<<(QDataStream &out, const Pose &pose);
QDataStream& operator>>(QDataStream &in, Pose &pose);






// This belongs to laserscanner, but we don't want to introduce a laserscanner-dependency into basestation
// This is defined in header to make it inlineable
inline QVector3D getWorldPositionOfScannedPoint(const Pose& scannerPose, const quint16& scannerIndex, const float& distance)
{
#ifdef false
    // Version using QMatrix4x4
    const QVector3D vectorScannerToPoint(
                sin(-0.0043633231299858238686f * (scannerIndex - 540)) * distance,  // X in meters
                0.0,                                                                // Y always 0
                cos(-0.0043633231299858238686f * (scannerIndex - 540)) * distance); // Z in meters

    QMatrix4x4 scannerOrientation;
    scannerOrientation.rotate(scannerPose.getYawDegrees(), QVector3D(0,1,0));
    scannerOrientation.rotate(scannerPose.getPitchDegrees(), QVector3D(1,0,0));
    scannerOrientation.rotate(scannerPose.getRollDegrees(), QVector3D(0,0,1));

    return scannerPose.position + scannerOrientation * vectorScannerToPoint;
#else
    // This is short, but uses getOrientation(), which is expensive.
    return QVector3D(
                scannerPose.position
                + scannerPose.getOrientation().rotatedVector(
                    QVector3D(
                        sin(-0.0043633231299858238686f * (scannerIndex - 540)) * distance,  // X in meters
                        0.0,                                                                // Y always 0
                        cos(-0.0043633231299858238686f * (scannerIndex - 540)) * distance   // Z in meters
                        )
                    )
                );
#endif

    /* Elaborate version, slightly slower?!

    // Determine vector from LaserScanner to scanned point, using the normal OpenGl coordinate system as seen from scanner,
    // +x is right, -x is left, y is always 0, +z is back, -z is front,
    // We could use the UrgCtrl::index2rad(), but this seems a bit more optimized
    const QVector3D vectorScannerToPoint(
                sin(-index2rad(scannerIndex)) * distance,  // X in meters
                0.0,                                                // Y always 0
                cos(-index2rad(scannerIndex)) * distance); // Z in meters

    // Or, even more optimized, as we cannot rely on gcc actually inlining what it marked as inline


    // Create a scanpoint
    const QVector3D scannedPoint(scannerPose.position + scannerPose.getOrientation().rotatedVector(vectorScannerToPoint));

//    qDebug() << "LaserScanner::getWorldPositionOfScannedPoint(): interpolated scanner pose is" << scannerPose;
//    qDebug() << "LaserScanner::getWorldPositionOfScannedPoint(): distance is" << distance << "and index" << scannerIndex << "=>" << mScanner.index2deg(scannerIndex) << "degrees";
//    qDebug() << "LaserScanner::getWorldPositionOfScannedPoint(): scanner to point in scanner frame is" << vectorScannerToPoint;
//    qDebug() << "LaserScanner::getWorldPositionOfScannedPoint(): point position   in world   frame is" << scannedPoint;

    return scannedPoint;
    */
}




#endif // POSE_H
