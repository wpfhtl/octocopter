#ifndef POSE_H
#define POSE_H

#include <QVector3D>
#include <QQuaternion>
#include <QDebug>

class Pose// : public QObject
{
//    Q_OBJECT
private:

public:
    Pose(const QVector3D &position, const QQuaternion &orientation);
    Pose();


    QQuaternion orientation;
    QVector3D position;

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

signals:

public slots:

};

// for using qDebug() << myPose;
QDebug operator<<(QDebug dbg, const Pose &pose);

#endif // POSE_H
