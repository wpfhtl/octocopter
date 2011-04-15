#ifndef PHYSICSSPHERE_H
#define PHYSICSSPHERE_H

#include <QObject>
#include <QVector3D>
#include <btBulletDynamicsCommon.h>

// A single collision shape can be shared among multiple collision objects.

class PhysicsSphere : public QObject
{
    Q_OBJECT

    btDiscreteDynamicsWorld* mBtWorld;
    btRigidBody mRigidBody;
    btDefaultMotionState mMotionState;


public:
    PhysicsSphere(btDiscreteDynamicsWorld* world, QObject *parent = 0);
    QVector3D getPosition(void) const;

signals:

public slots:

};

#endif // PHYSICSSPHERE_H
