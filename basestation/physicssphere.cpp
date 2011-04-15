#include "physicssphere.h"

PhysicsSphere::PhysicsSphere(btDiscreteDynamicsWorld* world, const float radius, QObject *parent) : QObject(parent)
{
    mBtWorld = world;

    btSphereShape *shapeSampleSphere = new btSphereShape(5);

    btTransform sphereTransform;
    sphereTransform.setOrigin(btVector3(x, mScanVolumeMax.y(), z));

    // We don't need any inertia, mass etc,. as this body is static.
    btDefaultMotionState* sampleSphereMotionState = new btDefaultMotionState(mLidarPointTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(1.0f, sampleSphereMotionState, shapeSampleSphere, btVector3(0,0,0));
    btRigidBody* body = new btRigidBody(rbInfo);

    // Add the body to the dynamics world
    mBtWorld->addRigidBody(body);
}

QVector3D PhysicsSphere::getPosition(void) const
{

}
