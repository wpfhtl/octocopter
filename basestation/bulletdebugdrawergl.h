#ifndef BULLETDEBUGDRAWERGL_H
#define BULLETDEBUGDRAWERGL_H

#include <btBulletDynamicsCommon.h>
#include "openglutilities.h"

#include <QDebug>

class BulletDebugDrawerGl : public btIDebugDraw
{
protected:
//    Ogre::SceneNode *mNode;
//    btDynamicsWorld *mWorld;
//    DynamicLines *mLineDrawer;
    bool mDebugOn;

public:

    BulletDebugDrawerGl();
    ~BulletDebugDrawerGl();

    void drawLine(const btVector3& from,const btVector3& to,const btVector3& color);
    void drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color);
    void reportErrorWarning(const char* warningString);
    void draw3dText(const btVector3& location,const char* textString);

    void drawSphere (btScalar radius, const btTransform &transform, const btVector3 &color);
    void drawSphere (const btVector3 &p, btScalar radius, const btVector3 &color);

    void drawAabb (const btVector3 &from, const btVector3 &to, const btVector3 &color);
    void drawBox(const btVector3 &min, const btVector3 &max, const btVector3 &color);

    //0 for off, anything else for on.
    void setDebugMode(int isOn);

    //0 for off, anything else for on.
    int getDebugMode() const;
};

#endif // BULLETDEBUGDRAWERGL_H
