#ifndef _BtOgreGP_H_
#define _BtOgreGP_H_

#include <QObject>

#include "btBulletDynamicsCommon.h"
#include "OgreSceneNode.h"
#include "BtOgreExtras.h"

namespace BtOgre {

// A MotionState is Bullet's way of informing you about updates to an object.
// Pass this MotionState to a btRigidBody to have your SceneNode updated automaticaly.
class RigidBodyState : public QObject, public btMotionState
{
    Q_OBJECT

    protected:
        btTransform mTransform;
        btTransform mCenterOfMassOffset;

        Ogre::SceneNode *mNode;

    public:
        RigidBodyState(Ogre::SceneNode *node, const btTransform &transform, const btTransform &offset = btTransform::getIdentity())
            : mNode(node),
              mTransform(transform),
              mCenterOfMassOffset(offset)
        {
        }

        RigidBodyState(Ogre::SceneNode *node) :
                mNode(node),
                mTransform(((node != NULL) ? BtOgre::Convert::toBullet(node->getOrientation()) : btQuaternion(0,0,0,1)),
                           ((node != NULL) ? BtOgre::Convert::toBullet(node->getPosition())    : btVector3(0,0,0))),
                mCenterOfMassOffset(btTransform::getIdentity())
        {
        }

        virtual void getWorldTransform(btTransform &ret) const
        {
            ret = mCenterOfMassOffset.inverse() * mTransform;
        }

        virtual void setWorldTransform(const btTransform &in)
        {
            mTransform = in;
            btTransform transform = in * mCenterOfMassOffset;

            btQuaternion rot = transform.getRotation();
            btVector3 pos = transform.getOrigin();

            if(mNode)
            {
                mNode->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
                mNode->setPosition(pos.x(), pos.y(), pos.z());

                emit newPose(mNode->getPosition(), mNode->getOrientation());
            }

        }

        Ogre::Vector3 getPosition(void) const
        {
            btVector3 pos = mTransform.getOrigin();
            return Ogre::Vector3(pos.x(), pos.y(), pos.z());
        }

        Ogre::Quaternion getOrientation(void) const
        {
            btQuaternion rot = mTransform.getRotation();
            return Ogre::Quaternion(rot.w(), rot.x(), rot.y(), rot.z());
        }

        void setNode(Ogre::SceneNode *node)
        {
            mNode = node;
        }

        Ogre::SceneNode* getNode(void)
        {
            return mNode;
        }

    signals:
        void newPose(const Ogre::Vector3 pos, const Ogre::Quaternion rot);
};

//Softbody-Ogre connection goes here!

}

#endif
