/*
 * =====================================================================================
 *
 *       Filename:  BtOgrePG.h
 *
 *    Description:  The part of BtOgre that handles information transfer from Bullet to
 *                  Ogre (like updating graphics object positions).
 *
 *        Version:  1.0
 *        Created:  27/12/2008 03:40:56 AM
 *
 *         Author:  Nikhilesh (nikki)
 *
 * =====================================================================================
 */

#ifndef _BtOgreGP_H_
#define _BtOgreGP_H_

#include <QObject>

#include "btBulletDynamicsCommon.h"
#include "OgreSceneNode.h"
#include <Terrain/OgreTerrainGroup.h>
#include "BtOgreExtras.h"

namespace BtOgre {

// A MotionState is Bullet's way of informing you about updates to an object.
// Pass this MotionState to a btRigidBody to have your SceneNode updated automaticaly.
// Extended by ben to handle the new Ogre Terrain
class RigidBodyState : public QObject, public btMotionState
{
    Q_OBJECT

    protected:
        btTransform mTransform;
        btTransform mCenterOfMassOffset;

        Ogre::SceneNode *mNode;
        Ogre::TerrainGroup *mTerrainGroup;

    public:
        RigidBodyState(Ogre::SceneNode *node, const btTransform &transform, const btTransform &offset = btTransform::getIdentity())
            : mNode(node),
              mTransform(transform),
              mCenterOfMassOffset(offset)
        {
        }

        RigidBodyState(Ogre::SceneNode *node) :
                mNode(node),
                mTerrainGroup(0),
                mTransform(((node != NULL) ? BtOgre::Convert::toBullet(node->getOrientation()) : btQuaternion(0,0,0,1)),
                           ((node != NULL) ? BtOgre::Convert::toBullet(node->getPosition())    : btVector3(0,0,0))),
                mCenterOfMassOffset(btTransform::getIdentity())
        {
        }

        RigidBodyState(Ogre::TerrainGroup *group) :
                mNode(0),
                mTerrainGroup(group),
                mTransform(btQuaternion(0,0,0,1), ((group != NULL) ? BtOgre::Convert::toBullet(group->getOrigin()) : btVector3(0,0,0))),
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
            else if(mTerrainGroup)
            {
                mTerrainGroup->setOrigin(BtOgre::Convert::toOgre(pos));
                emit newPose(mTerrainGroup->getOrigin(), Ogre::Quaternion::IDENTITY);
            }
            else
            {
                assert(false);
            }
        }

        void setNode(Ogre::SceneNode *node)
        {
            mNode = node;
        }

        void setTerrainGroup(Ogre::TerrainGroup *group)
        {
            mTerrainGroup = group;
        }

    signals:
        void newPose(const Ogre::Vector3 pos, const Ogre::Quaternion rot);
};

//Softbody-Ogre connection goes here!

}

#endif
