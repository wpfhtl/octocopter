#ifndef _BtOgrePG_H
#define _BtOgrePG_H

#include <QObject>

#include "btBulletDynamicsCommon.h"
#include "OgreQuaternion.h"
#include "OgreSceneNode.h"
#include "BtOgreExtras.h"
#include <pose.h>

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

            if(mNode)
            {
                btTransform transform = in * mCenterOfMassOffset;
                const btQuaternion rotBt = transform.getRotation();
                const btVector3 posBt = transform.getOrigin();

                Ogre::Quaternion rotO(rotBt.w(), rotBt.x(), rotBt.y(), rotBt.z());

                if(rotO.isNaN())
                    rotO = Ogre::Quaternion::IDENTITY;

                mNode->setOrientation(rotO);

                Ogre::Vector3 posO(posBt.x(), posBt.y(), posBt.z());

                if(posO.isNaN())
                    posO = Ogre::Vector3(0.0f, 0.0f ,0.0f);

                mNode->setPosition(posO);

//                qDebug() << "RigidBodyState::setWorldTransform(): emitting new pose";
//                emit newPose(getPose());
            }
        }

        const Pose getPose()
        {
            Ogre::Matrix3 mat;
            mNode->_getDerivedOrientation().ToRotationMatrix(mat);

            Ogre::Radian yaw, pitch, roll;
            mat.ToEulerAnglesYXZ(yaw, pitch, roll);

            btVector3 posBt = (mTransform * mCenterOfMassOffset).getOrigin();

            Pose p(
                        QVector3D(posBt.x(), posBt.y(), posBt.z()),
                        //fmod(yaw.valueRadians() + Ogre::Degree(360.0).valueRadians(), 360.0*M_PI/180.0),
                        fmod(yaw.valueDegrees() + Ogre::Degree(360.0).valueDegrees(), 360.0),
                        pitch.valueDegrees(),
                        roll.valueDegrees()
                        );

            // Whee, we are precise!!
            p.precision = Pose::ModeIntegrated | Pose::AttitudeAvailable | Pose::HeadingFixed | Pose::RtkFixed | Pose::CorrectionAgeLow;

            return p;
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
//        void newPose(const Pose& pose);
};

//Softbody-Ogre connection goes here!

}

#endif
