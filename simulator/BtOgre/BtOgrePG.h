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
        quint32 mPoseCounter;

        // used to calculate velocities
        QTime mPreviousPoseTime;
        btVector3 mPreviousPosePosition;

        Ogre::SceneNode *mNode;

    public:
        RigidBodyState(Ogre::SceneNode *node, const btTransform &transform, const btTransform &offset = btTransform::getIdentity())
            : mNode(node),
              mTransform(transform),
              mCenterOfMassOffset(offset),
              mPoseCounter(0)
        {
        }

        RigidBodyState(Ogre::SceneNode *node) :
                mNode(node),
                mTransform(((node != NULL) ? BtOgre::Convert::toBullet(node->getOrientation()) : btQuaternion(0,0,0,1)),
                           ((node != NULL) ? BtOgre::Convert::toBullet(node->getPosition())    : btVector3(0,0,0))),
                mCenterOfMassOffset(btTransform::getIdentity()),
                mPoseCounter(0)
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

//                emit newPose(getPose());
            }
        }

        const Pose getPose()
        {
            Ogre::Matrix3 mat;
            mNode->_getDerivedOrientation().ToRotationMatrix(mat);

            Ogre::Radian yaw, pitch, roll;
            mat.ToEulerAnglesYXZ(yaw, pitch, roll);

            btVector3 posePosition = (mTransform * mCenterOfMassOffset).getOrigin();

            float timeDiff = mPreviousPoseTime.msecsTo(QTime::currentTime()) / 1000.0f;
            mPreviousPoseTime = QTime::currentTime();

            QVector3D velocity(
                        (posePosition.x() - mPreviousPosePosition.x()) / timeDiff,
                        (posePosition.y() - mPreviousPosePosition.y()) / timeDiff,
                        (posePosition.z() - mPreviousPosePosition.z()) / timeDiff);

            mPreviousPosePosition = posePosition;

            Pose p(
                        QVector3D(posePosition.x(), posePosition.y(), posePosition.z()),
                        //fmod(yaw.valueRadians() + Ogre::Degree(360.0).valueRadians(), 360.0*M_PI/180.0),
                        fmod(yaw.valueDegrees() + Ogre::Degree(360.0).valueDegrees(), 360.0),
                        pitch.valueDegrees(),
                        roll.valueDegrees()
                        );

            p.setVelocity(velocity);

            // Whee, we are precise!!
            p.precision = Pose::AttitudeAvailable | Pose::HeadingFixed | Pose::RtkFixed | Pose::CorrectionAgeLow;

            // Every second pose is integrated. I'd prefer to update the counter in setTRansform(), but that is called
            // multiple times per physics iteration. Often, its two times, making this mechanism useless.
            if(mPoseCounter++ % 2 == 0) p.precision |= Pose::ModeIntegrated;


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
