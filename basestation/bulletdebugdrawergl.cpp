#include "bulletdebugdrawergl.h"

BulletDebugDrawerGl::BulletDebugDrawerGl(/*Ogre::SceneNode *node, btDynamicsWorld *world*/) : mDebugOn(true)
//    mNode(node),
//    mWorld(world),
{
}

BulletDebugDrawerGl::~BulletDebugDrawerGl()
{
}

/*void BulletDebugDrawerGl::step()
{
    if (mDebugOn)
    {
        mWorld->debugDrawWorld();
        mLineDrawer->update();
        mNode->needUpdate();
        mLineDrawer->clear();
    }
    else
    {
        mLineDrawer->clear();
        mLineDrawer->update();
        mNode->needUpdate();
    }
}*/

void BulletDebugDrawerGl::drawLine(const btVector3& from,const btVector3& to,const btVector3& color)
{
//    qDebug() << "BulletDebugDrawerGl::drawLine";
    glLineWidth(1);
    glColor4f(color.x(), color.y(), color.z(), 0.9);

    glBegin(GL_LINES);

    glVertex3f(from.x(), from.y(), from.z());
    glVertex3f(to.x(), to.y(), to.z());

    glEnd();
}

void BulletDebugDrawerGl::drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color)
{
    glLineWidth(2);
    glColor4f(color.x(), color.y(), color.z(), std::max(0.2, 1.0/lifeTime));

    glBegin(GL_LINES);
    glVertex3f(PointOnB.x(), PointOnB.y(), PointOnB.z());
    const btVector3 secondPoint = PointOnB + (normalOnB * distance * 10);
    glVertex3f(secondPoint.x(), secondPoint.y(), secondPoint.z());
    glEnd();
}

void BulletDebugDrawerGl::drawSphere (btScalar radius, const btTransform &transform, const btVector3 &color)
{
    // Old GL immediate mode
//    OpenGlUtilities::drawSphere(QVector3D(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z()), radius, 5.0, QColor(color.x()*255,color.y()*255,color.z()*255, 30));
}

void BulletDebugDrawerGl::drawSphere (const btVector3 &p, btScalar radius, const btVector3 &color)
{
    // Old GL immediate mode
//    OpenGlUtilities::drawSphere(QVector3D(p.x(), p.y(), p.z()), radius, 5.0, QColor(color.x()*255,color.y()*255,color.z()*255));
}

void BulletDebugDrawerGl::drawAabb(const btVector3 &min, const btVector3 &max, const btVector3 &color)
{
    glLineWidth(1);
    glColor4f(color.x(), color.y(), color.z(), 0.5);

    glBegin(GL_LINE_STRIP);
    glVertex3f(min.x(), min.y(), min.z());
    glVertex3f(max.x(), min.y(), min.z());
    glVertex3f(max.x(), max.y(), min.z());
    glVertex3f(min.x(), max.y(), min.z());
    glVertex3f(min.x(), min.y(), min.z());
    glVertex3f(min.x(), min.y(), max.z());
    glVertex3f(max.x(), min.y(), max.z());
    glVertex3f(max.x(), max.y(), max.z());
    glVertex3f(min.x(), max.y(), max.z());
    glVertex3f(min.x(), min.y(), max.z());
    glEnd();

    glBegin(GL_LINES);
    glVertex3f(min.x(), max.y(), min.z());
    glVertex3f(min.x(), max.y(), max.z());
    glVertex3f(max.x(), min.y(), min.z());
    glVertex3f(max.x(), min.y(), max.z());
    glVertex3f(max.x(), max.y(), min.z());
    glVertex3f(max.x(), max.y(), max.z());
    glEnd();
}

void BulletDebugDrawerGl::drawBox(const btVector3 &min, const btVector3 &max, const btVector3 &color)
{
    glLineWidth(1);
    glColor4f(color.x(), color.y(), color.z(), 0.5);

    glBegin(GL_LINE_STRIP);
    glVertex3f(min.x(), min.y(), min.z());
    glVertex3f(max.x(), min.y(), min.z());
    glVertex3f(max.x(), max.y(), min.z());
    glVertex3f(min.x(), max.y(), min.z());
    glVertex3f(min.x(), min.y(), min.z());
    glVertex3f(min.x(), min.y(), max.z());
    glVertex3f(max.x(), min.y(), max.z());
    glVertex3f(max.x(), max.y(), max.z());
    glVertex3f(min.x(), max.y(), max.z());
    glVertex3f(min.x(), min.y(), max.z());
    glEnd();

    glBegin(GL_LINES);
    glVertex3f(min.x(), max.y(), min.z());
    glVertex3f(min.x(), max.y(), max.z());
    glVertex3f(max.x(), min.y(), min.z());
    glVertex3f(max.x(), min.y(), max.z());
    glVertex3f(max.x(), max.y(), min.z());
    glVertex3f(max.x(), max.y(), max.z());
    glEnd();
}

void BulletDebugDrawerGl::reportErrorWarning(const char* warningString)
{
//    emit messageWarning(QString(warningString));
    qDebug() << "BulletDebugDrawerGl::reportErrorWarning():" << warningString;
}

void BulletDebugDrawerGl::draw3dText(const btVector3& location,const char* textString)
{
//    Ogre::LogManager::getSingleton().logMessage(textString);
    qDebug() << "BulletDebugDrawerGl::draw3dText():" << textString;
}

//0 for off, anything else for on.
void BulletDebugDrawerGl::setDebugMode(int isOn)
{
    mDebugOn = (isOn == 0) ? false : true;

//    if (!mDebugOn)
//        mLineDrawer->clear();
}

//0 for off, anything else for on.
int BulletDebugDrawerGl::getDebugMode() const
{
    return mDebugOn;
}
