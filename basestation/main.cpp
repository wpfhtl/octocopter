#include "basestation.h"
#include <bullet/btBulletDynamicsCommon.h>

#include <QApplication>
#include <QIcon>

int main(int argc, char *argv[])
{
    // Having OGRE or bullet use double precision means nothing but trouble.
//    qDebug() << "main(): sizeof: btScalar" << sizeof(btScalar);
    Q_ASSERT(sizeof(btScalar) == 4 && "Please make sure Bullet is compiled to use single precision");

    QApplication app(argc, argv);
    app.setWindowIcon(QIcon(":/images/appicon.png"));

    QVector3D peter(0,1,0);
    QMatrix4x4 peterRot;

    // rückwwärts ausführen
    peterRot.rotate(45, QVector3D(0,1,0));
    peterRot.rotate(90, QVector3D(1,0,0));
    peterRot.rotate(90, QVector3D(0,0,1));

    qDebug() <<
                ((
                    QQuaternion::fromAxisAndAngle(QVector3D(0,1,0), 45)
*                QQuaternion::fromAxisAndAngle(QVector3D(1,0,0), 90)
*
                    QQuaternion::fromAxisAndAngle(QVector3D(0,0,1), 90))
    .rotatedVector(peter));

    qDebug() << peterRot.map(peter); // Achsen drehen sich mit 001
    qDebug() << peterRot.mapVector(peter); // Achsen drehen sich mit
    qDebug() << peterRot * peter; // Achsen drehen sich mit
//    qDebug() << peter * peterRot.transposed(); // Axen bleiben beim rotieren fest -0,7 0 -0,7


    BaseStation b;
    b.show();

    return app.exec();
}
