#include "basestation.h"

#ifdef COMPILED_WITH_BULLET
#include <bullet/btBulletDynamicsCommon.h>
#endif


int main(int argc, char *argv[])
{
    // Having OGRE or bullet use double precision means nothing but trouble.
//    qDebug() << "main(): sizeof: btScalar" << sizeof(btScalar);

#ifdef COMPILED_WITH_BULLET
    Q_ASSERT(sizeof(btScalar) == 4 && "Please make sure Bullet is compiled to use single precision");
#endif

    Q_ASSERT(sizeof(QVector3D) == 12 && "ui.");
    Q_ASSERT(sizeof(QVector4D) == 16 && "ui.");
    Q_ASSERT(sizeof(LidarPoint) == 24 && "lp off");

    QApplication app(argc, argv);
    app.setWindowIcon(QIcon(":/images/appicon.png"));

    BaseStation b;
    b.show();


    return app.exec();
}
