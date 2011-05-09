#include "basestation.h"
#include <bullet/btBulletDynamicsCommon.h>

#include <QApplication>
#include <QIcon>

int main(int argc, char *argv[])
{
    // Having OGRE or bullet use double precision means nothing but trouble.
    qDebug() << "main(): sizeof: btScalar" << sizeof(btScalar);
    Q_ASSERT(sizeof(btScalar) == 4 && "Please make sure Bullet is compiled to use single precision");

    QApplication app(argc, argv);
    app.setWindowIcon(QIcon(":/images/appicon.png"));

    BaseStation b;
    b.show();

    return app.exec();
}
