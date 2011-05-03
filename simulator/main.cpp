#include "simulator.h"



#include <QApplication>
#include <QIcon>

// to catch floating point exceptions
#ifndef _GNU_SOURCE
  #define _GNU_SOURCE
#endif
#include <fenv.h>


int main(int argc, char *argv[])
{
//    feenableexcept(FE_INVALID  | FE_DIVBYZERO | FE_OVERFLOW | FE_UNDERFLOW);

    // Having OGRE or bullet use double precision means nothing but trouble.
    qDebug() << "main(): sizeof: Ogre::Real" << sizeof(Ogre::Real) << "btScalar" << sizeof(btScalar);
    Q_ASSERT(sizeof(btScalar) == 4 && "Please make sure Bullet is compiled to use single precision");
    Q_ASSERT(sizeof(Ogre::Real) == 4 && "Please make sure OGRE is compiled to use single precision");


    QApplication app(argc, argv);
    app.setWindowIcon(QIcon(":/images/appicon.png"));

    Simulator simulator;
    simulator.show();

    return app.exec();
}
