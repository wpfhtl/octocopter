#include "simulator.h"

#include <QApplication>
#include <QIcon>

// to catch floating point exceptions
#define _GNU_SOURCE
#include <fenv.h>


int main(int argc, char *argv[])
{
    feenableexcept(FE_INVALID  | FE_DIVBYZERO | FE_OVERFLOW | FE_UNDERFLOW);

    QApplication app(argc, argv);
    app.setWindowIcon(QIcon(":/images/appicon.png"));

    Simulator simulator;
    simulator.show();

    return app.exec();
}
