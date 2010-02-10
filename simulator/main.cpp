#include "simulator.h"

#include <QApplication>
#include <QIcon>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    app.setWindowIcon(QIcon(":/images/appicon.png"));

    Simulator simulator;
    simulator.show();

    return app.exec();
}
