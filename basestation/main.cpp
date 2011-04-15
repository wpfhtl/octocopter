#include "triangulator.h"

#include <QApplication>
#include <QIcon>

int main(int argc, char *argv[])
{
//    ScanReceiver receiver(argc, argv);
//    return receiver.exec();

    QApplication app(argc, argv);
    app.setWindowIcon(QIcon(":/images/appicon.png"));

    Triangulator t;
    t.show();

    return app.exec();
}
