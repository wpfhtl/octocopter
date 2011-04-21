#include "basestation.h"

#include <QApplication>
#include <QIcon>

int main(int argc, char *argv[])
{
//    ScanReceiver receiver(argc, argv);
//    return receiver.exec();

    QApplication app(argc, argv);
    app.setWindowIcon(QIcon(":/images/appicon.png"));

    BaseStation b;
    b.show();

    return app.exec();
}
