#include "basestation.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    app.setWindowIcon(QIcon(":/images/appicon.png"));

    BaseStation b;
    b.show();

    return app.exec();
}
