#include "basestation.h"

#include <tsp/graph.h>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    app.setWindowIcon(QIcon(":/images/appicon.png"));

    BaseStation b;
    b.show();

    return app.exec();
}
