#include "simulator.h"

#include <cstdlib>

Simulator::Simulator(void) : QMainWindow()
{
    CoordinateGps cOrigin(53.6055, 9.91, 0.0);
    mCoordinateConverter.setOrigin(cOrigin);
    CoordinateGps cg(53.6058, 9.81, 1.0);
    CoordinateOgre co = mCoordinateConverter.convert(cg);
    qDebug() << "CoordinateOgre:" << co << "Origin: " << cOrigin.toString() << "Other:" << cg.toString();

    mOgreWidget = new OgreWidget(this);
    mOgreWidget->setFocus();
    setCentralWidget(mOgreWidget);
    mOgreWidget->show();


    mBattery = new Battery(this, 12.0, 4.0);
    mBattery->setDischargeCurrent(20.0);

    mStatusWidget = new StatusWidget(this, mBattery);
    addDockWidget(Qt::RightDockWidgetArea, mStatusWidget);

    connect(mOgreWidget, SIGNAL(currentStats(QSize, int, float)), mStatusWidget, SLOT(slotUpdateVisualization(QSize, int, float)));



    QTimer* peter = new QTimer(this);
    peter->setInterval(1000);
    connect(peter, SIGNAL(timeout()), SLOT(slotFoo()));
    peter->start();

    srand(time(0));
}



void Simulator::slotFoo()
{
//    qDebug() << "Simulator::slotFoo()";
//    mOgreWidget->setBackgroundColor(QColor(rand() % 255, rand() % 255, rand() % 255, 0));
}
