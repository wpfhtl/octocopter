#ifndef PLYMANAGER_H
#define PLYMANAGER_H

#include <QtCore>
#include <QtGui>
#include <QtNetwork>
#include <QDebug>

#include <QUdpSocket>
#include <QMutex>

#include <octree.h>
#include <lidarpoint.h>
#include <flightplannerinterface.h>

class FlightPlannerInterface;

class PlyManager : public QObject
{
    Q_OBJECT

private:
    // recursive reading from nodes and subnodes of octree, exports into textstream that is created by public version below
    static bool savePly(const Node* node, QTextStream* stream, QProgressDialog* progress);

public:
    PlyManager();
    ~PlyManager();

    // saves cloud from @tree into @fileName, reporting progress using a dialog
    static bool savePly(QWidget* widget, const Octree* tree, const QString &fileName);

    // loads cloud from @fileName into all the given @trees and @flightplanners
    static bool loadPly(QWidget* widget, const QList<Octree*>& trees, const QList<FlightPlannerInterface*>& flightPlanners, const QString &fileName);
};

#endif
