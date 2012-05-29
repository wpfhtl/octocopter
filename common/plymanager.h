#ifndef PLYMANAGER_H
#define PLYMANAGER_H

#include <QFile>
#include <QVector>
#include <QVector3D>
#include <QString>
#include <QTextStream>
#include <QDateTime>
#include <QDebug>
#include <QMessageBox>

//#ifdef BASESTATION
#include <QWidget>
#include <QProgressDialog>
#include <octree.h>
#include <flightplannerinterface.h>
#include <lidarpoint.h>
class FlightPlannerInterface;
//#endif

class PlyManager : public QObject
{
    Q_OBJECT

    enum IncludesNormals { NormalsIncluded, NormalsNotIncluded };
    enum IncludesDirection { DirectionIncluded, DirectionNotIncluded };

    QFile* mFileRead;
    QFile* mFileWrite;

    quint32 mPointsWritten;

private:
#ifdef BASESTATION
    // recursive reading from nodes and subnodes of octree, exports into textstream that is created by public version below
    static bool savePly(const Node* node, QTextStream* stream, QProgressDialog* progress);
#endif

    static const QString createHeader(const quint32& vertexCount, const PlyManager::IncludesNormals& includesNormals = NormalsNotIncluded, const PlyManager::IncludesDirection& includesDirection = DirectionNotIncluded);

public:
    enum DataDirection {DataRead, DataWrite};

    PlyManager(const QString& fileName, const PlyManager::DataDirection&);
    ~PlyManager();

    // saves cloud from @points into @fileName
    static bool savePly(const QVector<QVector3D>& points, const QString &fileName);
    static bool savePly(const QList<QVector3D>& points, const QString &fileName);

#ifdef BASESTATION
    static bool savePly(const QVector<LidarPoint>& points, const QString &fileName);

    // saves cloud from @tree into @fileName, reporting progress using a dialog
    static bool savePly(QWidget* widget, const Octree* tree, const QString &fileName);

    // loads cloud from @fileName into all the given @trees and @flightplanners
    static bool loadPly(QWidget* widget, const QList<Octree*>& trees, const QList<FlightPlannerInterface*>& flightPlanners, const QString &fileName);
#endif

public slots:
    void slotNewPoints(const QVector<QVector3D>& points, const QVector3D& scanPosition);
    void slotNewPoints(const QList<QVector3D>& points, const QVector3D& scanPosition);
};

#endif
