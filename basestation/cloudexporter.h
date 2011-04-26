#ifndef CLOUDEXPORTER_H
#define CLOUDEXPORTER_H

#include <QtCore>
#include <QtGui>
#include <QtNetwork>
#include <QDebug>

#include <QUdpSocket>
#include <QMutex>

#include <octree.h>
#include <lidarpoint.h>

class CloudExporter : public QObject
{
    Q_OBJECT

private:

public:
    CloudExporter();
    ~CloudExporter();

    static bool savePly(QWidget* widget, const Octree* tree, const QString &fileName);
    static bool savePly(const Node* node, QTextStream* stream, QProgressDialog* progress);

};

#endif