#ifndef CLOUDEXPORTER_H
#define CLOUDEXPORTER_H

#include <QtCore>
#include <QtGui>
#include <QtNetwork>
#include <QDebug>

#include <QUdpSocket>
#include <QMutex>

#include "octree.h"

class CloudExporter : public QObject
{
    Q_OBJECT

private:
    Octree* mOctree;

private slots:
    void slotReadSocketPoints();
    void slotReadSocketImages();
    void slotSocketPointsError(QAbstractSocket::SocketError socketError);
    void slotSocketImagesError(QAbstractSocket::SocketError socketError);

public:
    CloudExporter(const Octree* octree);
    ~CloudExporter();

public slots:
    savePly(const QString &fileName);


};

#endif
