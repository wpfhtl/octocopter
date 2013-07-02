#ifndef NETWORKSERVER_H
#define NETWORKSERVER_H

#include <QtCore>
#include <QtNetwork>
#include <QDebug>

#include "port.h"

class NetworkServer : public Port
{
    Q_OBJECT

private:
    QTcpServer* mTcpServer;
    QList<QTcpSocket*> mTcpSockets;
    QHostAddress mAddress;
    quint16 mPort;

private slots:
    void slotNewConnection(void);
    void slotConnectionEnded(void);
    void slotReadSocket(bool lockMutex = true);
//    void slotSocketError(QAbstractSocket::SocketError socketError);

public:
    NetworkServer(const QString& networkPort);
    ~NetworkServer();

public slots:
    void write(const QByteArray&);
};

#endif
