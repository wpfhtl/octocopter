#ifndef NETWORKCLIENT_H
#define NETWORKCLIENT_H

#include <QtCore>
#include <QtNetwork>
#include <QDebug>

#include "port.h"

class NetworkClient : public Port
{
    Q_OBJECT

private:
    QTcpSocket* mTcpSocket;
    QMap<QAbstractSocket::SocketState, QString> mSocketStateMap;
    QString mAddress;
    quint16 mPort;

private slots:
    void slotReadSocket(bool lockMutex = true);
    void slotStateChanged(QAbstractSocket::SocketState state);

public:
    NetworkClient(const QString& networkPort);
    ~NetworkClient();

signals:
    void data(const QByteArray&);

public slots:
    void write(const QByteArray&);
};

#endif
