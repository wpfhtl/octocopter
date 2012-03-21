#include "networkserver.h"

NetworkServer::NetworkServer(const QString& networkPort) : Port()
{
    mAddress.setAddress(networkPort.section(":", 0, 0));
    mPort = networkPort.section(":", 1, 1).toInt();

    qDebug() << "NetworkServer::NetworkServer: listening on" << mAddress.toString() << ":" << mPort;

    mTcpServer = new QTcpServer(this);
    connect(mTcpServer, SIGNAL(newConnection()), SLOT(slotNewConnection()));
    mTcpServer->listen(mAddress, mPort);
}

NetworkServer::~NetworkServer()
{
    qDebug() << "NetworkServer::~NetworkServer: closing server on address" << mAddress.toString() << ":" << mPort;
    mTcpServer->close();
    delete mTcpServer;
}

void NetworkServer::slotConnectionEnded()
{
    for(int i=0; i < mTcpSockets.size(); i++)
    {
        QTcpSocket* socket = mTcpSockets.at(i);
        if(socket->state() == QAbstractSocket::ClosingState || socket->state() == QAbstractSocket::UnconnectedState)
        {
            qDebug() << "NetworkServer::slotConnectionEnded(): connection from" << socket->peerAddress() << socket->peerPort() << "closed, removing socket.";
            socket->disconnect();
            socket->deleteLater();
            mTcpSockets.removeAt(i);
        }
    }
}

void NetworkServer::slotNewConnection()
{
    QTcpSocket* socket = mTcpServer->nextPendingConnection();
    qDebug() << "NetworkServer::slotNewConnection(): accepting connection from" << socket->peerName() << "and port" << socket->peerPort();
    connect(socket, SIGNAL(disconnected()), SLOT(slotConnectionEnded()));
    connect(socket, SIGNAL(error(QAbstractSocket::SocketError)), SLOT(slotConnectionEnded()));
    connect(socket, SIGNAL(readyRead()), SLOT(slotReadSocket()));

    mTcpSockets.append(socket);
}

void NetworkServer::slotReadSocket(bool lockMutex)
{
    QTcpSocket *sendingSocket = (QTcpSocket*)sender();
    const QByteArray payload = sendingSocket->readAll();

    // send data to all other connected clients
    for(int i=0; i < mTcpSockets.size(); i++)
    {
        QTcpSocket *currentSocket = mTcpSockets.at(i);
        if(sendingSocket != currentSocket) currentSocket->write(payload);
    }

    emit data(payload);
}

void NetworkServer::write(const QByteArray &data)
{
    if(mTcpSockets.size() == 0)
    {
        //qDebug() << "NetworkServer::write(): no client connected, throwing data away...";
        return;
    }

    for(int i=0; i < mTcpSockets.size(); i++)
    {
        mTcpSockets.at(i)->write(data);
    }
}
