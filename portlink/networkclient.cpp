#include "networkclient.h"

NetworkClient::NetworkClient(const QString& networkPort) : Port()
{
    mSocketStateMap.insert(QAbstractSocket::UnconnectedState, "The socket is not connected.");
    mSocketStateMap.insert(QAbstractSocket::HostLookupState, "The socket is performing a host name lookup.");
    mSocketStateMap.insert(QAbstractSocket::ConnectingState, "The socket has started establishing a connection.");
    mSocketStateMap.insert(QAbstractSocket::ConnectedState, "A connection is established.");
    mSocketStateMap.insert(QAbstractSocket::BoundState, "The socket is bound to an address and port (for servers).");
    mSocketStateMap.insert(QAbstractSocket::ClosingState, "The socket is about to close (data may still be waiting to be written).");
    mSocketStateMap.insert(QAbstractSocket::ListeningState, "For internal use only.");

    QStringList addressStrings = networkPort.split(":");
    if(addressStrings.size() != 2) qFatal("NetworkClient::NetworkClient(): cannot parse hostname/address and port from %s, please use \"host/ip:port\" syntax", qPrintable(networkPort));
    mAddress = addressStrings.takeFirst();
    mPort = addressStrings.takeFirst().toInt();

    qDebug() << "NetworkClient::NetworkClient: connecting to address" << mAddress << "and port" << mPort;

    mTcpSocket = new QTcpSocket(this);
    connect(mTcpSocket, SIGNAL(readyRead()), SLOT(slotReadSocket()));
    connect(mTcpSocket, SIGNAL(stateChanged(QAbstractSocket::SocketState)), SLOT(slotStateChanged(QAbstractSocket::SocketState)));
    mTcpSocket->connectToHost(mAddress, mPort);
}

NetworkClient::~NetworkClient()
{
    mTcpSocket->close();
    mTcpSocket->deleteLater();
}

void NetworkClient::slotStateChanged(QAbstractSocket::SocketState state)
{
    qDebug() << "NetworkClient::slotStateChanged():" << mAddress << mPort << "new state:" << mSocketStateMap.value(mTcpSocket->state());

    switch(state)
    {
        case QAbstractSocket::UnconnectedState:
            qDebug() << "NetworkClient::slotStateChanged: connection closed, reconnecting to address" << mAddress << "and port" << mPort;
            usleep(250000);
            mTcpSocket->connectToHost(mAddress, mPort);
            break;
    }
}

void NetworkClient::slotReadSocket(bool lockMutex)
{
    emit data(mTcpSocket->readAll());
}

void NetworkClient::write(const QByteArray &data)
{
    if(mTcpSocket->state() != QAbstractSocket::ConnectedState)
    {
        qDebug() << "NetworkClient::slotSendData(): not currently connected to" << mAddress << mPort << ", throwing data away...";
        return;
    }

    mTcpSocket->write(data);
}
