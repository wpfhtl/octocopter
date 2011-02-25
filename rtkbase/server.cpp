#include "server.h"

Server::Server(QObject *parent, uint port) :
    QObject(parent)
{
	mServer = new QTcpServer(this);
	connect(mServer, SIGNAL(newConnection()), SLOT(slotNewConnection()));

	if(mServer->listen(QHostAddress::Any, port))
	{
		qDebug() << "Server::Server(): successfully opened TCP server on port" << port;
	}
	else
	{
		qDebug() << "Server::Server(): failed to open TCP server on port" << port;
		exit(1);
	}
}

Server::~Server()
{
    qDebug() << "Server::~Server()";
    mServer->close();

    foreach(QTcpSocket* socket, mTcpSockets)
    {
	socket->close();
	socket->deleteLater();
    }

    mServer->deleteLater();
}

void Server::slotNewConnection()
{
    QTcpSocket* newSocket = mServer->nextPendingConnection();
    qDebug() << "Server::Server(): new network connection from" << newSocket->peerAddress();
    mTcpSockets.append(newSocket);

    connect(newSocket, SIGNAL(readyRead()), SLOT(slotIncomingPacket()));
    connect(newSocket, SIGNAL(disconnected()), SLOT(slotSocketDisconnected()));
}

void Server::slotSocketDisconnected()
{
    foreach(QTcpSocket* socket, mTcpSockets)
    {
	if(socket->state() != QTcpSocket::ConnectedState)
	{
	    qDebug() << "Server::slotSocketDisconnected(): client disconnected:" << socket->peerAddress();
	    mTcpSockets.removeOne(socket);
	    socket->deleteLater();
	}
    }
}

void Server::slotIncomingPacket()
{
	foreach(QTcpSocket* socket, mTcpSockets)
	{
	    if(socket->bytesAvailable()) qDebug() << "Server::slotIncomingPacket(): ignoring client" << socket->peerAddress()<< "message:" << socket->readAll();
	}
}

void Server::slotSendCorrectionData(QByteArray data)
{
    qDebug() << "Server::slotSendCorrectionData(): sending data to clients:" << data;

    foreach(QTcpSocket* socket, mTcpSockets)
    {
//	QDataStream out(socket);
//	out.setVersion(QDataStream::Qt_4_7);
//	out << data;
	qDebug() << "Server::slotSendCorrectionData(): sending data to" << socket->peerAddress();
	socket->write(data);
    }

}
