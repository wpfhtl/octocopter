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
	mServer->close();
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

/*
	QDataStream in(mTcpSocket);
	in.setVersion(QDataStream::Qt_4_0);

	QString numberOfBytesAvailable = QString::number(mTcpSocket->bytesAvailable());

	// Set the blocksize of the incoming packet
	if(packetLength == 0)
	{
		//printf("bytes of incoming packet available: %d.\n", (int)tcpSocket->bytesAvailable());

		if (mTcpSocket->bytesAvailable() < (int)sizeof(quint16))
			return;

		in >> packetLength;
		//printf("wrote packetLength: %d.\n", packetLength);
	}

	if(mTcpSocket->bytesAvailable() < packetLength)
	{
		printf("packetLength is %d, but only %s bytes available. returning.\n", packetLength, qPrintable(numberOfBytesAvailable));
		return;
	}

	// reset packetLength for the next incoming packet
	packetLength = 0;

//	QList<QPointF> startAndEnd;
//	in >> startAndEnd;

//	handleRequest(startAndEnd);

	mTcpSocket->close();

	startTcpServer();
	*/
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

void Server::close()
{
}
