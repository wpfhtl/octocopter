#ifndef SERVER_H
#define SERVER_H

#include <QCoreApplication>
#include <QTcpSocket>
#include <QTcpServer>
#include <QList>

//#include "configuration.h"
//#include "logger.h"
//#include "packet.h"

//#include <stdlib.h>	// needed for rand() and friends
//#include <sys/time.h>	// needed for gettimeofday()

/// @class Server
/// @brief
/// This class is a very simple server. It receives UDP datagrams, reads their
/// commands, calls methods in the CanDevice-members and *can* send data back
/// to the sender

class Server : public QObject
{
	Q_OBJECT

	private:
		QList<QTcpSocket*> mTcpSockets;
		QTcpServer* mServer;

	private slots:
		void slotIncomingPacket();
		void slotSocketDisconnected();
		void slotNewConnection();
	public slots:
		void slotSendCorrectionData(QByteArray);

	public:
		Server(QObject *parent, uint port);
		~Server(void);

		void close();
};

#endif
