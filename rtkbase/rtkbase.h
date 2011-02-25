#ifndef RTKBASE_H
#define RTKBASE_H

#include <sys/socket.h>
#include <signal.h>

#include <QCoreApplication>
#include <QList>
#include <QTimer>
#include <QStringList>


#include "server.h"
#include "gpsdevice.h"

/// @class RtkBase
/// @brief
/// This class uses a TCP-Server and a serial port to send RTK correction
/// data to any connected clients.

class RtkBase : public QCoreApplication
{
    Q_OBJECT

    public:
	RtkBase(int argc, char **argv);
	~RtkBase(void);

	// Unix signal handler
	static void signalHandler(int unused);

    public slots:
	void slotHandleSignal();

    private:
	// We use this pipe for all signals.
	static int signalFd[2];
	Server *mServer;
	GpsDevice *mGpsDevice;


	QSocketNotifier *snSignalPipe;
};

#endif
