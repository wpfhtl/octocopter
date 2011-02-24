#ifndef RTKBASE_H
#define RTKBASE_H

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

    private:
	Server *mServer;
	GpsDevice *mGpsDevice;
};

#endif
