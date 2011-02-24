#include "rtkbase.h"

RtkBase::RtkBase(int argc, char **argv) : QCoreApplication(argc, argv)
{
	uint portNumberNetwork = 8888;
	QString portSerial = "/dev/ttyACM0";

	QStringList commandLine = arguments();


	if(commandLine.lastIndexOf("-p") != -1 && commandLine.size() > commandLine.lastIndexOf("-p") + 1)
	{
	    bool ok;
	    uint networkPortNew = commandLine.at(commandLine.lastIndexOf("-p")+1).toUInt(&ok);
	    if(ok) portNumberNetwork = networkPortNew;
	}

	if(commandLine.lastIndexOf("-s") != -1 && commandLine.size() > commandLine.lastIndexOf("-s") + 1)
	{
	    portSerial = commandLine.at(commandLine.lastIndexOf("-s") + 1);
	}

	qDebug() << "using serial port" << portSerial;
	qDebug() << "using network port" << portNumberNetwork;

	mServer = new Server(this, portNumberNetwork);
	mGpsDevice = new GpsDevice(portSerial, this);

	connect(mGpsDevice, SIGNAL(correctionDataReady(QByteArray)), mServer, SLOT(slotSendCorrectionData(QByteArray)));
}

RtkBase::~RtkBase()
{
    delete mServer;
    delete mGpsDevice;
}

int main(int argc, char **argv)
{
	RtkBase rtkBase(argc, argv);
	return rtkBase.exec();
}
