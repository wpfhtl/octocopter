#include <unistd.h>

#include "diffcorrfetcher.h"

DiffCorrFetcher::DiffCorrFetcher(const QString &hostName, const uint &port, QObject* parent) : QObject(parent)
{
      qDebug() << "DiffCorrFetcher::DiffCorrFetcher()";

      mNumberOfRemainingReplies = 0;
      mSerialPortOnDevice = "";

      mRemoteHost = hostName;
      mRemotePort = port;

      mTimerConnectionWatchdog.setInterval(2000);
      mTimerConnectionWatchdog.start();
      connect(&mTimerConnectionWatchdog, SIGNAL(timeout()), SLOT(slotEmitConnectionTimedOut()));

      mTcpSocket = new QTcpSocket(this);

      connect(mTcpSocket, &QTcpSocket::readyRead, this, &DiffCorrFetcher::slotSocketDataReady);
      connect(mTcpSocket, &QTcpSocket::stateChanged, this, &DiffCorrFetcher::slotSocketStateChanged);
      //connect(mTcpSocket, &QTcpSocket::error, this, &DiffCorrFetcher::slotSocketError);
      connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)), SLOT(slotSocketError(QAbstractSocket::SocketError)));

      slotConnectToRtkBase();
}

DiffCorrFetcher::~DiffCorrFetcher()
{
      qDebug() << "DiffCorrFetcher::~DiffCorrFetcher()";
}


void DiffCorrFetcher::slotEmitConnectionTimedOut()
{
    // This method is called only by a timer, which is reset whenever a packet comes in.
    // So, as lsong as we receive packets within intervals smaller than the timer's, we
    // should never emit a broken connection.
    emit connectionStatus(false);
}


void DiffCorrFetcher::slotConnectToRtkBase()
{
  if(mRemoteHost.isEmpty() || mRemotePort == -1)
  {
      qDebug() << "DiffCorrFetcher::slotConnectToRtkBase(): not connecting, remote host undefined:" << mRemoteHost << mRemotePort;
  }
  else
  {
      qDebug() << "DiffCorrFetcher::slotConnectToRtkBase(): connecting to" << mRemoteHost << mRemotePort;
      mTcpSocket->connectToHost(mRemoteHost, mRemotePort, QIODevice::ReadWrite);
  }
}

void DiffCorrFetcher::slotSocketDataReady()
{
    //usleep(100000); // wait for the later bytes of this message to come on in...
    mReceiveBuffer.append(mTcpSocket->readAll());

//    qDebug() << "DiffCorrFetcher::slotSocketDataReady():" << mReceiveBuffer;

    if(mNumberOfRemainingReplies != 0)
    {
        const int position = mReceiveBuffer.indexOf(mSerialPortOnDevice + QString(">"));
        if(position != -1)
        {
            qDebug() << "DiffCorrFetcher::slotSocketDataReady(): received reply to" << mLastCommandToDevice.trimmed() << ":" << mReceiveBuffer.left(position).trimmed();
            qDebug() << "DiffCorrFetcher::slotSocketDataReady(): now sending next command, if any";

            if(mReceiveBuffer.left(position).contains("$R? ASCII commands between prompts were discarded!")) qDebug() << "DiffCorrFetcher::slotSerialPortDataReady(): it seems we were talking too fast?!";

            mReceiveBuffer.remove(0, position+5);
            mNumberOfRemainingReplies--;
            slotFlushCommandQueue();
        }
    }
    else
    {
        // We're not waiting for a reply to a command, this must be correction data!
//        qDebug() << "DiffCorrFetcher::slotSocketDataReady(): emitting" << mReceiveBuffer.size() << "bytes of correction data.";

        // This if causes only big packets to be sent, thus delaying correction data. Disable it for now.
        if(mReceiveBuffer.size() > 300)
        {
            emit differentialCorrections(mReceiveBuffer);
            mReceiveBuffer.clear();

            // Restart the timer, so we don't get a timeout.
            mTimerConnectionWatchdog.start();
            emit connectionStatus(true);
        }
    }
}

void DiffCorrFetcher::slotSocketStateChanged(QAbstractSocket::SocketState socketState)
{
      qDebug() << "DiffCorrFetcher::slotSocketStateChanged(): ignoring socket statechange to" << socketState;

      if(socketState == QAbstractSocket::ConnectedState && mSerialPortOnDevice.isEmpty())
      {
//        determineSerialPortOnDevice();

        //rtkOutputInitialize();

        //rtkOutputStart();
      }
}

void DiffCorrFetcher::slotSocketError(QAbstractSocket::SocketError socketError)
{
      qWarning() << "DiffCorrFetcher::slotSocketError(): error in connection to" << mRemoteHost << ":" << mTcpSocket->errorString();
      qWarning() << "DiffCorrFetcher::slotSocketError(): will try to re-connect in 5s.";

      mTcpSocket->abort();
      QTimer::singleShot(2000, this, SLOT(slotConnectToRtkBase()));
}

bool DiffCorrFetcher::isReceiving(void) const
{
      return mTcpSocket->state() == QAbstractSocket::ConnectedState;
}


void DiffCorrFetcher::sendAsciiCommand(QString command)
{
    mCommandQueue.append(command.append("\r\n").toLatin1());

    slotFlushCommandQueue();
}

void DiffCorrFetcher::slotFlushCommandQueue()
{
    if(mNumberOfRemainingReplies == 0 && mCommandQueue.size())
    {
        mLastCommandToDevice = mCommandQueue.takeFirst();
        qDebug() << "DiffCorrFetcher::slotFlushCommandQueue(): currently not waiting for a reply, so sending next command:" << mLastCommandToDevice.trimmed();
        if(mReceiveBuffer.size() != 0) qDebug() << "DiffCorrFetcher::slotFlushCommandQueue(): WARNING! Receive Buffer still contains:" << mReceiveBuffer;

        mTcpSocket->write(mLastCommandToDevice);
        mNumberOfRemainingReplies++;
    }
    else if(mNumberOfRemainingReplies)
    {
        qDebug() << "DiffCorrFetcher::slotFlushCommandQueue(): still waiting for" << mNumberOfRemainingReplies << "command-replies, not sending.";
    }
    else
    {
        qDebug() << "DiffCorrFetcher::slotFlushCommandQueue(): nothing to send.";
    }
}

void DiffCorrFetcher::determineSerialPortOnDevice()
{
    // We just send any string causing a reply to the device, so we can see on what port it is talking to us.
    mTcpSocket->write("setDataInOut,all,CMD,none\n");
    usleep(100000);
    mTcpSocket->write("getReceiverCapabilities\n");
    usleep(100000);

    // If we enable this, we must make sure that readyRead() isn't emitted and the data is read by the connected slot instead of being read by us.
    //QCoreApplication::processEvents();

    // After waiting for the reply, read and analyze.
    QByteArray data = mTcpSocket->readAll();
    QString port = data.right(5).left(4);

    // Use line sending with e.g. COM2 or USB1 to determine the port on the serial device being used.
    if(data.right(1) == ">" && (port.left(3) == "COM" || port.left(3) == "USB"))
    {
        mSerialPortOnDevice = port;
        qDebug() << "DiffCorrFetcher::determineSerialPortOnDevice(): serial port on device is now " << mSerialPortOnDevice;
        qDebug() << "DiffCorrFetcher::determineSerialPortOnDevice(): other non-rtk data:" << data;
    }
    else
    {
        qFatal("DiffCorrFetcher::determineSerialPortOnDevice(): couldn't get serialPortOnDevice, data is: %s", qPrintable(QString(data)));
    }
}

void DiffCorrFetcher::rtkOutputInitialize()
{
    Q_ASSERT(mSerialPortOnDevice.size() == 4);

    // Have a look at the Septentrio Firmware User Manual.pdf. These commands
    // are necessary to set the device into RTK-Base-Mode

    qDebug() << "DiffCorrFetcher::rtkOutputInitialize(): sending rtk output init commands from rtk-init.txt";

    QFile file("rtk-init.txt");
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "DiffCorrFetcher::rtkOutputInitialize(): couldn't read rtk-init.txt to initialize gps device";
        QCoreApplication::quit();
    }

    while(!file.atEnd())
    {
        QByteArray line = file.readLine();
        line.replace("$PORT", mSerialPortOnDevice.toLatin1());
        sendAsciiCommand(line.trimmed());
    }

    qDebug() << "DiffCorrFetcher::rtkOutputInitialize(): done sending rtk-init.txt";
}

void DiffCorrFetcher::rtkOutputStart()
{
    qDebug() << "DiffCorrFetcher::rtkOutputStart(): sending rtk output init commands from rtk-start.txt";

    QFile file("rtk-start.txt");
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "DiffCorrFetcher::rtkOutputStart(): couldn't read rtk-start.txt to start rtk output";
        QCoreApplication::quit();
    }

    while(!file.atEnd())
    {
        QByteArray line = file.readLine();
        line.replace("$PORT", mSerialPortOnDevice.toLatin1());
        sendAsciiCommand(line.trimmed());
    }

    qDebug() << "DiffCorrFetcher::rtkOutputStart(): done sending rtk-start.txt";
}

void DiffCorrFetcher::rtkOutputStop()
{
    usleep(100000);
//    mSerialPort->write("SSSSSSSSSS");
    QCoreApplication::processEvents();
//    sleep(11);

    qDebug() << "DiffCorrFetcher::rtkOutputStop(): sending rtk output init commands from rtk-stop.txt";

    QFile file("rtk-stop.txt");
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "DiffCorrFetcher::rtkOutputStop(): couldn't read rtk-stop.txt to stop rtk output";
        QCoreApplication::quit();
    }

    while(!file.atEnd())
    {
        QByteArray line = file.readLine();
        line.replace("$PORT", mSerialPortOnDevice.toLatin1());
        sendAsciiCommand(line.trimmed());
    }

    qDebug() << "DiffCorrFetcher::rtkOutputStop(): done sending rtk-stop.txt";
}
