#include "rtkfetcher.h"

RtkFetcher::RtkFetcher(const QString &hostName, const uint &port, QObject* parent) : QObject(parent)
{
      qDebug() << "RtkFetcher::RtkFetcher()";

      mNumberOfRemainingReplies = 0;
      mSerialPortOnDevice = "";

      mRemoteHost = hostName;
      mRemotePort = port;

      mTcpSocket = new QTcpSocket(this);

      connect(mTcpSocket, SIGNAL(readyRead()), SLOT(slotSocketDataReady()));
      connect(mTcpSocket, SIGNAL(stateChanged(QAbstractSocket::SocketState)), SLOT(slotSocketStateChanged(QAbstractSocket::SocketState)));
      connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)), SLOT(slotSocketError(QAbstractSocket::SocketError)));

      slotConnectToRtkBase();
}

RtkFetcher::~RtkFetcher()
{
      qDebug() << "RtkFetcher::~RtkFetcher()";
}

void RtkFetcher::slotConnectToRtkBase()
{
  if(mRemoteHost.isEmpty() || mRemotePort == -1)
  {
      qDebug() << "RtkFetcher::slotConnectToRtkBase(): not connecting, remote host undefined:" << mRemoteHost << mRemotePort;
  }
  else
  {
      qDebug() << "RtkFetcher::slotConnectToRtkBase(): connecting to" << mRemoteHost << mRemotePort;
      mTcpSocket->connectToHost(mRemoteHost, mRemotePort, QIODevice::ReadWrite);
  }
}

void RtkFetcher::slotSocketDataReady()
{
    usleep(100000); // wait for the later bytes of this message to come on in...
    mReceiveBuffer.append(mTcpSocket->readAll());

//    qDebug() << "RtkFetcher::slotSocketDataReady():" << mReceiveBuffer;

    if(mNumberOfRemainingReplies != 0)
    {
        const int position = mReceiveBuffer.indexOf(mSerialPortOnDevice + QString(">"));
        if(position != -1)
        {
            qDebug() << "RtkFetcher::slotSocketDataReady(): received reply to" << mLastCommandToDevice.trimmed() << ":" << mReceiveBuffer.left(position).trimmed();
            qDebug() << "RtkFetcher::slotSocketDataReady(): now sending next command, if any";

            if(mReceiveBuffer.left(position).contains("$R? ASCII commands between prompts were discarded!")) qDebug() << "RtkFetcher::slotSerialPortDataReady(): it seems we were talking too fast?!";

            mReceiveBuffer.remove(0, position+5);
            mNumberOfRemainingReplies--;
            slotFlushCommandQueue();
        }
    }
    else
    {
        // We're not waiting for a reply to a command, this must be correction data!
//        qDebug() << "RtkFetcher::slotSocketDataReady(): emitting" << mReceiveBuffer.size() << "bytes of correction data.";
        emit rtkData(mReceiveBuffer);
        mReceiveBuffer.clear();
    }
}

void RtkFetcher::slotSocketStateChanged(QAbstractSocket::SocketState socketState)
{
      qDebug() << "RtkFetcher::slotSocketStateChanged(): ignoring socket statechange to" << socketState;

      if(socketState == QAbstractSocket::ConnectedState && mSerialPortOnDevice.isEmpty())
      {
//        determineSerialPortOnDevice();

        //rtkOutputInitialize();

        //rtkOutputStart();
      }
}

void RtkFetcher::slotSocketError(QAbstractSocket::SocketError socketError)
{
      qWarning() << "RtkFetcher::slotSocketError(): error in connection to" << mRemoteHost << ":" << mTcpSocket->errorString();
      qWarning() << "RtkFetcher::slotSocketError(): will try to re-connect in 5s.";

      mTcpSocket->abort();
      QTimer::singleShot(5000, this, SLOT(slotConnectToRtkBase()));
}

bool RtkFetcher::isReceiving(void) const
{
      return mTcpSocket->state() == QAbstractSocket::ConnectedState;
}


void RtkFetcher::sendAsciiCommand(QString command)
{
    mCommandQueue.append(command.append("\r\n").toAscii());

    slotFlushCommandQueue();
}

void RtkFetcher::slotFlushCommandQueue()
{
    if(mNumberOfRemainingReplies == 0 && mCommandQueue.size())
    {
        mLastCommandToDevice = mCommandQueue.takeFirst();
        qDebug() << "RtkFetcher::slotFlushCommandQueue(): currently not waiting for a reply, so sending next command:" << mLastCommandToDevice.trimmed();
        if(mReceiveBuffer.size() != 0) qDebug() << "RtkFetcher::slotFlushCommandQueue(): WARNING! Receive Buffer still contains:" << mReceiveBuffer;

        mTcpSocket->write(mLastCommandToDevice);
        mNumberOfRemainingReplies++;
    }
    else if(mNumberOfRemainingReplies)
    {
        qDebug() << "RtkFetcher::slotFlushCommandQueue(): still waiting for" << mNumberOfRemainingReplies << "command-replies, not sending.";
    }
    else
    {
        qDebug() << "RtkFetcher::slotFlushCommandQueue(): nothing to send.";
    }
}

void RtkFetcher::determineSerialPortOnDevice()
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
        qDebug() << "RtkFetcher::determineSerialPortOnDevice(): serial port on device is now " << mSerialPortOnDevice;
        qDebug() << "RtkFetcher::determineSerialPortOnDevice(): other non-rtk data:" << data;
    }
    else
    {
        qFatal("RtkFetcher::determineSerialPortOnDevice(): couldn't get serialPortOnDevice, data is: %s", qPrintable(QString(data)));
    }
}

void RtkFetcher::rtkOutputInitialize()
{
    Q_ASSERT(mSerialPortOnDevice.size() == 4);

    // Have a look at the Septentrio Firmware User Manual.pdf. These commands
    // are necessary to set the device into RTK-Base-Mode

    qDebug() << "RtkFetcher::rtkOutputInitialize(): sending rtk output init commands from rtk-init.txt";

    QFile file("rtk-init.txt");
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "RtkFetcher::rtkOutputInitialize(): couldn't read rtk-init.txt to initialize gps device";
        QCoreApplication::quit();
    }

    while(!file.atEnd())
    {
        QByteArray line = file.readLine();
        line.replace("$PORT", mSerialPortOnDevice.toAscii());
        sendAsciiCommand(line.trimmed());
    }

    qDebug() << "RtkFetcher::rtkOutputInitialize(): done sending rtk-init.txt";
}

void RtkFetcher::rtkOutputStart()
{
    qDebug() << "RtkFetcher::rtkOutputStart(): sending rtk output init commands from rtk-start.txt";

    QFile file("rtk-start.txt");
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "RtkFetcher::rtkOutputStart(): couldn't read rtk-start.txt to start rtk output";
        QCoreApplication::quit();
    }

    while(!file.atEnd())
    {
        QByteArray line = file.readLine();
        line.replace("$PORT", mSerialPortOnDevice.toAscii());
        sendAsciiCommand(line.trimmed());
    }

    qDebug() << "RtkFetcher::rtkOutputStart(): done sending rtk-start.txt";
}

void RtkFetcher::rtkOutputStop()
{
    usleep(100000);
//    mSerialPort->write("SSSSSSSSSS");
    QCoreApplication::processEvents();
//    sleep(11);

    qDebug() << "RtkFetcher::rtkOutputStop(): sending rtk output init commands from rtk-stop.txt";

    QFile file("rtk-stop.txt");
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "RtkFetcher::rtkOutputStop(): couldn't read rtk-stop.txt to stop rtk output";
        QCoreApplication::quit();
    }

    while(!file.atEnd())
    {
        QByteArray line = file.readLine();
        line.replace("$PORT", mSerialPortOnDevice.toAscii());
        sendAsciiCommand(line.trimmed());
    }

    qDebug() << "RtkFetcher::rtkOutputStop(): done sending rtk-stop.txt";
}
