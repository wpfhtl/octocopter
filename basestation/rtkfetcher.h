#ifndef RTKFETCHER_H
#define RTKFETCHER_H

#include <QtCore>
#include <QtNetwork>

class RtkFetcher : public QObject
{
    Q_OBJECT

private:
    QTcpSocket* mTcpSocket;
    QString mRemoteHost;
    int mRemotePort;

    // When we get a packet indicating that the connection is alive, we re-start this timer,
    // which will switch to failure after no packet arrived for some seconds
    QTimer mTimerConnectionWatchdog;

    int mNumberOfRemainingReplies;
    QByteArray mLastCommandToDevice;
    QString mSerialPortOnDevice;
    QByteArray mReceiveBuffer;
    QList<QByteArray> mCommandQueue;

    void determineSerialPortOnDevice();
    void rtkOutputInitialize();
    void rtkOutputStart();
    void rtkOutputStop();

    void sendAsciiCommand(QString command);


private slots:
    void slotEmitConnectionTimedOut(void);
    void slotConnectToRtkBase();
    void slotSocketDataReady();
    void slotSocketStateChanged(QAbstractSocket::SocketState);
    void slotSocketError(QAbstractSocket::SocketError);
    void slotFlushCommandQueue();

public:
    RtkFetcher(const QString &hostName, const uint &port, QObject* parent = 0);

    ~RtkFetcher();

    bool isReceiving(void) const;

signals:
    void rtkData(const QByteArray& data);
    void connectionStatus(const bool& connected);
};

#endif
