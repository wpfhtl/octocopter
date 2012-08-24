#ifndef DIFFCORRFETCHER_H
#define DIFFCORRFETCHER_H

#include <QtCore>
#include <QtNetwork>

class DiffCorrFetcher : public QObject
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
    DiffCorrFetcher(const QString &hostName, const uint &port, QObject* parent = 0);

    ~DiffCorrFetcher();

    bool isReceiving(void) const;

signals:
    void differentialCorrections(const QByteArray& data);
    void connectionStatus(const bool& connected);
};

#endif
