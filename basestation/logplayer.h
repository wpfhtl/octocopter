#ifndef LOGPLAYER_H
#define LOGPLAYER_H

#include <QDockWidget>
#include <QMessageBox>
#include <QInputDialog>
#include <QTimer>
#include <QFileDialog>
#include <QProgressBar>
#include <QMouseEvent>
#include <QString>
#include <QByteArray>

#include <sbfparser.h>
#include <gnssstatus.h>
#include <vehiclestatus.h>
#include <flightcontrollervalues.h>
#include <sensorfuser.h>

class ProgressBar : public QProgressBar
{
    Q_OBJECT

public:
    ProgressBar(QWidget* parent = 0) : QProgressBar(parent)
    {
        setFormat("TOW %v");
        setValue(0);
    }

protected:
    void mouseReleaseEvent ( QMouseEvent * event )
    {
        const float width = size().width();// - 7.0;
        float pos = event->pos().x() + 1.5;
        const qint32 tow = minimum() + qBound(0.0f, pos / width, 1.0f) * (maximum() - minimum());
//        qDebug() << "width" << width << "pos" << pos << "percent:" << (pos / width) * 100.0f << "tow" << tow;
        emit seekToTow(tow);
    }

signals:
    void seekToTow(qint32);
};


namespace Ui {
    class LogPlayer;
}

class LogPlayer : public QDockWidget
{
    Q_OBJECT

public:
    LogPlayer(QWidget *parent = 0);
    ~LogPlayer();

    const FlightControllerValues* const getFlightControllerValues() const {return &mFlightControllerValues;}
    //void keyPressEvent(QKeyEvent* event);

private:
    // probably cheaper than creating lots of QByteArrays using QByteArray::mid().
    struct Data
    {
        const char* data;
        qint32 size;

        Data() : data(0), size(0) {}
        Data(const char* const d, const qint32 s) : data(d), size(s) {}
    };

    // We can have multiple Laserscanners, so wrap their informations in a class. While we're at it,
    // also move GNSS and FC logs into this structure.
    struct LogData
    {
        QByteArray data;
        qint32 cursor;

        LogData() : cursor(-1) {}
        LogData(const QByteArray& d) : data(d), cursor(-1) {}
    };

    enum LogType
    {
        LogTypeGnss,
        LogTypeLaser,
        LogTypeFlightController,
        LogTypeInvalid
    };

    // Used to indicate a data/logfile source, device, etc. Necessary now that we can have multiple laserscanners
    struct DataSource
    {
        LogType type;
        quint8 index;

        DataSource() : type(LogTypeInvalid), index(0) {}
        DataSource(const LogType& t) : type(t), index(0) {}
        DataSource(const LogType& t, const quint8 i) : type(t), index(i) {}
    };

    // For the step-datasource-selection
    QSignalMapper* mStepSignalMapper;
    QMenu* mStepMenu;
    LogType mStepUntilLogType;

    Ui::LogPlayer *ui;
    SbfParser* mSbfParser;
    SensorFuser* mSensorFuser;
    LogData mLogGnss, mLogFlightController;
    QMap<QString, LogData> mLogsLaser;

//    qint32 mCursorLaser; // points to either 1) the beginning, 2) a byte after a \n or 3) behind the QByteArray's last byte (which should equal 2))
//    qint32 mCursorSbf; // points to either 1) the beginning, 2) two bytes $@ or 3) behind the QByteArray's last byte (which should equal 2))
//    qint32 mCursorFlightController; // points to either 1) the beginning, 2) a byte after a \n or 3) behind the QByteArray's last byte (which should equal 2))

    // Stuff needed for realtime-playback
    QTimer* mTimerAnimation;
    QTime mTimePlaybackStartReal;
    qint32 mTimePlaybackStartTow;

    ProgressBar* mProgressBarTow;

    // Retrieves the next valid packet from the private data, or an empty packet if that
    Data getNextPacket(const LogType &type);

    // Returns the TOW of the next Laser-Packet/Line, or -1 when there's no more packet available.
    qint32 getNextTow(const LogType &source);
    qint32 getLastTow(const LogType &type);

    DataSource getNextDataSource(qint32* tow = 0);

    void processPacket(const LogPlayer::DataSource& source, const Data &packetLaser);

    // This class keeps instances of objects that are updated from the logfiles. After they are,
    // we simply emit pointers to this data.
    GnssStatus mGnssStatus;
    FlightControllerValues mFlightControllerValues;
    Pose mPose; // is also within FlightControllerValues!?
    QVector<QVector3D> mRegisteredPoints;
    QVector3D mScannerPosition;
    VehicleStatus mVehicleStatus;

private slots:
//    void slotLaserScannerRelativePoseChanged();
    void slotNewSbfTime(const qint32 tow, const char *, quint16);

    bool slotOpenLogFiles();
    DataSource slotStepForward(DataSource source = DataSource());
    void slotRewind();
    void slotGoToTow(qint32 towTarget = -1);
    void slotPlay();

    void slotStepDataSourceChanged(const int logType);
    bool slotStepUntilDataSourceProcessed();

signals:
    void message(const LogImportance& importance, const QString& source, const QString& message);

    void vehiclePose(const Pose* const);
    void scanData(const float* const points, const quint32& count, const QVector3D* const scannerPosition);
    void vehicleStatus(const VehicleStatus* const);
    void gnssStatus(const GnssStatus* const);
    void flightControllerValues(const FlightControllerValues* const);
    void flightControllerWeightsChanged();

    // Emits the raw distances, like the ones being fed into sensorfuser. You are NOT owner of this data, sensorfuser
    // is and it will probably delete it. So, if you need it, copy it!
    void newScanData(qint32 timestampScanner, std::vector<quint16> * const distances);
};

#endif // LOGPLAYER_H
