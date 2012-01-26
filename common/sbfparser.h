#ifndef SBFPARSER_H
#define SBFPARSER_H

#include <QCoreApplication>
#include <QTimer>
#include <pose.h>

#include <gpsstatusinformation.h>

/**
  This class parses binary SBF (Septentrio Binary Format) data and emits
  the interesting values as signals.

  The processSbfData() method takes a QByteArray
  */


static const quint16 CRC_16CCIT_LookUp[256] = {

  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
  0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
  0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
  0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
  0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
  0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
  0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
  0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
  0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
  0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
  0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
  0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
  0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
  0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
  0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
  0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
  0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
  0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
  0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
  0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
  0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};


class SbfParser : public QObject
{
    Q_OBJECT
private:
    bool mFirmwareBug_20120111_RtkWasEnabledAfterAttitudeDeterminationSucceeded;

    Pose mLastPose;

    GpsStatusInformation::GpsStatus mGpsStatus;

    QDateTime mTimeStampStartup; // to determine runtime and clock skew at the end.

    quint8 mPoseClockDivisor; // for emitting a low-frequency pose for debugging

    double mOriginLongitude, mOriginLatitude, mOriginElevation;

    // To emit status in regular intervals
//    QTimer* mStatusTimer;

    struct Sbf_Header
    {
      quint16       Sync;
      quint16       CRC;
      quint16       ID;
      quint16       Length;
    };

    struct Sbf_ReceiverTime
    {
      Sbf_Header  Header;

      quint32       TOW;
      quint16       WNc;

      qint8         UTCYear;
      qint8         UTCMonth;
      qint8         UTCDay;
      qint8         UTCHour;
      qint8         UTCMin;
      qint8         UTCSec;
      qint8         DeltaLS;
      quint8        SyncLevel;
      quint8        Reserved[2];
    };

    struct Sbf_IntAttCovEuler
    {
      Sbf_Header  Header;

      quint32       TOW;
      quint16       WNc;

      quint8        Mode;
      quint8        Error;

      float          Cov_HeadHead;
      float          Cov_PitchPitch;
      float          Cov_RollRoll;
      float          Cov_HeadPitch;
      float          Cov_HeadRoll;
      float          Cov_PitchRoll;
    };

    struct Sbf_ExtEvent
    {
        Sbf_Header  Header;
        quint32       TOW;
        quint16       WNc;
        quint8        Source;
        quint8        Polarity;
        float          Offset;
        double         RxClkBias;
        quint16       PVTAge;
    };

    struct Sbf_PVAAGeod
    {
        Sbf_Header  Header;

        quint32       TOW;
        quint16       WNc;

        quint8        Mode;
        quint8        Error;

        quint16       Info;

        quint8        GNSSPVTMode;
        quint8        Datum;
        quint8        GNSSage;

        quint8        NrSVAnt;
        quint8        Reserved;

        quint8        PosFine;
        qint32        Lat;
        qint32        Lon;
        qint32        Alt;

        qint32        Vnorth;
        qint32        Veast;
        qint32        Vup;

        qint16        Ax;
        qint16        Ay;
        qint16        Az;

        quint16       Heading;
        qint16        Pitch;
        qint16        Roll;
    };

    struct Sbf_IntAttEuler
    {
        Sbf_Header Header;

        quint32       TOW;
        quint16       WNc;

        quint8        Mode;
        quint8        Error;

        quint16       Info;
        quint8        NrSV;
        quint8        NrAnt;

        quint8        Reserved;
        quint8        Datum;
        quint16       GNSSage;

        float          Heading;
        float          Pitch;
        float          Roll;

        float          PitchDot;
        float          RollDot;
        float          HeadingDot;
    };

    struct Sbf_PVTCartesian
    {
      Sbf_Header  Header;

      quint32       TOW;
      quint16       WNc;

      quint8        Mode;
      quint8        Error;
      double         X;
      double         Y;
      double         Z;
      float          Undulation;
      float          Vx;
      float          Vy;
      float          Vz;
      float          COG;
      double         RxClkBias;
      float          RxClkDrift;
      quint8        TimeSystem;
      quint8        Datum;
      quint8        NrSV;
      quint8        WACorrInfo;
      quint16       ReferenceId;
      quint16       MeanCorrAge;
      quint32       SignalInfo;
      quint8        AlertFlag;
      quint8        NrBases;
      quint8        Reserved[2];
    };

    struct Sbf_ReceiverStatus
    {
      Sbf_Header  Header;

      quint32       TOW;
      quint16       WNc;

      quint8        CPULoad;
      quint8        ExtError;
      quint32       UpTime;
      quint32       RxStatus;
      quint32       RxError;
      quint8        N;
      quint8        SBSize;
      quint8        Reserved2[2];
      // unused AGCState_2_0_t AGCState[MAXSB_AGCSTATE];
    };

    void setPose(const qint32& lon, const qint32& lat, const qint32& alt, const quint16& heading, const qint16& pitch, const qint16& roll, const quint32& tow);

    quint16 getCrc(const void *buf, unsigned int length);

    QVector3D convertGeodeticToCartesian(const double& lon, const double& lat, const double& elevation);

public:
    SbfParser(QObject *parent = 0);
    ~SbfParser();

    // Returns the TOW of the next SBF packet in @sbfData, or -1 of there is no packet at the beginning of @sbfData
    qint32 peekNextTow(const QByteArray& sbfData);

    // Tries to process a single SBF packet in @sbfData, even if @sbfData contains many SBF packets.
    // This method also removes processed packet-data from @sbfData.
    // Returns true when more packets can be processed in @sbfData, false otherwise.
    // This method does not return errors when it encounters corrupt SBF, as there's no way to fix
    // corrupt SBF anyway.
    bool processSbfData(QByteArray& sbfData);

signals:
    void newVehiclePose(const Pose&); // emitted at 20Hz, includes non-FixedRTK poses
    void newVehiclePosePrecise(const Pose&); // emitted at 20Hz, FixedRTK and IntegratedAttitude, used for sensor-fusion
    void newVehiclePoseLowFreq(const Pose&); // emitted at ~1Hz;

    void processedPacket(const QByteArray& sbfPacket);

//    void newInfo(const quint16&);
//    void newInfoText(const QString&);

//    void newIntegrationMode(const quint8&);
//    void newIntegrationModeText(const QString&);

//    void newGnssMode(const quint8&);
//    void newGnssModeText(const QString&);

//    void newError(const quint8&);
//    void newErrorText(const QString&);

//    void newCpuLoad(const quint8&);
//    void newMeanCorrAge(const quint8&);
//    void newCovariances(const float& maxCov);
//    void newTimeOfWeek(qint32& tow);

    // log/status messages
    void message(const LogImportance& importance, const QString&, const QString& message);
    void status(const GpsStatusInformation::GpsStatus&);

    void receiverCommand(const QString&);

    // Again, timestamp is number of milliseconds since last sunday 00:00:00 AM (midnight)
    void scanFinished(const quint32& timestamp);

    void gpsTimeOfWeekEstablished(const quint32& timestamp);

private slots:
//    void slotEmitCurrentGpsStatus(const QString& text = QString());


};

#endif // SBFPARSER_H
