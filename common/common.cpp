#include "common.h"

qint32 getCurrentGpsTowTime()
{
    const QDate today = QDate::currentDate();
    QDateTime beginningOfWeek(today.addDays(-(today.dayOfWeek() % 7)), QTime(0, 0, 0, 0));

//    qDebug() << beginningOfWeek.toString("ddd hh:mm:ss:zzz");
    //Q_ASSERT(beginningOfWeek.date().dayOfWeek() == Qt::Sunday && beginningOfWeek.toString("hh:mm:ss:zzz") == QString("00:00:00:000"));

    return beginningOfWeek.msecsTo(QDateTime::currentDateTime());
}

//bool isBitSet(quint8 number, quint8 bit)
//{
//    return ((number & ipow(2, bit)) != 0);
//}

bool testBit(quint16 number, quint8 bit)
{
    return ((number & ipow(2, bit)) != 0);
}

bool testBitEqual(quint16 number1, quint16 number2, quint8 bit)
{
    quint16 mask = ipow(2, bit);

    return (number1 & mask) == (number2 & mask);
}

QString t() { return QTime::currentTime().toString("HH:mm:ss:zzz"); }



QString getFlightStateString(const FlightState flightState)
{
    switch(flightState)
    {
    case UserControl: return "UserControl"; break;
    case ApproachWayPoint: return "ApproachWayPoint"; break;
    case Hover: return "Hover"; break;
    case Idle: return "Idle"; break;
    }

    qDebug() << "getFlightStateString(): FLIGHTSTATE" << flightState << "UNDEFINED!";
}

FlightState getFlightState(const QString& flightStateString)
{
    if(flightStateString == "UserControl")
        return UserControl;
    else if(flightStateString == "ApproachWayPoint")
        return ApproachWayPoint;
    if(flightStateString == "Hover")
        return Hover;
    if(flightStateString == "Idle")
        return Idle;

    qDebug() << "undefined flightstate string!";
}
