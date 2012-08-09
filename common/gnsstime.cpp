#include "gnsstime.h"

//GnssTime::GnssTime()
//{
//}


qint32 GnssTime::currentTow()
{
    static const QDate today = QDate::currentDate();
    static const QDateTime beginningOfWeek(today.addDays(-(today.dayOfWeek() % 7)), QTime(0, 0, 0, 0));

//    qDebug() << beginningOfWeek.toString("ddd hh:mm:ss:zzz");
    //Q_ASSERT(beginningOfWeek.date().dayOfWeek() == Qt::Sunday && beginningOfWeek.toString("hh:mm:ss:zzz") == QString("00:00:00:000"));

    return beginningOfWeek.msecsTo(QDateTime::currentDateTime());
}
