#include "gnsstime.h"

qint32 GnssTime::currentTow()
{
    // Uhhhhhh.... don't you fly at midnight!
    static const QDate today = QDate::currentDate();
    static const QDateTime beginningOfWeek(today.addDays(-(today.dayOfWeek() % 7)), QTime(0, 0, 0, 0));

//    qDebug() << beginningOfWeek.toString("ddd hh:mm:ss:zzz");
    //Q_ASSERT(beginningOfWeek.date().dayOfWeek() == Qt::Sunday && beginningOfWeek.toString("hh:mm:ss:zzz") == QString("00:00:00:000"));

    // We add 16 seconds offset between UTC and GPS time
    return beginningOfWeek.msecsTo(QDateTime::currentDateTime().addMSecs(16000));
}

QString GnssTime::currentTowString()
{
    QString towString = QString::number(currentTow());
    int i = towString.size()-3;
    while(i > 0)
    {
        towString.insert(i, '.');
        i -= 3;
    }

    return towString;
}
