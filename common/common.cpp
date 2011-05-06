#include "common.h"

QString hash(QList<WayPoint> list)
{
    QCryptographicHash hash(QCryptographicHash::Md4);
    foreach(const WayPoint v, list)
    {
        hash.addData(QByteArray::number(v.x()));
        hash.addData(QByteArray::number(v.y()));
        hash.addData(QByteArray::number(v.z()));
    }

    return hash.result();
}


quint32 getCurrentGpsTowTime()
{
    const QDate today = QDate::currentDate();
    QDateTime beginningOfWeek(today.addDays(-(today.dayOfWeek() % 7)), QTime(0, 0, 0, 0));

//    qDebug() << beginningOfWeek.toString("ddd hh:mm:ss:zzz");
    Q_ASSERT(beginningOfWeek.date().dayOfWeek() == Qt::Sunday && beginningOfWeek.toString("hh:mm:ss:zzz") == QString("00:00:00:000"));

    return beginningOfWeek.msecsTo(QDateTime::currentDateTime());
}
