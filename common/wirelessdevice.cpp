#include "wirelessdevice.h"

WirelessDevice::WirelessDevice(const QString& interfaceName) : QObject()
{
    mInterfaceName = interfaceName;

    mUpdateTimer = new QTimer;
    connect(mUpdateTimer, SIGNAL(timeout()), SLOT(slotEmitRssi()));
    mUpdateTimer->start(250);
}

qint8 WirelessDevice::getRssi()
{
    QFile file("/proc/net/wireless");

    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "WirelessDevice::getRssi(): couldn't open file" << file.fileName();
        return -1;
    }

    /*
      Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE
       face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22
       wlan0: 0000   27.  -83.  -256        0      0      0      0      0        0

      Maximum link quality is 70, so we use that as a 100% scale.
    */

    QStringList lines = QTextStream(&file).readAll().split('\n');

    for(int i=lines.size()-1;i>=0;i--)
    {
        QStringList fields = lines.at(i).simplified().split(' ');

        if(fields.size() < 11) continue;

        if(fields.at(0).left(mInterfaceName.length()) == mInterfaceName.toLower())
        {
            qint16 link = QString(fields.at(2)).replace('.', "").toInt();
//            qint16 level = QString(fields.at(3)).replace('.', "").toInt();
//            qint16 noise = QString(fields.at(4)).replace('.', "").toInt();

//            qDebug() << "WirelessDevice::getRssi():" << ((float)link)/70.0*100.0 << link << level << noise;

            return (int)((float)link)/70.0*100.0;
        }
    }

    return -1;
}

void WirelessDevice::slotEmitRssi()
{
    emit rssi(getRssi());
}
