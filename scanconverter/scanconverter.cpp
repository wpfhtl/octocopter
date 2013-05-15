#include <QtCore>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    if(a.arguments().size() != 2)
    {
        qDebug() << "usage: scanconverter <ascii-scan-file-to-convert.lsr>";
        exit(0);
    }

    QString fileNameSource = a.arguments().last();

    QFile* fileSource = new QFile(fileNameSource);
    if(fileSource->open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "opened file for reading:" << fileNameSource;
    }
    else
    {
        qDebug() << "couldn't open scan file" << fileSource->fileName() << "for reading, exiting.";
        exit(1);
    }

    QString fileNameTarget(fileNameSource);
    fileNameTarget.replace(".lsr", ".lsrb");

    Q_ASSERT(fileNameSource != fileNameTarget);

    QFile* fileTarget = new QFile(fileNameTarget);
    if(fileTarget->open(QIODevice::WriteOnly))
    {
        qDebug() << "opened file for writing:" << fileNameTarget;
    }
    else
    {
        qDebug() << "couldn't open scan file" << fileTarget->fileName() << "for writing, exiting.";
        exit(1);
    }

//    QDataStream stream(fileTarget);
    bool success = false;

    quint32 line = 0;
    // Read the source file, line by line
    while(!fileSource->atEnd())
    {
        line++;
        QList<QByteArray> values = fileSource->readLine().trimmed().split(' ');

        qint32 tow = values.takeFirst().toInt(&success);
        Q_ASSERT(success);


        quint16 indexStart = 0;
        while(values.at(indexStart).toInt() == 1)
            indexStart++;

        quint16 indexStop = values.size() - 1;
        while(values.at(indexStop).toInt() == 1)
            indexStop--;


        // Write the total amount of bytes of this scan into the stream
        quint16 length = 5 // LASER
                + sizeof(quint16) // length at beginning
                + sizeof(tow)
                + sizeof(indexStart)
                + ((indexStop - indexStart ) + 1) * sizeof(quint16); // size of the distance-data

        qDebug()
                << "converting scan" << line
                << "from tow:" << tow
                << "with indices" << indexStart << "/" << values.at(indexStart) << "to" << indexStop << "/" << values.at(indexStop)
                << "of" << values.size() << "values,"
                << "packetsize" << length;

        QByteArray magic("LASER");
        Q_ASSERT(magic.size() == 5);

        fileTarget->write(magic.constData(), magic.size());
        fileTarget->write((const char*)&length, sizeof(length));
        fileTarget->write((const char*)&tow, sizeof(tow));
        fileTarget->write((const char*)&indexStart, sizeof(indexStart));

        while(indexStart<=indexStop)
        {
            quint16 distance = (quint16)values.at(indexStart).toInt(&success);
            Q_ASSERT(success);
            fileTarget->write((const char*)&distance, sizeof(distance));
            indexStart++;
        }
    }

    qDebug() << "done converting.";

    fileSource->close();
    delete fileSource;

    fileTarget->close();
    delete fileTarget;
}
