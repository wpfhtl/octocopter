#ifndef PORT_H
#define PORT_H

#include <QtCore>

class Port : public QObject
{
    Q_OBJECT

signals:
    void data(const QByteArray&);

public slots:
    virtual void write(const QByteArray&) = 0;
};

#endif
