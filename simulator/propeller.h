#ifndef ENGINE_H
#define ENGINE_H

#include <QObject>
#include <QMap>
#include <QString>

class Engine : public QObject
{
Q_OBJECT
private:
    struct Propeller
    {
//        QString product;
        double c1, c2, c3;
        int rpmMin, rpmMax;
        float diameter;
        float pitch;
    };

    QMap<QString, Propeller> mPropellers;

    void initializePropellers();

public:
    Engine(QObject *parent = 0);

signals:

public slots:

};

#endif
