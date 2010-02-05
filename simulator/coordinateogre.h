#ifndef COORDINATEOGRE_H
#define COORDINATEOGRE_H

#include <QVector3D>
#include <QDebug>

class CoordinateOgre : public QVector3D
{
//Q_OBJECT

public:
    CoordinateOgre();
};

QDebug operator<<(QDebug dbg, const CoordinateOgre &c);

#endif
