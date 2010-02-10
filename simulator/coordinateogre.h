#ifndef COORDINATEOGRE_H
#define COORDINATEOGRE_H

#include <Ogre.h>
#include <QDebug>

class CoordinateOgre : public Ogre::Vector3//QVector3D
{
//Q_OBJECT

public:
    CoordinateOgre();
};

QDebug operator<<(QDebug dbg, const CoordinateOgre &c);

#endif
