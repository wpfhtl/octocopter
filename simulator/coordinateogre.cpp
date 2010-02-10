#include "coordinateogre.h"

// we inherit from QVector3D and up until now, don't add anything. Thus, a typedef might be smarter.

CoordinateOgre::CoordinateOgre() : Ogre::Vector3()
{
    this->x = 0.0f;
    this->y = 0.0f;
    this->z = 0.0f;
}

QDebug operator<<(QDebug dbg, const CoordinateOgre &c)
 {
     dbg.nospace() << "(" << c.x << ", " << c.y << ", " << c.z << ")";

     return dbg.space();
 }
