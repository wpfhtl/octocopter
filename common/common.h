#ifndef COMMON_H
#define COMMON_H

#include <QList>
#include <QVector3D>
#include <QString>
#include <QDateTime>
#include <QDebug>

#define RAD2DEG(RAD) ((RAD)*180/M_PI)
#define DEG2RAD(DEG) ((DEG)*((M_PI)/(180.0)))

#define SQUARE(X) ((X)*(X))

// So I can do qDebug() << "string is" << Q(myStringObject) << "."; without having quotes around the string-value inthe output
#define Q(string) (string).toStdString().c_str()

template <typename T>
T swap_endian(T u)
{
    union
    {
        T u;
        unsigned char u8[sizeof(T)];
    } source, dest;

    source.u = u;

    for (size_t k = 0; k < sizeof(T); k++)
        dest.u8[k] = source.u8[sizeof(T) - k - 1];

    return dest.u;
}

struct PositionGeodetic
{
    // degrees, meters
    float longitude;
    float latitude;
    float elevation;
};

enum class WayPointListSource
{
    WayPointListSourceControlWidget,
    WayPointListSourceFlightPlanner,
    WayPointListSourceRover,
    WayPointListSourceUnknown
};

enum class OperatingMode
{
    OperatingOnline,    // connected to simulator or kopter
    OperatingOffline    // using LogPlayer
};

struct Box3D
{
    QVector3D min,max;

    Box3D() {}
    Box3D(const QVector3D& minBox, const QVector3D& maxBox) {min = minBox; max = maxBox;}
    QVector3D size() const {return max - min;}
    QVector3D center() const {return min + (max - min)/2.0f;}

    Box3D tryToKeepWithin(const Box3D& other) const
    {
        const QVector3D otherCenter = other.center();
        const QVector3D otherSize = other.size();
        Box3D result;

        if(size().x() <= otherSize.x())
        {
            // fitting is possible - move within bounds if necessary!
            if(min.x() < other.min.x())
            {
                result.min.setX(other.min.x());
                result.max.setX(result.min.x() + size().x());
            }
            else if(max.x() > other.max.x())
            {
                result.min.setX(other.max.x() - size().x());
                result.max.setX(other.max.x());
            }
            else
            {
                result.min.setX(min.x());
                result.max.setX(max.x());
            }
        }
        else
        {
            // box is too large in this dimension, so place it in center of other
            result.min.setX(otherCenter.x() - size().x()/2.0f);
            result.max.setX(otherCenter.x() + size().x()/2.0f);
        }

        if(size().y() <= otherSize.y())
        {
            // fitting is possible - move within bounds if necessary!
            if(min.y() < other.min.y())
            {
                result.min.setY(other.min.y());
                result.max.setY(result.min.y() + size().y());
            }
            else if(max.y() > other.max.y())
            {
                result.min.setY(other.max.y() - size().y());
                result.max.setY(other.max.y());
            }
            else
            {
                result.min.setY(min.y());
                result.max.setY(max.y());
            }
        }
        else
        {
            // box is too large in this dimension, so place it in center of other
            result.min.setY(otherCenter.y() - size().y()/2.0f);
            result.max.setY(otherCenter.y() + size().y()/2.0f);
        }


        if(size().z() <= otherSize.z())
        {
            // fitting is possible - move within bounds if necessary!
            if(min.z() < other.min.z())
            {
                result.min.setZ(other.min.z());
                result.max.setZ(result.min.z() + size().z());
            }
            else if(max.z() > other.max.z())
            {
                result.min.setZ(other.max.z() - size().z());
                result.max.setZ(other.max.z());
            }
            else
            {
                result.min.setZ(min.z());
                result.max.setZ(max.z());
            }
        }
        else
        {
            // box is too large in this dimension, so place it in center of other
            result.min.setZ(otherCenter.z() - size().z()/2.0f);
            result.max.setZ(otherCenter.z() + size().z()/2.0f);
        }

        return result;
    }
};

// for using qDebug()
QDebug operator<<(QDebug dbg, const Box3D &box);


//bool isBitSet(quint8 number, quint8 bit);
bool testBit(quint16 number, quint8 bit);
bool testBitEqual(quint16 number1, quint16 number2, quint8 bit);

// fast exponentiation algorithm
inline int ipow(int base, int exp)
{
    int result = 1;
    while (exp)
    {
        if (exp & 1)
            result *= base;
        exp >>= 1;
        base *= base;
    }

    return result;
}

QString t();

enum LogImportance
{
    Information,
    Warning,
    Error,
    Desaster
};


// compute the next higher power of 2 of 32-bit v
/*
quint32 nextHigherPowerOfTwo(quint32 v)
{
    // decrements, then sets all bits below its most significant bit to 1, then it increments
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    return v + 1;
}*/

struct Vector3i
{
    qint16 x,y,z;

    Vector3i() {x = y = z = 0;}
    Vector3i(qint16 x, qint16 y, qint16 z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }
};

#endif // COMMON_H
