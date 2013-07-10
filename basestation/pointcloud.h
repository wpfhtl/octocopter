#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <QVector>
#include <QVector3D>
#include <QColor>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLFunctions_4_3_Core>

#include <plymanager.h> // for saving to .ply files

// This defines the INTERFACE of all pointclouds!

class PointCloud : public QObject, public OPENGL_FUNCTIONS_CLASS
{
    Q_OBJECT
protected:
//    QVector3D mBBoxMin, mBBoxMax;
    bool mAcceptPointsOutsideBoundingBox;

public:

    void setAcceptPointsOutsideBoundingBox(bool accept) { mAcceptPointsOutsideBoundingBox = accept; }
    bool acceptPointsOutsideBoundingBox() const { return mAcceptPointsOutsideBoundingBox; }

    QString mName; // for debugging only

    // Every pointcloud needs to offer a list of VBOs to render. The list can be just a single VboInfo with a single VBO.
    struct VboInfo
    {
        quint32 vbo;
        quint8 elementSize; // number of floats, usually 3 or 4
        QColor color; // color to be used for rendering, invalid by default
        quint32 size; // the number of points
        quint8 stride; // stride between consecutive elements

        VboInfo()
        {
            vbo = 0;
            elementSize = 4;
            color = QColor();
            size = 0;
            stride = 0;
        }

        bool layoutMatches(const VboInfo* const otherVbo) const
        {
            return elementSize == otherVbo->elementSize && stride == otherVbo->stride;
        }
    };

    PointCloud(const QVector3D &min, const QVector3D &max);
    ~PointCloud();

//    const QVector3D& getBoundingBoxMin() const { return mBBoxMin; }
//    const QVector3D& getBoundingBoxMax() const { return mBBoxMax; }

//    virtual void setBoundingBox(const QVector3D& min, const QVector3D& max) = 0;



    virtual const QVector<VboInfo>& getVboInfo() const = 0;

    virtual void setMinimumPointDistance(const float &distance) = 0;
    virtual float getMinimumPointDistance() const = 0;

    virtual quint32 getNumberOfPoints(void) const = 0;
    virtual quint32 getCapacity(void) const = 0;

    virtual bool exportToPly(const QString& fileName, QWidget* widget = 0) = 0;
    virtual bool importFromPly(const QString& fileName, QWidget* widget = 0) = 0;


    /*
    // Returns the N nearest neighbors of a given point in space. Result is not sorted by distance to @point.
    QList<const QVector4D*> findNearestNeighbors(const QVector4D &point, const quint8 count) const;

    // Returns a list of QVector4Ds in @radius of @point.
    QList<const QVector4D*> findNeighborsWithinRadius(const QVector4D &point, const float radius) const;

    // Returns only the number of points in @radius of @point. More efficient than the methods above
    // if you're not interested in the points themselves
    quint32 numberOfNeighborsWithinRadius(const QVector4D &point, const float radius) const;

    // Yet more efficient: check whether there is AT LEAST ONE point within @radius of@point
    bool isNeighborWithinRadius(const QVector4D &point, const float radius) const;

    // Sort @list of points according to distance to @point
    void sortPointList(const QVector4D &point, QList<const QVector4D*>* list) const;
    */


public slots:
    // Clears the datastructure, but does not destruct it. Points can still be inserted afterwards.
    virtual void slotReset() = 0;
    virtual void slotReduce() = 0;

    // Tell the cloud to insert the given points. As you can see, the cloud may not change or even own the points.
    // Returns true if successful, false if not (e.g. full)
    virtual bool slotInsertPoints(const QVector<QVector3D>* const pointList) = 0;
    virtual bool slotInsertPoints(const QVector<QVector4D>* const pointList) = 0;
    virtual bool slotInsertPoints3(const float* const pointList, const quint32 numPoints) = 0;
    virtual bool slotInsertPoints4(const float* const pointList, const quint32 numPoints) = 0;

signals:
    // A signal to make others aware that new points are present in the pointcloud. This signal is not guaranteed
    // to be fired whenever there are new points (e.g. after every scan), because that would cause a lot of signals.
    // Instead, the pointcloud is free to define larger intervals between signals.
    void pointsInserted(PointCloud* const pointcloud, const quint32& firstPoint, const quint32& numPoints);
    void numberOfPoints(quint32 numberOfPoints);
};

#endif
