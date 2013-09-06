#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <QVector>
#include <QVector3D>
#include <QColor>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLFunctions_4_3_Core>
#include <QOpenGLVertexArrayObject>
#include <plymanager.h> // for saving to .ply files
#include <common.h>

// This defines the INTERFACE of all pointclouds!
class PointCloud : public QObject, public OPENGL_FUNCTIONS_CLASS
{
    Q_OBJECT
protected:
    bool mAcceptPointsOutsideBoundingBox;

public:

    void setAcceptPointsOutsideBoundingBox(bool accept) { mAcceptPointsOutsideBoundingBox = accept; }
    bool acceptPointsOutsideBoundingBox() const { return mAcceptPointsOutsideBoundingBox; }

    QString mName; // for debugging only

    // Every pointcloud needs to offer a list of VBOs to render. The list can be just a single VboInfo with a single VBO.
    struct RenderInfo
    {
        QOpenGLVertexArrayObject* vao;
        quint32 vbo;
        quint8 elementSize; // number of floats, usually 3 or 4
        QColor color; // color to be used for rendering, invalid by default
        quint32 size; // the number of points
        quint8 stride; // stride between consecutive elements

        RenderInfo()
        {
            // don't create the VAO yet. A non-existing VAO will be configured during first render!
            //qDebug() << __PRETTY_FUNCTION__;
            vao = 0;
            vbo = 0;
            elementSize = 4;
            color = QColor();
            size = 0;
            stride = 0;
        }

        ~RenderInfo()
        {
            if(vao) vao->deleteLater();
        }

        bool layoutMatches(const RenderInfo* const otherVbo) const
        {
            return elementSize == otherVbo->elementSize && stride == otherVbo->stride;
        }
    };

    PointCloud(const Box3D &boundingBox);
    ~PointCloud();

    virtual QVector<RenderInfo*>* getRenderInfo() = 0;

    virtual void setMinimumPointDistance(const float &distance) = 0;
//    virtual float getMinimumPointDistance() const = 0;

    virtual quint32 getNumberOfPoints(void) const = 0;
    virtual quint32 getCapacity(void) const = 0;

    virtual bool exportToFile(const QString& fileName, QWidget* widget = 0) = 0;
    virtual bool importFromFile(const QString& fileName, QWidget* widget = 0) = 0;

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
