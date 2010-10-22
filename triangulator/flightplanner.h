#ifndef FLIGHTPLANNER_H
#define FLIGHTPLANNER_H

#include <QObject>
#include "octree.h"
#include "node.h"
#include "lidarpoint.h"
#include "glwidget.h"

#include <btBulletDynamicsCommon.h>

class FlightPlanner : public QObject
{
    Q_OBJECT
public:
    FlightPlanner(QObject *parent = 0);
    ~FlightPlanner();

    Node* insertPoint(LidarPoint* const point);

    QVector<QVector3D> getNextRoute();

    void visualize() const;

private:

    Octree* mOctree;
    btTransform mLidarPointTransform;
    btCollisionShape *mShapeLidarPoint, *mShapeSampleSphere, *mShapeFloor;


//    btAxisSweep3 *mBtBroadphase;
    btDbvtBroadphase *mBtBroadphase;

    btDefaultCollisionConfiguration *mBtCollisionConfig;
    btCollisionDispatcher *mBtDispatcher;
    btSequentialImpulseConstraintSolver *mBtSolver;
    btDiscreteDynamicsWorld *mBtWorld;

signals:

public slots:

};

#endif // FLIGHTPLANNER_H
