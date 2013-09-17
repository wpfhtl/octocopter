#include "pointcloud.h"

PointCloud::PointCloud(const Box3D &boundingBox)
{
    mOutlierTreatment = OutlierTreatment::Remove;
}

PointCloud::~PointCloud()
{
}
