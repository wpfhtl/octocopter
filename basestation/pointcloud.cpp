#include "pointcloud.h"

PointCloud::PointCloud()
{
    mOutlierTreatment = OutlierTreatment::Remove;
}

PointCloud::~PointCloud()
{
}
