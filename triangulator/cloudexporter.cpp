#include "cloudexporter.h"

CloudExporter::CloudExporter(Octree* const octree) : QObject()
{
    qDebug() << "CloudExporter::CloudExporter()";
    mOctree = octree;
}

CloudExporter::~CloudExporter()
{
}
