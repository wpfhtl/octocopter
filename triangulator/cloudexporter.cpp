#include "cloudexporter.h"

CloudExporter::CloudExporter() : QObject()
{
    qDebug() << "CloudExporter::CloudExporter()";
}

CloudExporter::~CloudExporter()
{
}

bool CloudExporter::savePly(const Octree* tree, const QString &fileName)
{
	QFile file(fileName);
	if(!file.open(QFile::WriteOnly | QFile::Truncate))
		return false;

	QTextStream stream(&file);
	stream.setRealNumberPrecision(6);
	stream.setRealNumberNotation(QTextStream::FixedNotation);

	// write header
	stream << QString("ply");
        stream << QString("format ascii 1.0");
        stream << QString("comment written by traingulator / ben adler on ").append(QDateTime::currentDateTime().toString("yyyy-mm-dd hh:ii:ss"));
        stream << QString("element vertex ").append(QString::number(tree->numberOfItems()));
        stream << QString("comment the coordinates of the point");
	stream << QString("property float x"); // FIXME: double?
	stream << QString("property float y");
	stream << QString("property float z");
        stream << QString("comment the vector back to the laserscanner");
        stream << QString("property float lidarx"); // FIXME: double?
        stream << QString("property float lidary");
        stream << QString("property float lidarz");
        stream << QString("comment squared distance");
	stream << QString("property float sqdist");
	stream << QString("end_header");

	savePly(tree->root(), &stream);

	file.close();

	return true;
}

void CloudExporter::savePly(const Node* node, QTextStream* stream)
{
	if(node->isLeaf())
	{
		// save the points into @stream
		foreach(const LidarPoint* point, node->data)
		{
			(*stream) << point->position.x() << point->position.y() << point->position.z();
			(*stream) << point->direction.x() << point->direction.y() << point->direction.z();
			(*stream) << point->squaredDistance;
			(*stream) << endl;
		}
	}
	else
	{
		// invoke recursively for childnodes/leafs
		const QList<const Node*> childNodes = node->getAllChildLeafs();
		foreach(const Node* childNode, childNodes)
			savePly(childNode, stream);
	}
}

