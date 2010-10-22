#include "cloudexporter.h"

CloudExporter::CloudExporter() : QObject()
{
    qDebug() << "CloudExporter::CloudExporter()";
}

CloudExporter::~CloudExporter()
{
}

bool CloudExporter::savePly(QWidget* widget, const Octree* tree, const QString &fileName)
{
        QFile file(fileName);
        if(!file.open(QFile::WriteOnly | QFile::Truncate))
                return false;

        QTextStream stream(&file);
        stream.setRealNumberPrecision(6);
        stream.setRealNumberNotation(QTextStream::FixedNotation);

        QProgressDialog progress("Saving cloud...", "Cancel", 0, tree->getNumberOfItems(), widget);
        progress.setWindowModality(Qt::WindowModal);

        // write header
        stream << QString("ply") << endl;
        stream << QString("format ascii 1.0") << endl;
        stream << QString("comment written by triangulator / ben adler on ").append(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")) << endl;
        stream << QString("element vertex ").append(QString::number(tree->getNumberOfItems())) << endl;
        stream << QString("comment the coordinates of the point") << endl;
        stream << QString("property float x") << endl;
        stream << QString("property float y") << endl;
        stream << QString("property float z") << endl;
        stream << QString("comment the normalized vector back to the laserscanner") << endl;
        stream << QString("property float nx") << endl;
        stream << QString("property float ny") << endl;
        stream << QString("property float nz") << endl;
        stream << QString("comment squared distance") << endl;
        stream << QString("property float sqdist") << endl;
        stream << QString("end_header") << endl;

        if(savePly(tree->root(), &stream, &progress))
        {
            file.close();
            return true;
        }
        else
        {
            file.close();
            file.remove();
            return false;
        }

}

bool CloudExporter::savePly(const Node* node, QTextStream* stream, QProgressDialog* progress)
{
    if(progress->wasCanceled()) return false;

    if(node->isLeaf())
    {
        // save the points into @stream
        foreach(const LidarPoint* point, node->data)
        {
            QVector3D normal = point->direction.normalized();

            (*stream) << point->position.x() << " " << point->position.y() << " " << point->position.z() << " ";
            (*stream) << normal.x() << " " << normal.y() << " " << normal.z() << " ";
            (*stream) << point->squaredDistance;
            (*stream) << endl;
        }

        progress->setValue(progress->value()+node->data.size());
    }
    else
    {
        // invoke recursively for childnodes/leafs
        const QList<const Node*> childNodes = node->getAllChildLeafs();
        foreach(const Node* childNode, childNodes)
            if(!savePly(childNode, stream, progress))
                return false;
    }

    return true;
}

