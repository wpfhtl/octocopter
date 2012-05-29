#include "plymanager.h"

PlyManager::PlyManager(const QString& fileName, const PlyManager::DataDirection& direction) : QObject()
{
    mFileWrite = 0;
    mFileRead = 0;
    mPointsWritten = 0;

    if(direction == DataRead)
    {
        qDebug() << "PlyManager::PlyManager(): opening file" << fileName << "for reading data";
        mFileRead = new QFile(fileName);
        if(!mFileRead->open(QIODevice::ReadOnly | QIODevice::Text))
            qFatal("PlyManager::PlyManager(): couldn't open file %s for reading, exiting.", qPrintable(mFileRead->fileName()));
    }
    else
    {
        qDebug() << "PlyManager::PlyManager(): opening file" << fileName << "for writing data";
        mFileWrite = new QFile(fileName);
        if(!mFileWrite->open(QIODevice::ReadWrite | QIODevice::Text))
            qFatal("PlyManager::PlyManager(): couldn't open file %s for writing, exiting.", qPrintable(mFileWrite->fileName()));

        // Create a ply-header in the file
        QTextStream out(mFileWrite);
        out << createHeader(0, PlyManager::NormalsNotIncluded, PlyManager::DirectionNotIncluded);
    }
}

PlyManager::~PlyManager()
{
    if(mFileRead)
    {
        qDebug() << "PlyManager::~PlyManager(): closing file" << mFileRead->fileName();
        mFileRead->close();
        delete mFileRead;
    }
    if(mFileWrite)
    {
        qDebug() << "PlyManager::~PlyManager(): fixing vertex count in header of" << mFileWrite->fileName();
        // fix "element vertex x" line in header
        QTextStream stream(mFileWrite);
        stream.seek(0);
        while(!stream.atEnd())
        {
                    QString line = stream.readLine();
                    if(line.contains("element vertex"))
                    {
                        stream.seek(stream.pos() - line.length() - 1);
                        stream << QString("element vertex %1").arg(mPointsWritten, 7, 10, QChar(' ')) << endl;
                        break;
                    }
        }
        qDebug() << "PlyManager::~PlyManager(): done, closing file";

        mFileWrite->flush();
        mFileWrite->close();
        delete mFileWrite;
    }
}

void PlyManager::slotNewPoints(const QVector<QVector3D>& points, const QVector3D& scanPosition)
{
    Q_UNUSED(scanPosition);
    Q_ASSERT(mFileWrite != 0 && mFileRead == 0 && "PlyManager::slotNewPoints(): fileWrite is 0 or fileRead is not 0");

    QTextStream stream(mFileWrite);
    stream.setRealNumberPrecision(3);
    stream.setRealNumberNotation(QTextStream::FixedNotation);
    for(int i = 0; i < points.size(); ++i)
        stream << points.at(i).x() << " " << points.at(i).y() << " " << points.at(i).z() << endl;

    mPointsWritten += points.size();
}

void PlyManager::slotNewPoints(const QList<QVector3D>& points, const QVector3D& scanPosition)
{
    Q_UNUSED(scanPosition);
    Q_ASSERT(mFileWrite != 0 && mFileRead == 0 && "PlyManager::slotNewPoints(): fileWrite is 0 or fileRead is not 0");


    QTextStream stream(mFileWrite);
    stream.setRealNumberPrecision(3);
    stream.setRealNumberNotation(QTextStream::FixedNotation);
    for(int i = 0; i < points.size(); ++i)
        stream << points.at(i).x() << " " << points.at(i).y() << " " << points.at(i).z() << endl;

    mPointsWritten += points.size();
}

const QString PlyManager::createHeader(const quint32& vertexCount, const IncludesNormals& includesNormals, const IncludesDirection& includesDirection)
{
    QString header;
    QTextStream stream(&header);
    stream << QString("ply") << endl;
    stream << QString("format ascii 1.0") << endl;
    stream << QString("comment written by koptertools / ben adler on ").append(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")) << endl;
    stream << QString("element vertex %1").arg(vertexCount, 7, 10, QChar(' ')) << endl; // extra space for overwriting with bigger numbers later-on, meshlab doesn't mind.
    stream << QString("comment the coordinates of the point") << endl;
    stream << QString("property float x") << endl;
    stream << QString("property float y") << endl;
    stream << QString("property float z") << endl;

    if(includesNormals == NormalsIncluded)
    {
        stream << QString("comment the normalized vector back to the laserscanner") << endl;
        stream << QString("property float nx") << endl;
        stream << QString("property float ny") << endl;
        stream << QString("property float nz") << endl;
    }

    if(includesDirection == DirectionIncluded)
    {
        stream << QString("comment squared distance") << endl;
        stream << QString("property float sqdist") << endl;
    }

    stream << QString("end_header") << endl;

    return header;
}

bool PlyManager::savePly(const QVector<QVector3D>& points, const QString &fileName)
{
    QFile file(fileName);
    if(!file.open(QFile::WriteOnly | QFile::Truncate))
        return false;

    QTextStream stream(&file);
    stream.setRealNumberPrecision(3);
    stream.setRealNumberNotation(QTextStream::FixedNotation);

    // write header
    stream << createHeader(points.size(), PlyManager::NormalsNotIncluded, PlyManager::DirectionNotIncluded);

    foreach(const QVector3D& point, points)
    {
        stream << point.x() << " " << point.y() << " " << point.z() << endl;
    }
}


bool PlyManager::savePly(const QList<QVector3D>& points, const QString &fileName)
{
    QFile file(fileName);
    if(!file.open(QFile::WriteOnly | QFile::Truncate))
        return false;

    QTextStream stream(&file);
    stream.setRealNumberPrecision(6);
    stream.setRealNumberNotation(QTextStream::FixedNotation);

    // write header
    stream << createHeader(points.size(), PlyManager::NormalsNotIncluded, PlyManager::DirectionNotIncluded);

    foreach(const QVector3D& point, points)
    {
        stream << point.x() << " " << point.y() << " " << point.z() << endl;
    }
}

#ifdef BASESTATION

bool PlyManager::savePly(const QVector<LidarPoint>& points, const QString &fileName)
{
    QFile file(fileName);
    if(!file.open(QFile::WriteOnly | QFile::Truncate))
        return false;

    QTextStream stream(&file);
    stream.setRealNumberPrecision(6);
    stream.setRealNumberNotation(QTextStream::FixedNotation);

    // write header
    stream << createHeader(points.size(), PlyManager::NormalsIncluded, PlyManager::DirectionIncluded);

    foreach(const LidarPoint& point, points)
    {
        stream << point.position.x() << " " << point.position.y() << " " << point.position.z() << " ";
        stream << point.laserPos.x() << " " << point.laserPos.y() << " " << point.laserPos.z();
        stream << endl;
    }
}


bool PlyManager::savePly(QWidget* widget, const Octree* tree, const QString &fileName)
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
    stream << createHeader(tree->getNumberOfItems(), NormalsIncluded, DirectionIncluded);

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

bool PlyManager::savePly(const Node* node, QTextStream* stream, QProgressDialog* progress)
{
    if(progress->wasCanceled()) return false;

    if(node->isLeaf())
    {
        // save the points into @stream
        for(int i=0;i<node->pointIndices.size();i++)
        {
            const LidarPoint point = node->mTree->data()->at(node->pointIndices.at(i));
            (*stream) << point.position.x() << " " << point.position.y() << " " << point.position.z() << " ";
            (*stream) << point.laserPos.x() << " " << point.laserPos.y() << " " << point.laserPos.z();
            (*stream) << endl;
        }

        progress->setValue(progress->value()+node->pointIndices.size());
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

bool PlyManager::loadPly(QWidget* widget, const QList<Octree*>& trees, const QList<FlightPlannerInterface*>& flightPlanners, const QString &fileName)
{
    QFile file(fileName);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QMessageBox::critical(widget, "Error reading file", QString("Unable to open file %1").arg(fileName));
        return false;
    }

    QProgressDialog progress("Loading cloud...", "Update View", 0, 0, widget);
    progress.setWindowModality(Qt::WindowModal);

    bool dataStarted = false;
    quint32 lineNumber = 0;
    quint32 numberOfVerticesTotal = 0;
    quint32 numberOfVerticesProcessed = 0;

    QTextStream in(&file);
    while (!in.atEnd())
    {
        const QString line = in.readLine();

        if(!dataStarted)
        {
            if(line.startsWith("element vertex ", Qt::CaseInsensitive))
            {
                bool ok = false;
                numberOfVerticesTotal = line.split(" ", QString::SkipEmptyParts).last().toInt(&ok);
                if(!ok)
                {
                    QMessageBox::critical(widget, "Error reading file", QString("Unable to parse number of vertices at line %1: %2").arg(lineNumber).arg(line));
                    return false;
                }
                progress.setMaximum(numberOfVerticesTotal);
            }

            if(line == "end_header")
                dataStarted = true;
        }
        else
        {
            QStringList values = line.split(" ", QString::SkipEmptyParts);

            if(values.size() != 7)
            {
                QMessageBox::critical(widget, "Error reading file", QString("Number of elements in line %1 of file %2 is %3, expected 7.").arg(lineNumber).arg(fileName).arg(values.size()));
                return false;
            }
            else
            {
                for(int i=0;i<trees.size();i++)
                {
                    trees.at(i)->insertPoint(
                                new LidarPoint(
                                    QVector3D(
                                        values.at(0).toDouble(),
                                        values.at(1).toDouble(),
                                        values.at(2).toDouble()
                                        ),
                                    QVector3D(
                                        values.at(3).toDouble(),
                                        values.at(4).toDouble(),
                                        values.at(5).toDouble()
                                        )
                                    )
                                );
                }

                for(int i=0;i<flightPlanners.size();i++)
                {
                    flightPlanners.at(i)->insertPoint(
                                new LidarPoint(
                                    QVector3D(
                                        values.at(0).toDouble(),
                                        values.at(1).toDouble(),
                                        values.at(2).toDouble()
                                        ),
                                    QVector3D(
                                        values.at(3).toDouble(),
                                        values.at(4).toDouble(),
                                        values.at(5).toDouble()
                                        )
                                    )
                                );
                }

                numberOfVerticesProcessed++;

                progress.setValue(numberOfVerticesProcessed);
            }
        }
        lineNumber++;
    }
}

#endif
