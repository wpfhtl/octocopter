#include "plymanager.h"

PlyManager::PlyManager(QWidget *widget) : QObject()
{
    mFile = 0;
    mNumberOfPoints = 0;
    mProgressDialog = 0;
    mWidget = widget;

    if(mWidget)
    {
        mProgressDialog = new QProgressDialog("Processing cloud...", "Cancel", 0, 100, widget);
        mProgressDialog->setWindowModality(Qt::WindowModal);
    }
}

bool PlyManager::open(const QString& fileName, const PlyManager::DataDirection& direction)
{
    mDataDirection = direction;
    if(mDataDirection == DataLoadFromFile)
    {
        qDebug() << "PlyManager::PlyManager(): opening file" << fileName << "for loading data";
        mFile = new QFile(fileName);
        if(!mFile->open(QIODevice::ReadOnly | QIODevice::Text))
        {
            qDebug() << "PlyManager::open(): couldn't open file" << mFile->fileName() << "for reading, exiting.";
            return false;
        }

        // Find out how many vertices we have
        QTextStream stream(mFile);
        stream.seek(0);
        while(!stream.atEnd())
        {
            const QString line = stream.readLine();
            if(line.contains("element vertex"))
            {
                const QStringList lineList = line.split(' ', QString::SkipEmptyParts, Qt::CaseSensitive);
                bool success = false;
                mNumberOfPoints = lineList.last().toInt(&success);
                if(!success)
                {
                    qDebug() << "PlyManager::open(): unable to determine number of points in file, line was:" << line;
                    return false;
                }
                qDebug() << "PlyManager::PlyManager(): file" << fileName << "contains" << mNumberOfPoints << "points.";
                break;
            }
        }
    }
    else
    {
        qDebug() << "PlyManager::PlyManager(): opening file" << fileName << "for writing data";
        mFile = new QFile(fileName);
        if(!mFile->open(QIODevice::ReadWrite | QIODevice::Text | QIODevice::Truncate))
        {
            qDebug() << "PlyManager::open(): couldn't open file" << mFile->fileName() << "for writing, exiting.";
            return false;
        }
    }

    return true;
}

PlyManager::~PlyManager()
{
    if(mFile)
    {
        qDebug() << "PlyManager::~PlyManager(): closing file" << mFile->fileName();
        mFile->close();
        delete mFile;
    }

    delete mProgressDialog;
}

quint32 PlyManager::getNumberOfPoints() const
{
    Q_ASSERT(mDataDirection == DataLoadFromFile);

    return mNumberOfPoints;
}

void PlyManager::writeHeader(const quint32& vertexCount, const IncludesNormals& includesNormals, const IncludesDirection& includesDirection)
{
    Q_ASSERT(mDataDirection == DataSaveToFile);

    QTextStream stream(mFile);
    stream << QString("ply") << endl;
    stream << QString("format ascii 1.0") << endl;
    stream << QString("comment written by koptertools / ben adler")/*.append(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"))*/ << endl; // no timestamp, that prevents checksum comparisons
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
}

void PlyManager::savePly4D(const float* const points, const quint32 numPoints)
{
    QTextStream stream(mFile);
    stream.setRealNumberPrecision(4);
    stream.setRealNumberNotation(QTextStream::FixedNotation);
    for(int i = 0; i < numPoints; ++i)
    {
        stream << points[4*i+0] << " " << points[4*i+1] << " " << points[4*i+2] << endl;
        if(mProgressDialog && i%1000 == 0) mProgressDialog->setValue((i*100)/numPoints);
    }

    mProgressDialog->close();
}

void PlyManager::loadPly4D(float* points, const quint32 startPoint, const quint32 maxPoints)
{
    bool dataStarted = false;
    quint32 lineNumber = 0;
    quint32 vertexNumber = 0;
    quint32 numberOfVerticesProcessed = 0;

    if(mProgressDialog)
    {
        mProgressDialog->setValue(0);
        mProgressDialog->setMaximum(maxPoints);
        mProgressDialog->show();
    }

    QTextStream in(mFile);
    in.seek(0);
    while (!in.atEnd())
    {
        const QString line = in.readLine();

        if(!dataStarted)
        {
            if(line == "end_header")
                dataStarted = true;
        }
        else
        {
            const QStringList values = line.split(' ', QString::SkipEmptyParts);

            if(values.size() != 3)
            {
                QMessageBox::critical(mWidget, "Error reading file", QString("Number of elements in line %1 of file %2 is %3, expected 3.").arg(lineNumber).arg(mFile->fileName()).arg(values.size()));
                return;
            }
            else
            {
                if(vertexNumber >= startPoint && numberOfVerticesProcessed < maxPoints)
                {
                    points[numberOfVerticesProcessed*4 + 0] = values.at(0).toDouble();

                    // junhao: use z as height and mirror y axis
                    points[numberOfVerticesProcessed*4 + 1] = values.at(2).toDouble();
                    points[numberOfVerticesProcessed*4 + 2] = -values.at(1).toDouble();

//                    points[numberOfVerticesProcessed*4 + 1] = values.at(1).toDouble();
//                    points[numberOfVerticesProcessed*4 + 2] = values.at(2).toDouble();

                    points[numberOfVerticesProcessed*4 + 3] = 1.0;
                    numberOfVerticesProcessed++;
                    if(mProgressDialog && numberOfVerticesProcessed % 20000 == 0) mProgressDialog->setValue(numberOfVerticesProcessed);
                }
            }
        }
        lineNumber++;
    }
}
