#include "plymanager.h"
#include "common.h"

PointCloudReaderPly::PointCloudReaderPly(const QString& fileName)
{
    mNumberOfPoints = -1;
    mFile = new QFile(fileName);
    mDataFormat = DataFormatAscii;
}

PointCloudReaderPly::~PointCloudReaderPly()
{
    delete mFile;
}

bool PointCloudReaderPly::open()
{
    return mFile->open(/*QIODevice::Text | */QIODevice::ReadOnly);
}

qint64 PointCloudReaderPly::getNumberOfPoints()
{
    mFile->seek(0);
    QTextStream stream(mFile);
    QString line;
    do {
        line = stream.readLine();
        if(line.toLower().contains("element vertex"))
        {
            bool success = false;
            mNumberOfPoints = line.split(" ", QString::SkipEmptyParts).last().toLongLong(&success);
            if(!success) mNumberOfPoints = -1;
        }
        else if(line.toLower().startsWith("format "))
        {
            const QString format = line.split(" ", QString::SkipEmptyParts).at(1);
            if(format.contains("ascii")) mDataFormat = DataFormatAscii;
            else if(format.contains("little")) mDataFormat = DataFormatBinaryLittleEndian;
            else if(format.contains("big")) mDataFormat = DataFormatBinaryBigEndian;
        }
        else if(line.toLower().contains("end_header"))
        {
            return mNumberOfPoints;
        }
    } while (!line.isNull());

    return mNumberOfPoints;
}

qint64 PointCloudReaderPly::readPoints(float* target, qint64 maxNumberOfPoints, QWidget* widget)
{
    if(!mFile->isOpen())
        return -1;

    if(maxNumberOfPoints < 0)
        getNumberOfPoints();

    if(maxNumberOfPoints < 0)
        return -1;

    QProgressDialog* progressDialog = 0;

    if(widget != nullptr)
    {
        progressDialog = new QProgressDialog("Processing cloud...", "Cancel", 0, 100, widget);
        progressDialog->setWindowModality(Qt::WindowModal);
        progressDialog->setValue(0);
        progressDialog->setMaximum(std::min(mNumberOfPoints, maxNumberOfPoints));
        progressDialog->show();
    }

    const qint64 pointsToRead = std::min(maxNumberOfPoints, mNumberOfPoints);

    if(mDataFormat == DataFormatAscii)
    {
        QTextStream in(mFile);
        in.seek(0);
        bool dataStarted = false;
        qint64 lineNumber = 0;
        qint64 numberOfPointsProcessed = 0;
        while(!in.atEnd())
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
                    if(widget != nullptr)
                        QMessageBox::critical(widget, "Error reading file", QString("Number of coordinates element line %1 of file %2 is %3, expected 3.").arg(lineNumber).arg(mFile->fileName()).arg(values.size()));
                    return numberOfPointsProcessed;
                }
                else
                {
                    target[numberOfPointsProcessed*4 + 0] = values.at(0).toDouble();
                    target[numberOfPointsProcessed*4 + 1] = values.at(1).toDouble();
                    target[numberOfPointsProcessed*4 + 2] = values.at(2).toDouble();
                    target[numberOfPointsProcessed*4 + 3] = 1.0;
                    numberOfPointsProcessed++;
                    if(progressDialog && numberOfPointsProcessed % 20000 == 0) progressDialog->setValue(numberOfPointsProcessed);
                }
            }
            lineNumber++;
            if(numberOfPointsProcessed >= maxNumberOfPoints) break;
        }
    }
    else
    {
        mFile->seek(0);
        forever
        {
            const QString line = mFile->readLine();
            if(line.toLower().contains("end_header")) break;
        }

        for(qint64 i=0;i<pointsToRead;i++)
        {
            float x,y,z;
            mFile->read((char*)&x, sizeof(float));
            mFile->read((char*)&y, sizeof(float));
            mFile->read((char*)&z, sizeof(float));

            if(mDataFormat == DataFormatBinaryBigEndian)
            {
                x = swap_endian(x);
                y = swap_endian(y);
                z = swap_endian(z);
            }

            target[4*i+0] = x;
            target[4*i+1] = y;
            target[4*i+2] = z;
            target[4*i+3] = 1.0f;

            if(progressDialog && i % 20000 == 0) progressDialog->setValue(i);
        }
    }

    delete progressDialog;
    return pointsToRead;
}

void PointCloudReaderPly::close()
{
    mFile->close();
}



































PointCloudWriterPly::PointCloudWriterPly(const QString& fileName)
{
    mFile = new QFile(fileName);
    mDataFormat = DataFormatBinaryLittleEndian;
    mTotalNumberOfPointsWrittenToThisFile = 0;
}

PointCloudWriterPly::~PointCloudWriterPly()
{
    close();
    delete mFile;
}

bool PointCloudWriterPly::setDataFormat(const DataFormatPly dataFormat)
{
    if(mFile->isOpen())
    {
        return false;
    }
    else
    {
        mDataFormat = dataFormat;
        return true;
    }

}

bool PointCloudWriterPly::open()
{
    if(!mFile->open(QIODevice::Text | QIODevice::ReadWrite | QIODevice::Truncate))
        return false;

    QTextStream stream(mFile);
    stream << QString("ply") << endl;

    if(mDataFormat == DataFormatAscii) stream << QString("format ascii 1.0") << endl;
    else if(mDataFormat == DataFormatBinaryLittleEndian) stream << QString("format binary_little_endian 1.0") << endl;
    else if(mDataFormat == DataFormatBinaryBigEndian) stream << QString("format binary_big_endian 1.0") << endl;

    stream << QString("comment written by koptertools / ben adler") << endl; // no timestamp, that prevents checksum comparisons
    stream << QString("element vertex 0000000000") << endl; // extra space for overwriting with bigger numbers later-on, meshlab doesn't mind.
    stream << QString("comment the coordinates of the point") << endl;
    stream << QString("property float x") << endl;
    stream << QString("property float y") << endl;
    stream << QString("property float z") << endl;
    stream << QString("end_header") << endl;

    stream.flush();

    return true;
}

bool PointCloudWriterPly::writePoints(float* source, qint64 numberOfPoints, QWidget* widget)
{
    QProgressDialog* progressDialog = nullptr;

    if(widget != nullptr)
    {
        progressDialog = new QProgressDialog("Processing cloud...", "Cancel", 0, 100, widget);
        progressDialog->setWindowModality(Qt::WindowModal);

        progressDialog->setValue(0);
        progressDialog->setMaximum(numberOfPoints);
        progressDialog->show();
    }

    if(mDataFormat == DataFormatAscii)
    {
        QTextStream stream(mFile);
        stream.setRealNumberPrecision(3);
        stream.setRealNumberNotation(QTextStream::FixedNotation);
        for(int i = 0; i < numberOfPoints; ++i)
        {
            stream << source[4*i+0] << " " << source[4*i+1] << " " << source[4*i+2] << endl;
            if(i%10000 == 0 && progressDialog != nullptr) progressDialog->setValue((i*100)/numberOfPoints);
        }
    }
    else
    {
        // write binary data
        mFile->seek(mFile->size());
        if(mDataFormat == DataFormatBinaryLittleEndian)
        {
            for(int i = 0; i < numberOfPoints; ++i)
            {
                mFile->write((char*)&source[4*i+0], sizeof(float));
                mFile->write((char*)&source[4*i+1], sizeof(float));
                mFile->write((char*)&source[4*i+2], sizeof(float));
                mTotalNumberOfPointsWrittenToThisFile++;
                if(i%10000 == 0 && progressDialog != nullptr) progressDialog->setValue((i*100)/numberOfPoints);
            }
        }
        else
        {
            for(int i = 0; i < numberOfPoints; ++i)
            {
                const float x = swap_endian(source[4*i+0]);
                const float y = swap_endian(source[4*i+1]);
                const float z = swap_endian(source[4*i+2]);
                mFile->write((char*)&x, sizeof(float));
                mFile->write((char*)&y, sizeof(float));
                mFile->write((char*)&z, sizeof(float));
                mTotalNumberOfPointsWrittenToThisFile++;
                if(i%10000 == 0 && progressDialog != nullptr) progressDialog->setValue((i*100)/numberOfPoints);
            }
        }
    }

    progressDialog->close();
    delete progressDialog;
}

void PointCloudWriterPly::close()
{
    // We need to write the number of points into the header!
    mFile->seek(0);
    QString line;
    do {
        line = mFile->readLine();
        qDebug() << "line" << line;
        if(line.startsWith("element vertex 0000000000"))
        {
            QString numberString = QString::number(mTotalNumberOfPointsWrittenToThisFile);
            while(numberString.size() < 10) numberString.append(" ");

            const QString lineNew = QString("element vertex %1").arg(numberString);
            mFile->seek(mFile->pos() - line.size());
            mFile->write(lineNew.toLatin1());
            break;
        }
    } while (!line.isNull() && !mFile->atEnd());

    mFile->close();
}
