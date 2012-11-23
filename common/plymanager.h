#ifndef PLYMANAGER_H
#define PLYMANAGER_H

#include <QFile>
#include <QVector>
#include <QVector3D>
#include <QVector4D>
#include <QString>
#include <QTextStream>
#include <QDateTime>
#include <QDebug>
#include <QMessageBox>
#include <QProgressDialog>

class PlyManager : public QObject
{
    Q_OBJECT
public:
    enum DataDirection {DataLoadFromFile, DataSaveToFile};
    enum IncludesNormals { NormalsIncluded, NormalsNotIncluded };
    enum IncludesDirection { DirectionIncluded, DirectionNotIncluded };

    PlyManager(QWidget* widget = 0);
    ~PlyManager();

    bool open(const QString& fileName, const PlyManager::DataDirection& direction);

    quint32 getNumberOfPoints() const;

    // This will write a header to the file. DataDirection must be DataSaveToFile
    void writeHeader(const quint32& vertexCount, const PlyManager::IncludesNormals& includesNormals = NormalsNotIncluded, const PlyManager::IncludesDirection& includesDirection = DirectionNotIncluded);

    // These methods will read @points and stream them into the file. DataDirection must be DataSaveToFile
    void savePly4D(const float *const points, const quint32 numPoints);

    // These methods will read the file and write to @points. DataDirection must be DataLoadFromFile
    void loadPly4D(float *points, const quint32 startPoint = 0, const quint32 maxPoints = 0);

private:
    DataDirection mDataDirection;

    QWidget* mWidget;
    QProgressDialog* mProgressDialog;

    // The file to be written to or read from
    QFile* mFile;

    // If reading, this is the number of points present in the file
    quint32 mNumberOfPoints;

};

#endif
