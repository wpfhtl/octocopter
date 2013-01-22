#include <QApplication>
#include <QFileDialog>
#include <QVector4D>
#include <QString>
#include <QDebug>

#include <plymanager.h>

void compareClouds(QTextStream* const streamOv, const QString& directory, const QString& cloud1, const QString& cloud2, float* points1, float* points2, const quint32 numPoints)
{
    // This map maps the error-size to the error-count, eg:
    // 3 -> 4000 means 4000 points were 3-4mm off

    QFile dataFileHist(directory + "/" + "graph_cloud_diff_hist_"+cloud1+"_vs_"+cloud2+".dat");
    QFile dataFileAll(directory + "/" + "graph_cloud_diff_all_"+cloud1+"_vs_"+cloud2+".dat");

    if(!dataFileHist.open(QFile::WriteOnly | QFile::Truncate))
        qFatal("couldn't open file %s.", qPrintable(dataFileHist.fileName()));

    if(!dataFileAll.open(QFile::WriteOnly | QFile::Truncate))
        qFatal("couldn't open file %s.", qPrintable(dataFileAll.fileName()));

    QTextStream streamHist(&dataFileHist);
    QTextStream streamAll(&dataFileAll);

    QMap<quint16, quint32> errorHistogram;

    float distanceMax = 0.0f;
    float distanceTotal = 0.0f;

    for(int i=0;i<numPoints;i++)
    {
        QVector3D p1(
                    points1[i*4+0],
                    points1[i*4+1],
                    points1[i*4+2]);

        QVector3D p2(
                    points2[i*4+0],
                    points2[i*4+1],
                    points2[i*4+2]);

        const float distance = p1.distanceToLine(p2, QVector3D());

        const quint16 distanceInMm = floor(distance * 1000.0);
        errorHistogram[distanceInMm+1]++;

        streamAll << i << "\t" << distance << endl;

        distanceTotal += distance;
        distanceMax = std::max(distance, distanceMax);
    }

    (*streamOv) << "\"" << cloud1 << " vs " << cloud2 << "\"\t" << distanceTotal / numPoints << "\t" << distanceMax << "\n";

    streamHist << "# comparing " << cloud1 << " with " << cloud2 << ": average dist " << distanceTotal / numPoints << ", max dist " << distanceMax << ", error-histogram is:\n";
    QMapIterator<quint16, quint32> i(errorHistogram);
    while(i.hasNext())
    {
        i.next();
        streamHist << i.key() << "\t" << i.value() << endl;
    }
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QString cloudNameRef = QFileDialog::getOpenFileName(0, "Select reference cloud", QString(), QString("Pointclouds (*.ply)"), 0, 0);
    if(cloudNameRef.isEmpty()) return 1;

    // Loading of our pointclouds
    PlyManager cloudRef, cloudGnssLn, cloudGnssNn, cloudLaserCb, cloudLaserLn, cloudLaserNn;

    cloudRef.open(cloudNameRef, PlyManager::DataLoadFromFile);

    cloudGnssLn.open(QString(cloudNameRef).replace("gnsscb", "gnssln"), PlyManager::DataLoadFromFile);
    cloudGnssNn.open(QString(cloudNameRef).replace("gnsscb", "gnssnn"), PlyManager::DataLoadFromFile);
    cloudLaserCb.open(QString(cloudNameRef).replace("gnsscb", "lasercb"), PlyManager::DataLoadFromFile);
    cloudLaserLn.open(QString(cloudNameRef).replace("gnsscb", "laserln"), PlyManager::DataLoadFromFile);
    cloudLaserNn.open(QString(cloudNameRef).replace("gnsscb", "lasernn"), PlyManager::DataLoadFromFile);

    const quint32 pointsToCompare = cloudRef.getNumberOfPoints();

    // allocate memory
    float* cloudDataRef = (float*)malloc(pointsToCompare * 4 * sizeof(float));
    float* cloudDataGnssLn = (float*)malloc(pointsToCompare * 4 * sizeof(float));
    float* cloudDataGnssNn = (float*)malloc(pointsToCompare * 4 * sizeof(float));

    float* cloudDataLaserCb = (float*)malloc(pointsToCompare * 4 * sizeof(float));
    float* cloudDataLaserLn = (float*)malloc(pointsToCompare * 4 * sizeof(float));
    float* cloudDataLaserNn = (float*)malloc(pointsToCompare * 4 * sizeof(float));

    // read clouds
    cloudRef.loadPly4D(cloudDataRef, 0, pointsToCompare); qDebug() << "loading complete.";
    cloudGnssLn.loadPly4D(cloudDataGnssLn, 0, pointsToCompare); qDebug() << "loading complete.";
    cloudGnssNn.loadPly4D(cloudDataGnssNn, 0, pointsToCompare); qDebug() << "loading complete.";
    cloudLaserCb.loadPly4D(cloudDataLaserCb, 0, pointsToCompare); qDebug() << "loading complete.";
    cloudLaserLn.loadPly4D(cloudDataLaserLn, 0, pointsToCompare); qDebug() << "loading complete.";
    cloudLaserNn.loadPly4D(cloudDataLaserNn, 0, pointsToCompare); qDebug() << "loading complete.";

    QString dir = QFileInfo(cloudNameRef).canonicalPath();

    QFile dataFileOverview(dir + "/" + "graph_cloud_diff_overview.dat");

    if(!dataFileOverview.open(QFile::WriteOnly | QFile::Truncate))
        qFatal("couldn't open file %s.", qPrintable(dataFileOverview.fileName()));

    QTextStream streamOverview(&dataFileOverview);
    streamOverview << "Clouds\tAverage distance (m)\tMaximum distance (m)\n";

    compareClouds(&streamOverview, dir, "gnsscb", "gnsscb", cloudDataRef, cloudDataRef, pointsToCompare);
    compareClouds(&streamOverview, dir, "gnsscb", "gnssln", cloudDataRef, cloudDataGnssLn, pointsToCompare);
    compareClouds(&streamOverview, dir, "gnsscb", "gnssnn", cloudDataRef, cloudDataGnssNn, pointsToCompare);
    compareClouds(&streamOverview, dir, "gnsscb", "lasercb", cloudDataRef, cloudDataLaserCb, pointsToCompare);
    compareClouds(&streamOverview, dir, "gnsscb", "laserln", cloudDataRef, cloudDataLaserLn, pointsToCompare);
    compareClouds(&streamOverview, dir, "gnsscb", "lasernn", cloudDataRef, cloudDataLaserNn, pointsToCompare);

    return 0;// app.exec();
}
