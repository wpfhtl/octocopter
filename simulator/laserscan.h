#ifndef LASERSCAN_H
#define LASERSCAN_H

#include <QObject>

class LaserScan// : public QObject
{
//Q_OBJECT
private:
    QList<> points;
    OgreWidget *mOgreWidget;
    QList<LaserScanner*> mLaserScanners;

    // Creates LaserScanners according to QSettings config file and adds the meshes to ogre scene.
    void createLaserScannersFromConfiguration(void);

public:
    LaserScannerManager(Simulator *simulator);
    LaserScanner* getLaserScanner(const int index);

signals:

public slots:

};

#endif
