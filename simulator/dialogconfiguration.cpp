#include "dialogconfiguration.h"

DialogConfiguration::DialogConfiguration(Simulator *simulator) :
        QDialog((QWidget*)simulator),
        mSettings("BenAdler", "simulator")
{
    qDebug() << "DialogConfiguration::DialogConfiguration()";
    setupUi(this);

    mSimulator = simulator;
    mOgreWidget = mSimulator->mOgreWidget;

    connect(mTableWidgetLaserScanners, SIGNAL(itemChanged(QTableWidgetItem*)), SLOT(slotLaserScannerDetailChanged(QTableWidgetItem*)));
    connect(mBtnLaserScannerAdd, SIGNAL(clicked()), SLOT(slotLaserScannerAdd()));
    connect(mBtnLaserScannerDel, SIGNAL(clicked()), SLOT(slotLaserScannerDel()));

    connect(mTableWidgetCameras, SIGNAL(itemChanged(QTableWidgetItem*)), SLOT(slotCameraDetailChanged(QTableWidgetItem*)));
    connect(mBtnCameraAdd, SIGNAL(clicked()), SLOT(slotCameraAdd()));
    connect(mBtnCameraDel, SIGNAL(clicked()), SLOT(slotCameraDel()));

    connect(mChkBoxWindEnable, SIGNAL(stateChanged(int)), SLOT(slotWindDetailChanged()));
    connect(mSpinBoxWindFactor, SIGNAL(valueChanged(double)), SLOT(slotWindDetailChanged()));

    connect(mSpinBoxBatteryVoltage, SIGNAL(valueChanged(double)), mSimulator->mBattery, SLOT(slotSetVoltageMax(double)));
    connect(mSpinBoxBatteryCapacity, SIGNAL(valueChanged(double)), mSimulator->mBattery, SLOT(slotSetCapacity(double)));

//    QStringList labelsCams;
//    labelsCams << "X" << "Y" << "Z" << "Pitch" << "Roll" << "Yaw" << "Interval" << "Width" << "Height";
    mTableWidgetLaserScanners->horizontalHeader()->setVisible(true);
    mTableWidgetCameras->horizontalHeader()->setVisible(true);
    mTableWidgetMotors->horizontalHeader()->setVisible(true);
//    mTableWidgetCameras->setHorizontalHeaderLabels(labelsCams);


    connect(mBtnOk, SIGNAL(clicked()), SLOT(slotOkPressed()));

//    qRegisterMetaType<QVector<QVector3D>* >("QVector<QVector3D>*");

    qDebug() << "DialogConfiguration::DialogConfiguration(): done";
}


DialogConfiguration::~DialogConfiguration()
{
    qDebug() << "DialogConfiguration::~DialogConfiguration(): cams and lidars:" << mSimulator->mCameras->size() << mSimulator->mLaserScanners->size();
    slotSaveConfiguration();
    qDebug() << "DialogConfiguration::~DialogConfiguration(): done";
}

void DialogConfiguration::slotWindDetailChanged()
{
    emit windDetailChanged(mChkBoxWindEnable->isChecked(), mSpinBoxWindFactor->value());
}


void DialogConfiguration::slotOkPressed()
{
    qDebug() << "DialogConfiguration::slotOkPressed(): saving config";
    slotSaveConfiguration();
    hide();
}

void DialogConfiguration::slotReadConfiguration()
{
    slotReadConfigurationLaserScanner();
    slotReadConfigurationCamera();
    slotReadConfigurationMisc();
}

void DialogConfiguration::slotSaveConfiguration()
{
    slotSaveConfigurationLaserScanner();
    slotSaveConfigurationCamera();
    slotSaveConfigurationMisc();
    mSettings.sync();
}

void DialogConfiguration::slotReadConfigurationMisc()
{
    mSettings.beginGroup("misc");

    mChkBoxWindEnable->setChecked(mSettings.value("windEnable").toBool());
    mSpinBoxWindFactor->setValue(mSettings.value("windFactor").toFloat());

    mSpinBoxBatteryVoltage->setValue(mSettings.value("batteryVoltage", "14.8").toFloat());
    mSpinBoxBatteryCapacity->setValue(mSettings.value("batteryCapacity", "5").toFloat());

    mSettings.endGroup();
}

void DialogConfiguration::slotSaveConfigurationMisc()
{
    mSettings.beginGroup("misc");

    mSettings.setValue("windEnable", mChkBoxWindEnable->isChecked());
    mSettings.setValue("windFactor", mSpinBoxWindFactor->value());

    mSettings.setValue("batteryVoltage", mSpinBoxBatteryVoltage->value());
    mSettings.setValue("batteryCapacity", mSpinBoxBatteryCapacity->value());

    mSettings.endGroup();
}

void DialogConfiguration::slotReadConfigurationLaserScanner()
{
    QList<LaserScanner*> *laserScanners = mSimulator->getLaserScannerList();

    // delete all old laserscanners
    while(laserScanners->size())
    {
        LaserScanner* scanner = laserScanners->takeFirst();
        scanner->quit();
        scanner->wait();
        delete scanner;
    }

    while(mTableWidgetLaserScanners->rowCount()) mTableWidgetLaserScanners->removeRow(0);

    const int numberOfLaserScanners = mSettings.beginReadArray("scanners");
    qDebug() << "DialogConfiguration::slotReadConfigurationLaserScanner(): number of scanners found" << numberOfLaserScanners;

    for(int i = 0; i < numberOfLaserScanners; ++i)
    {
        mSettings.setArrayIndex(i);
        mTableWidgetLaserScanners->insertRow(i);

        qDebug() << "DialogConfiguration::slotReadConfigurationLaserScanner(): creating scanner" << i << "from configuration";

        // By default, laserscanners are slooow
        LaserScanner* newLaserScanner = new LaserScanner(
                mSimulator,
                mOgreWidget,
                mSettings.value("range", 20.0).toReal(),
                mSettings.value("speed", 100).toInt(),
                mSettings.value("angleStart", 45).toInt(),
                mSettings.value("angleStop", 315).toInt(),
                mSettings.value("angleStep", 0.25).toReal());

        newLaserScanner->moveToThread(newLaserScanner);

        // set scanner position from configuration
        newLaserScanner->setPosition(
                Ogre::Vector3(
                        mSettings.value("posX", 0.0).toReal(),
                        mSettings.value("posY", 0.0).toReal(),
                        mSettings.value("posZ", 0.0).toReal()
                        )
                );

        // set scanner orientation from form values
        Ogre::Quaternion yaw(Ogre::Degree(mSettings.value("rotYaw", 0.0).toReal()), Ogre::Vector3::UNIT_Y);
        Ogre::Quaternion pitch(Ogre::Degree(mSettings.value("rotPitch", 0.0).toReal()), Ogre::Vector3::UNIT_X);
        Ogre::Quaternion roll(Ogre::Degree(mSettings.value("rotRoll", 0.0).toReal()), Ogre::Vector3::UNIT_Z);
        newLaserScanner->setOrientation(yaw * pitch * roll);

        qDebug() << "DialogConfiguration::slotReadConfigurationLaserScanner(): scanner" << i << "orientation YPR:"
                 << mSettings.value("rotYaw", 0.0).toReal()
                 << mSettings.value("rotPitch", 0.0).toReal()
                 << mSettings.value("rotRoll", 0.0).toReal();

        newLaserScanner->start();

        laserScanners->append(newLaserScanner);

        connect(newLaserScanner, SIGNAL(newLidarPoints(QVector<QVector4D>,QVector3D)), mSimulator->mBaseConnection, SLOT(slotNewScanFused(QVector<QVector4D>,QVector3D)));

        // only do this for scanners pointing down!
        if(fabs(mSettings.value("rotPitch", 0.0).toReal()) > 85.0f && fabs(mSettings.value("rotPitch", 0.0).toReal()) < 95.0f)
            connect(newLaserScanner, &LaserScanner::heightOverGround, mSimulator->mFlightController, &FlightController::slotSetHeightOverGround);

        // Now create a row in the LaserScannerTable
        mTableWidgetLaserScanners->blockSignals(true);
        mTableWidgetLaserScanners->setItem(i, 0, new QTableWidgetItem(mSettings.value("posX", 0.0).toString()));
        mTableWidgetLaserScanners->setItem(i, 1, new QTableWidgetItem(mSettings.value("posY", 0.0).toString()));
        mTableWidgetLaserScanners->setItem(i, 2, new QTableWidgetItem(mSettings.value("posZ", 0.0).toString()));

        mTableWidgetLaserScanners->setItem(i, 3, new QTableWidgetItem(mSettings.value("rotYaw", 0.0).toString()));
        mTableWidgetLaserScanners->setItem(i, 4, new QTableWidgetItem(mSettings.value("rotPitch", 0.0).toString()));
        mTableWidgetLaserScanners->setItem(i, 5, new QTableWidgetItem(mSettings.value("rotRoll", 0.0).toString()));

        mTableWidgetLaserScanners->setItem(i, 6, new QTableWidgetItem(mSettings.value("range", 20.0).toString()));
        mTableWidgetLaserScanners->setItem(i, 7, new QTableWidgetItem(mSettings.value("speed", 100).toString()));
        mTableWidgetLaserScanners->setItem(i, 8, new QTableWidgetItem(mSettings.value("angleStart", 45).toString()));
        mTableWidgetLaserScanners->setItem(i, 9, new QTableWidgetItem(mSettings.value("angleStop", 315).toString()));
        mTableWidgetLaserScanners->setItem(i, 10,new QTableWidgetItem(mSettings.value("angleStep", 0.25).toString()));
        mTableWidgetLaserScanners->blockSignals(false);
    }
    mSettings.endArray();

    mOgreWidget->update();
}

void DialogConfiguration::slotCameraAdd()
{
    const int row = mTableWidgetCameras->rowCount();
    mTableWidgetCameras->insertRow(row);

    Camera* newCamera = new Camera(mSimulator, mOgreWidget, QSize(800, 600), 45.0, 2000);
//    newCamera->moveToThread(newCamera);

    mSimulator->mCameras->append(newCamera);

    // Now create a row in the LaserScannerTable
    mTableWidgetCameras->blockSignals(true);
    mTableWidgetCameras->setItem(row, 0, new QTableWidgetItem("0.0"));
    mTableWidgetCameras->setItem(row, 1, new QTableWidgetItem("0.0"));
    mTableWidgetCameras->setItem(row, 2, new QTableWidgetItem("0.0"));

    mTableWidgetCameras->setItem(row, 3, new QTableWidgetItem("0.0"));
    mTableWidgetCameras->setItem(row, 4, new QTableWidgetItem("0.0"));
    mTableWidgetCameras->setItem(row, 5, new QTableWidgetItem("0.0"));

    mTableWidgetCameras->setItem(row, 6, new QTableWidgetItem("800"));
    mTableWidgetCameras->setItem(row, 7, new QTableWidgetItem("600"));
    mTableWidgetCameras->setItem(row, 8, new QTableWidgetItem("45"));
    mTableWidgetCameras->setItem(row, 9, new QTableWidgetItem("2000"));
    mTableWidgetCameras->blockSignals(false);

//    newCamera->start();
    mOgreWidget->update();
}

void DialogConfiguration::slotReadConfigurationCamera()
{
    QList<Camera*> *cameras = mSimulator->mCameras;

    // delete all old cameras
    while(cameras->size())
    {
        Camera* camera = cameras->takeFirst();
//        camera->quit();
//        camera->wait();
//        delete camera;
        camera->deleteLater();
    }

    while(mTableWidgetCameras->rowCount()) mTableWidgetCameras->removeRow(0);

    const int numberOfCameras = mSettings.beginReadArray("cameras");

    for(int i = 0; i < numberOfCameras; ++i)
    {
        mSettings.setArrayIndex(i);
        mTableWidgetCameras->insertRow(i);

        qDebug() << "DialogConfiguration::slotReadConfigurationCamera(): creating camera" << i << "from configuration";

        Camera* newCamera = new Camera(mSimulator, mOgreWidget, QSize(mSettings.value("width", 800).toInt(), mSettings.value("height", 600).toInt()), mSettings.value("fovy", 45).toFloat(), mSettings.value("interval", 2000).toInt());

//        newCamera->moveToThread(newCamera);

        // set scanner position from configuration
        newCamera->setPosition(
                Ogre::Vector3(
                        mSettings.value("posX", 0.0).toReal(),
                        mSettings.value("posY", 0.0).toReal(),
                        mSettings.value("posZ", 0.0).toReal()
                        )
                );

        // set scanner orientation from form values
        Ogre::Quaternion yaw(Ogre::Degree(mSettings.value("rotYaw", 0.0).toReal()), Ogre::Vector3::UNIT_Y);
        Ogre::Quaternion pitch(Ogre::Degree(mSettings.value("rotPitch", 0.0).toReal()), Ogre::Vector3::UNIT_X);
        Ogre::Quaternion roll(Ogre::Degree(mSettings.value("rotRoll", 0.0).toReal()), Ogre::Vector3::UNIT_Z);
        newCamera->setOrientation(yaw * pitch * roll);

        cameras->append(newCamera);

        // Now create a row in the LaserScannerTable
        mTableWidgetCameras->blockSignals(true);
        mTableWidgetCameras->setItem(i, 0, new QTableWidgetItem(mSettings.value("posX", 0.0).toString()));
        mTableWidgetCameras->setItem(i, 1, new QTableWidgetItem(mSettings.value("posY", 0.0).toString()));
        mTableWidgetCameras->setItem(i, 2, new QTableWidgetItem(mSettings.value("posZ", 0.0).toString()));

        mTableWidgetCameras->setItem(i, 3, new QTableWidgetItem(mSettings.value("rotYaw", 0.0).toString()));
        mTableWidgetCameras->setItem(i, 4, new QTableWidgetItem(mSettings.value("rotPitch", 0.0).toString()));
        mTableWidgetCameras->setItem(i, 5, new QTableWidgetItem(mSettings.value("rotRoll", 0.0).toString()));

        mTableWidgetCameras->setItem(i, 6, new QTableWidgetItem(mSettings.value("width", 800).toString()));
        mTableWidgetCameras->setItem(i, 7, new QTableWidgetItem(mSettings.value("height", 600).toString()));
        mTableWidgetCameras->setItem(i, 8, new QTableWidgetItem(mSettings.value("fovy", 45).toString()));
        mTableWidgetCameras->setItem(i, 9, new QTableWidgetItem(mSettings.value("interval", 2000).toString()));
        mTableWidgetCameras->blockSignals(false);
    }
    mSettings.endArray();

    mOgreWidget->update();
}

void DialogConfiguration::slotSaveConfigurationCamera()
{
    // Save the configuration
    qDebug() << "DialogConfiguration::slotSaveConfigurationCamera()";

    QList<Camera*> *cameras = mSimulator->mCameras;

    // Delete old scanners-section, as that might still contain more scanners than what we're about to write
    mSettings.remove("cameras");

    mSettings.beginWriteArray("cameras");

    for(int i = 0; i < cameras->size(); ++i)
    {
        // write settings for this camera into config file
        qDebug() << "DialogConfiguration::slotSaveConfigurationCamera(): now saving camera-config" << i;
        mSettings.setArrayIndex(i);

        mSettings.setValue("posX", cameras->at(i)->getPosition().x);
        mSettings.setValue("posY", cameras->at(i)->getPosition().y);
        mSettings.setValue("posZ", cameras->at(i)->getPosition().z);

        mSettings.setValue("rotYaw", cameras->at(i)->getOrientation().getYaw(false).valueDegrees());
        mSettings.setValue("rotPitch", cameras->at(i)->getOrientation().getPitch(false).valueDegrees());
        mSettings.setValue("rotRoll", cameras->at(i)->getOrientation().getRoll(false).valueDegrees());

        mSettings.setValue("width", cameras->at(i)->imageSize().width());
        mSettings.setValue("height", cameras->at(i)->imageSize().height());
        mSettings.setValue("fovy", cameras->at(i)->fovY());
        mSettings.setValue("interval", cameras->at(i)->interval());
    }

    mSettings.endArray();

    mSettings.sync();

    qDebug() << "DialogConfiguration::slotSaveConfigurationCamera(): done";
}

void DialogConfiguration::slotLaserScannerDetailChanged(QTableWidgetItem* item)
{
    // The user changed a detail. Propagate this change to the corresponding laserscanner.
    qDebug() << "DialogConfiguration::slotLaserScannerDetailChanged()";
    LaserScanner* scanner = mSimulator->getLaserScannerList()->at(item->row());

    scanner->setPosition(Ogre::Vector3(
            mTableWidgetLaserScanners->item(item->row(), 0)->text().toFloat(),
            mTableWidgetLaserScanners->item(item->row(), 1)->text().toFloat(),
            mTableWidgetLaserScanners->item(item->row(), 2)->text().toFloat()
            ));

    Ogre::Quaternion yaw(Ogre::Degree(mTableWidgetLaserScanners->item(item->row(), 3)->text().toFloat()), Ogre::Vector3::UNIT_Y);
    Ogre::Quaternion pitch(Ogre::Degree(mTableWidgetLaserScanners->item(item->row(), 4)->text().toFloat()), Ogre::Vector3::UNIT_X);
    Ogre::Quaternion roll(Ogre::Degree(mTableWidgetLaserScanners->item(item->row(), 5)->text().toFloat()), Ogre::Vector3::UNIT_Z);
    scanner->setOrientation(yaw * pitch * roll);

    scanner->setRange(mTableWidgetLaserScanners->item(item->row(), 6)->text().toFloat());
    scanner->setSpeed(mTableWidgetLaserScanners->item(item->row(), 7)->text().toInt());
    scanner->setAngleStart(mTableWidgetLaserScanners->item(item->row(), 8)->text().toInt());
    scanner->setAngleStop(mTableWidgetLaserScanners->item(item->row(), 9)->text().toInt());
    scanner->setAngleStep(mTableWidgetLaserScanners->item(item->row(), 10)->text().toFloat());

    mOgreWidget->update();
}

void DialogConfiguration::slotCameraDetailChanged(QTableWidgetItem* item)
{
    // The user changed a detail. Propagate this change to the corresponding camera.
    qDebug() << "DialogConfiguration::slotLaserScannerDetailChanged()";
    Camera* camera = mSimulator->mCameras->at(item->row());

    camera->setPosition(Ogre::Vector3(
            mTableWidgetCameras->item(item->row(), 0)->text().toFloat(),
            mTableWidgetCameras->item(item->row(), 1)->text().toFloat(),
            mTableWidgetCameras->item(item->row(), 2)->text().toFloat()
            ));

    Ogre::Quaternion pitch(Ogre::Degree(mTableWidgetCameras->item(item->row(), 3)->text().toFloat()), Ogre::Vector3::UNIT_X);
    Ogre::Quaternion roll(Ogre::Degree(mTableWidgetCameras->item(item->row(), 4)->text().toFloat()), Ogre::Vector3::UNIT_Z);
    Ogre::Quaternion yaw(Ogre::Degree(mTableWidgetCameras->item(item->row(), 5)->text().toFloat()), Ogre::Vector3::UNIT_Y);
    camera->setOrientation(yaw * pitch * roll);

    camera->setImageSize(QSize(mTableWidgetCameras->item(item->row(), 6)->text().toInt(), mTableWidgetCameras->item(item->row(), 7)->text().toInt()));
    camera->setFovY(mTableWidgetCameras->item(item->row(), 8)->text().toFloat());
    camera->setInterval(mTableWidgetCameras->item(item->row(), 9)->text().toInt());

    mOgreWidget->update();
}

void DialogConfiguration::slotSaveConfigurationLaserScanner()
{
    // Save the configuration
    qDebug() << "DialogConfiguration::slotSaveConfigurationLaserScanner()";

    QList<LaserScanner*> *scanners = mSimulator->getLaserScannerList();

    // Delete old scanners-section, as that might still contain more scanners than what we're about to write
    mSettings.remove("scanners");

    mSettings.beginWriteArray("scanners");

    for(int i= 0; i < mTableWidgetLaserScanners->rowCount(); i++)
    {
        mSettings.setArrayIndex(i);

        mSettings.setValue("posX", mTableWidgetLaserScanners->item(i, 0)->text().toFloat());
        mSettings.setValue("posY", mTableWidgetLaserScanners->item(i, 1)->text().toFloat());
        mSettings.setValue("posZ", mTableWidgetLaserScanners->item(i, 2)->text().toFloat());

        mSettings.setValue("rotYaw", mTableWidgetLaserScanners->item(i, 3)->text().toFloat());
        mSettings.setValue("rotPitch", mTableWidgetLaserScanners->item(i, 4)->text().toFloat());
        mSettings.setValue("rotRoll", mTableWidgetLaserScanners->item(i, 5)->text().toFloat());

        mSettings.setValue("range", mTableWidgetLaserScanners->item(i, 6)->text().toFloat());
        mSettings.setValue("speed", mTableWidgetLaserScanners->item(i, 7)->text().toFloat());
        mSettings.setValue("angleStart", mTableWidgetLaserScanners->item(i, 8)->text().toFloat());
        mSettings.setValue("angleStop", mTableWidgetLaserScanners->item(i, 9)->text().toFloat());
        mSettings.setValue("angleStep", mTableWidgetLaserScanners->item(i, 10)->text().toFloat());
    }

    mSettings.endArray();

    mSettings.sync();

    qDebug() << "DialogConfiguration::slotSaveConfigurationLaserScanner(): done";
}


void DialogConfiguration::slotLaserScannerAdd()
{
    const int row = mTableWidgetLaserScanners->rowCount();
    mTableWidgetLaserScanners->insertRow(row);

    LaserScanner* newLaserScanner = new LaserScanner(mSimulator, mOgreWidget, 20.0, 100, 45, 315, 0.25);
    newLaserScanner->moveToThread(newLaserScanner);

    mSimulator->getLaserScannerList()->append(newLaserScanner);

    // Now create a row in the LaserScannerTable
    mTableWidgetLaserScanners->blockSignals(true);
    mTableWidgetLaserScanners->setItem(row, 0, new QTableWidgetItem("0.0"));
    mTableWidgetLaserScanners->setItem(row, 1, new QTableWidgetItem("0.0"));
    mTableWidgetLaserScanners->setItem(row, 2, new QTableWidgetItem("0.0"));

    mTableWidgetLaserScanners->setItem(row, 3, new QTableWidgetItem("0.0"));
    mTableWidgetLaserScanners->setItem(row, 4, new QTableWidgetItem("0.0"));
    mTableWidgetLaserScanners->setItem(row, 5, new QTableWidgetItem("0.0"));

    mTableWidgetLaserScanners->setItem(row, 6, new QTableWidgetItem("20.0"));
    mTableWidgetLaserScanners->setItem(row, 7, new QTableWidgetItem("100"));
    mTableWidgetLaserScanners->setItem(row, 8, new QTableWidgetItem("45"));
    mTableWidgetLaserScanners->setItem(row, 9, new QTableWidgetItem("315"));
    mTableWidgetLaserScanners->setItem(row, 10,new QTableWidgetItem("0.25"));
    mTableWidgetLaserScanners->blockSignals(false);

    connect(newLaserScanner, SIGNAL(newLidarPoints(QVector<QVector4D>,QVector3D)), mSimulator->mBaseConnection, SLOT(slotNewScanFused(QVector<QVector4D>,QVector3D)));
    //connect(newLaserScanner, &LaserScanner::heightOverGround, mSimulator->mFlightController, &FlightController::slotSetHeightOverGround);

    newLaserScanner->start();
    mOgreWidget->update();
}

void DialogConfiguration::slotLaserScannerDel()
{
    const int row = mTableWidgetLaserScanners->currentRow() == -1 ? mTableWidgetLaserScanners->rowCount()-1 : mTableWidgetLaserScanners->currentRow();
    mTableWidgetLaserScanners->removeRow(row);

    LaserScanner* scanner = mSimulator->getLaserScannerList()->takeAt(row);
    scanner->quit();
    scanner->wait();
    scanner->deleteLater();

    mOgreWidget->update();
}


void DialogConfiguration::slotCameraDel()
{
    const int row = mTableWidgetCameras->currentRow() == -1 ? mTableWidgetCameras->rowCount()-1 : mTableWidgetCameras->currentRow();
    mTableWidgetCameras->removeRow(row);

    Camera* camera = mSimulator->mCameras->takeAt(row);
//    camera->quit();
//    camera->wait();
//    delete camera;
    camera->deleteLater();

    mOgreWidget->update();
}
