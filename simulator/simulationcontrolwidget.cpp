#include "simulationcontrolwidget.h"

SimulationControlWidget::SimulationControlWidget(Simulator *simulator, OgreWidget *ogreWidget) :
        QDockWidget((QWidget*)simulator),
        //ui(new Ui::SimulationControlWidget),
        mSettings("BenAdler", "simulator")
{
    qDebug() << "SimulationControlWidget::SimulationControlWidget()";
    /*ui->*/setupUi(this);

    mSimulator = simulator;
    mOgreWidget = ogreWidget;

    // Wire up the time-box
    connect(mSpinBoxTimeFactor, SIGNAL(valueChanged(double)), SIGNAL(timeFactorChanged(double)));
    connect(mBtnStart, SIGNAL(clicked()), SLOT(slotSimulationStarted()));
    connect(mBtnPause, SIGNAL(clicked()), SLOT(slotSimulationPaused()));

    // Wire up the laserscanner-box
    connect(mSpinBoxScannerIndex, SIGNAL(valueChanged(int)), SLOT(slotScannerIndexChanged()));

    connect(mSpinBoxPropertiesRange, SIGNAL(valueChanged(double)), SLOT(slotScannerSettingsChanged()));
    connect(mSpinBoxPropertiesSpeed, SIGNAL(valueChanged(int)), SLOT(slotScannerSettingsChanged()));
    connect(mSpinBoxPropertiesAngleStart, SIGNAL(valueChanged(int)), SLOT(slotScannerSettingsChanged()));
    connect(mSpinBoxPropertiesAngleStop, SIGNAL(valueChanged(int)), SLOT(slotScannerSettingsChanged()));
    connect(mSpinBoxPropertiesAngleStep, SIGNAL(valueChanged(double)), SLOT(slotScannerSettingsChanged()));

    connect(mSpinBoxScannerPositionX, SIGNAL(valueChanged(double)), SLOT(slotScannerSettingsChanged()));
    connect(mSpinBoxScannerPositionY, SIGNAL(valueChanged(double)), SLOT(slotScannerSettingsChanged()));
    connect(mSpinBoxScannerPositionZ, SIGNAL(valueChanged(double)), SLOT(slotScannerSettingsChanged()));

    connect(mSpinBoxScannerRotationPitch, SIGNAL(valueChanged(double)), SLOT(slotScannerSettingsChanged()));
    connect(mSpinBoxScannerRotationRoll, SIGNAL(valueChanged(double)), SLOT(slotScannerSettingsChanged()));
    connect(mSpinBoxScannerRotationYaw, SIGNAL(valueChanged(double)), SLOT(slotScannerSettingsChanged()));

    connect(mBtnLaserScannerCreate, SIGNAL(clicked()), SLOT(slotScannerCreate()));
    connect(mBtnLaserScannerDelete, SIGNAL(clicked()), SLOT(slotScannerDelete()));
    connect(mBtnLaserScannerConfigurationLoad, SIGNAL(clicked()), SLOT(slotScannerLoadConfiguration()));
    connect(mBtnLaserScannerConfigurationSave, SIGNAL(clicked()), SLOT(slotScannerSaveConfiguration()));

    qDebug() << "SimulationControlWidget::SimulationControlWidget(): done";
}

void SimulationControlWidget::slotSimulationStarted()
{
    mBtnPause->setEnabled(true);
    mBtnStart->setEnabled(false);

    emit simulationStart();
}

void SimulationControlWidget::slotSimulationPaused()
{
    mBtnPause->setEnabled(false);
    mBtnStart->setEnabled(true);

    emit simulationPause();
}

void SimulationControlWidget::slotScannerIndexChanged(void)
{
    qDebug() << "SimulationControlWidget::slotScannerIndexChanged()";

    // Load that scanner's data into the widget. If the scanner exists, that is.
    if(mSimulator->getLaserScannerList()->size() >= mSpinBoxScannerIndex->value()) return;

    qDebug() << "SimulationControlWidget::slotScannerIndexChanged(): scanner" << mSpinBoxScannerIndex->value() << "exists, loading its settigns into form";

    // Get Scanner and its SceneNode
    LaserScanner *currentScanner = mSimulator->getLaserScannerList()->at(mSpinBoxScannerIndex->value());

    mSpinBoxPropertiesRange->setValue(currentScanner->range());
    mSpinBoxPropertiesSpeed->setValue(currentScanner->speed());
    mSpinBoxPropertiesAngleStart->setValue(currentScanner->angleStart());
    mSpinBoxPropertiesAngleStop->setValue(currentScanner->angleStop());
    mSpinBoxPropertiesAngleStep->setValue(currentScanner->angleStep());

    mSpinBoxScannerPositionX->setValue(currentScanner->getPosition().x);
    mSpinBoxScannerPositionY->setValue(currentScanner->getPosition().y);
    mSpinBoxScannerPositionZ->setValue(currentScanner->getPosition().z);

    mSpinBoxScannerRotationPitch->setValue(currentScanner->getOrientation().getPitch().valueDegrees());
    mSpinBoxScannerRotationRoll->setValue(currentScanner->getOrientation().getRoll().valueDegrees());
    mSpinBoxScannerRotationYaw->setValue(currentScanner->getOrientation().getYaw().valueDegrees());

    qDebug() << "SimulationControlWidget::slotScannerIndexChanged(): done.";
}

void SimulationControlWidget::slotScannerSettingsChanged(void)
{
    // The currently selected scanner has had its properties changed
    // in the form. Propagate the new values to the LaserScanner*

    qDebug() << "SimulationControlWidget::slotScannerSettingsChanged(): form changes detected, sending form values to currently selected scanner.";

    // Get Scanner and its SceneNode
    LaserScanner *currentScanner = mSimulator->getLaserScannerList()->at(mSpinBoxScannerIndex->value());

    // set scanner properties from form values
    currentScanner->setRange(mSpinBoxPropertiesRange->value());
    currentScanner->setSpeed(mSpinBoxPropertiesSpeed->value());
    currentScanner->setAngleStart(mSpinBoxPropertiesAngleStart->value());
    currentScanner->setAngleStop(mSpinBoxPropertiesAngleStop->value());
    currentScanner->setAngleStep(mSpinBoxPropertiesAngleStep->value());

    // set scanner position from form values
    currentScanner->setPosition(
            Ogre::Vector3(
                    mSpinBoxScannerPositionX->value(),
                    mSpinBoxScannerPositionY->value(),
                    mSpinBoxScannerPositionZ->value()
                    )
            );

    // set scanner orientation from form values
    Ogre::Quaternion pitch(Ogre::Degree(mSpinBoxScannerRotationPitch->value()), Ogre::Vector3::UNIT_X);
    Ogre::Quaternion roll(Ogre::Degree(mSpinBoxScannerRotationRoll->value()), Ogre::Vector3::UNIT_Z);
    Ogre::Quaternion yaw(Ogre::Degree(mSpinBoxScannerRotationYaw->value()), Ogre::Vector3::UNIT_Y);
    currentScanner->setOrientation(pitch * roll * yaw);

    qDebug() << "SimulationControlWidget::slotScannerSettingsChanged(): done";
}

void SimulationControlWidget::slotScannerCreate(void)
{
    qDebug() << "SimulationControlWidget::slotScannerCreate(): creating new scanner";

    LaserScanner* newLaserScanner = new LaserScanner(
            mSimulator,
            mOgreWidget,
            mSpinBoxPropertiesRange->value(),
            mSpinBoxPropertiesSpeed->value(),
            mSpinBoxPropertiesAngleStart->value(),
            mSpinBoxPropertiesAngleStop->value(),
            mSpinBoxPropertiesAngleStep->value());

    // set scanner position from form values
    newLaserScanner->setPosition(
            Ogre::Vector3(
                    mSpinBoxScannerPositionX->value(),
                    mSpinBoxScannerPositionY->value(),
                    mSpinBoxScannerPositionZ->value()
                    )
            );

    // set scanner orientation from form values
    Ogre::Quaternion pitch(Ogre::Degree(mSpinBoxScannerRotationPitch->value()), Ogre::Vector3::UNIT_X);
    Ogre::Quaternion roll(Ogre::Degree(mSpinBoxScannerRotationRoll->value()), Ogre::Vector3::UNIT_Z);
    Ogre::Quaternion yaw(Ogre::Degree(mSpinBoxScannerRotationYaw->value()), Ogre::Vector3::UNIT_Y);
    newLaserScanner->setOrientation(pitch * roll * yaw);

    mSimulator->getLaserScannerList()->append(newLaserScanner);

    mSpinBoxScannerIndex->setMaximum(mSpinBoxScannerIndex->maximum() + 1);
    mSpinBoxScannerIndex->setValue(mSpinBoxScannerIndex->maximum());
    // TODO: does setValue emit valueChanged? If not, call slotScannerIndexChanged here.

    qDebug() << "SimulationControlWidget::slotScannerCreate(): done.";
}

void SimulationControlWidget::slotScannerDelete(void)
{
    qDebug() << "SimulationControlWidget::slotScannerDelete(): deleting scanner at" << mSpinBoxScannerIndex->value();
    mSimulator->getLaserScannerList()->at(mSpinBoxScannerIndex->value())->deleteLater();
    // I hope that QSpinBox would change its value after value() > maximum...
    mSpinBoxScannerIndex->setMaximum(mSpinBoxScannerIndex->maximum() - 1);

    // adjust form to a valid scanner
    slotScannerIndexChanged();

    qDebug() << "SimulationControlWidget::slotScannerDelete(): done";
}

void SimulationControlWidget::slotScannerSaveConfiguration(void)
{
    qDebug() << "SimulationControlWidget::slotScannerSaveConfiguration()";

    QList<LaserScanner*> *scanners = mSimulator->getLaserScannerList();

    // Delete old scanners-section, as that might still contain more scanners than what we're about to write
    mSettings.remove("scanners");

    mSettings.beginWriteArray("scanners");

    for(int i = 0; i < scanners->size(); ++i)
    {
        // write settings for this scanner into config file
        qDebug() << "SimulationControlWidget::slotScannerSaveConfiguration(): now saving scanner-config" << i;
        mSettings.setArrayIndex(i);

        mSettings.setValue("range", scanners->at(i)->range());
        mSettings.setValue("speed", scanners->at(i)->speed());
        mSettings.setValue("angleStart", scanners->at(i)->angleStart());
        mSettings.setValue("angleStop", scanners->at(i)->angleStop());
        mSettings.setValue("angleStep", scanners->at(i)->angleStep());

        mSettings.setValue("posX", scanners->at(i)->getPosition().x);
        mSettings.setValue("posY", scanners->at(i)->getPosition().y);
        mSettings.setValue("posZ", scanners->at(i)->getPosition().z);

        mSettings.setValue("rotPitch", scanners->at(i)->getOrientation().getPitch().valueDegrees());
        mSettings.setValue("rotRoll", scanners->at(i)->getOrientation().getRoll().valueDegrees());
        mSettings.setValue("rotYaw", scanners->at(i)->getOrientation().getYaw().valueDegrees());
//        mSettings.endGroup();
    }

    mSettings.endArray();

    mSettings.sync();

    qDebug() << "SimulationControlWidget::slotScannerSaveConfiguration(): done";
}

void SimulationControlWidget::slotScannerLoadConfiguration(void)
{
    QList<LaserScanner*> *laserScanners = mSimulator->getLaserScannerList();

    // delete all old laserscanners
    while(laserScanners->size()) laserScanners->takeFirst()->deleteLater();

    const int numberOfLaserScanners = mSettings.beginReadArray("scanners");

    mSpinBoxScannerIndex->setMaximum(numberOfLaserScanners - 1);

    for(int i = 0; i < numberOfLaserScanners; ++i)
    {
        mSettings.setArrayIndex(i);

        qDebug() << "SimulationControlWidget::slotScannerLoadConfiguration(): creating scanner" << i << "from configuration";

        // By default, laserscanners are slooow
        LaserScanner* newLaserScanner = new LaserScanner(
                mSimulator,
                mOgreWidget,
                mSettings.value("range", 20.0).toReal(),
                mSettings.value("speed", 100).toInt(),
                mSettings.value("angleStart", 45).toInt(),
                mSettings.value("angleStop", 315).toInt(),
                mSettings.value("angleStep", 0.25).toInt());

        // set scanner position from configuration
        newLaserScanner->setPosition(
                Ogre::Vector3(
                        mSettings.value("posX", 0.0).toReal(),
                        mSettings.value("posY", 0.0).toReal(),
                        mSettings.value("posZ", 0.0).toReal()
                        )
                );

        // set scanner orientation from form values
        Ogre::Quaternion pitch(Ogre::Degree(mSettings.value("rotPitch", 0.0).toReal()), Ogre::Vector3::UNIT_X);
        Ogre::Quaternion roll(Ogre::Degree(mSettings.value("rotRoll", 0.0).toReal()), Ogre::Vector3::UNIT_Z);
        Ogre::Quaternion yaw(Ogre::Degree(mSettings.value("rotYaw", 0.0).toReal()), Ogre::Vector3::UNIT_Y);
        newLaserScanner->setOrientation(pitch * roll * yaw);

        laserScanners->append(newLaserScanner);
    }
    mSettings.endArray();

    // TODO: Will this load the settings?
    qDebug() << "SimulationControlWidget::slotScannerLoadConfiguration(): done, setting index spinbox to maximum: " << mSpinBoxScannerIndex->maximum();
    mSpinBoxScannerIndex->setValue(mSpinBoxScannerIndex->maximum());
}

double SimulationControlWidget::getTimeFactor()
{
    return mSpinBoxTimeFactor->value();
}

SimulationControlWidget::~SimulationControlWidget()
{
//    delete ui;
}
