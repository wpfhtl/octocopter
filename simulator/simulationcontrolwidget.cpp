#include "simulationcontrolwidget.h"

SimulationControlWidget::SimulationControlWidget(Simulator *simulator, OgreWidget *ogreWidget) :
    QDockWidget(simulator),
//    ui(new Ui::SimulationControlWidget),
    mSettings("BenAdler", "simulator")
{
    /*ui->*/setupUi(this);

    mSimulator = simulator;
    mOgreWidget = ogreWidget;

    // Wire up the time-box
    connect(mSpinBoxTimeFactor, SIGNAL(valueChanged(double)), SIGNAL(timeFactorChanged(double)));
    connect(mBtnStart, SIGNAL(clicked()), SIGNAL(start()));
    connect(mBtnPause, SIGNAL(clicked()), SIGNAL(pause()));

    // Wire up the laserscanner-box
    connect(mSpinBoxScannerIndex, SIGNAL(valueChanged(int)), SLOT(slotScannerIndexChanged()));

    connect(mSpinBoxPropertiesRange, SIGNAL(valueChanged(double)), SLOT(slotScannerSettingsChanged()));
    connect(mSpinBoxPropertiesSpeed, SIGNAL(valueChanged(double)), SLOT(slotScannerSettingsChanged()));
    connect(mSpinBoxPropertiesAngleStart, SIGNAL(valueChanged(double)), SLOT(slotScannerSettingsChanged()));
    connect(mSpinBoxPropertiesAngleStop, SIGNAL(valueChanged(double)), SLOT(slotScannerSettingsChanged()));
    connect(mSpinBoxPropertiesAngleStep, SIGNAL(valueChanged(double)), SLOT(slotScannerSettingsChanged()));

    connect(mSpinBoxScannerPositionX, SIGNAL(valueChanged(double)), SLOT(slotScannerSettingsChanged()));
    connect(mSpinBoxScannerPositionY, SIGNAL(valueChanged(double)), SLOT(slotScannerSettingsChanged()));
    connect(mSpinBoxScannerPositionZ, SIGNAL(valueChanged(double)), SLOT(slotScannerSettingsChanged()));
    connect(mSpinBoxScannerRotationPitch, SIGNAL(valueChanged(double)), SLOT(slotScannerSettingsChanged()));
    connect(mSpinBoxScannerRotationRoll, SIGNAL(valueChanged(double)), SLOT(slotScannerSettingsChanged()));
    connect(mSpinBoxScannerRotationYaw, SIGNAL(valueChanged(double)), SLOT(slotScannerSettingsChanged()));

    connect(mBtnLaserScannerCreate, SIGNAL(clicked()), SLOT(slotScannerCreate()));
    connect(mBtnLaserScannerDelete, SIGNAL(clicked()), SLOT(slotScannerDelete()));
    connect(mBtnLaserScannerSaveConfiguration, SIGNAL(clicked()), SLOT(slotScannerSaveConfiguration()));

    createScannersFromConfiguration();
}

void SimulationControlWidget::createScannersFromConfiguration(void)
{
    const int numberOfLaserScanners = mSettings.beginReadArray("scanners");

    mSpinBoxScannerIndex->setMaximum(numberOfLaserScanners - 1);

    for(int i = 0; i < numberOfLaserScanners; ++i)
    {
        mSettings.setArrayIndex(i);


        LaserScanner* newLaserScanner = new LaserScanner(
                mSimulator,
                mOgreWidget,
                mSettings.value("range").toReal(),
                mSettings.value("speed").toInt(),
                mSettings.value("angleStart").toInt(),
                mSettings.value("angleStop").toInt(),
                mSettings.value("angleStep").toInt());

        mSimulator->getLaserScannerList()->append(newLaserScanner);
    }
    mSettings.endArray();

    // TODO: Will this load the settings?
    mSpinBoxScannerIndex->setValue(mSpinBoxScannerIndex->maximum());
}

void SimulationControlWidget::slotScannerIndexChanged(void)
{
    // Load that scanner's data into the widget

    // Get Scanner and its SceneNode
    LaserScanner *currentScanner = mSimulator->getLaserScannerList()->at(mSpinBoxScannerIndex->value());
    Ogre::SceneNode *currentScannerNode = currentScanner->getSceneNode();

    mSpinBoxPropertiesRange->setValue(currentScanner->range());
    mSpinBoxPropertiesSpeed->setValue(currentScanner->speed());
    mSpinBoxPropertiesAngleStart->setValue(currentScanner->angleStart());
    mSpinBoxPropertiesAngleStop->setValue(currentScanner->angleStop());
    mSpinBoxPropertiesAngleStep->setValue(currentScanner->angleStep());

    mSpinBoxScannerPositionX->setValue(currentScannerNode->getPosition().x);
    mSpinBoxScannerPositionY->setValue(currentScannerNode->getPosition().y);
    mSpinBoxScannerPositionZ->setValue(currentScannerNode->getPosition().z);

    mSpinBoxScannerRotationPitch->setValue(currentScannerNode->getOrientation().getPitch().valueDegrees());
    mSpinBoxScannerRotationRoll->setValue(currentScannerNode->getOrientation().getRoll().valueDegrees());
    mSpinBoxScannerRotationYaw->setValue(currentScannerNode->getOrientation().getYaw().valueDegrees());
}

void SimulationControlWidget::slotScannerSettingsChanged(void)
{
    // The currently selected scanner has had its properties changed
    // in the form. Propagate the new values to the LaserScanner*

    // Get Scanner and its SceneNode
    LaserScanner *currentScanner = mSimulator->getLaserScannerList()->at(mSpinBoxScannerIndex->value());
    Ogre::SceneNode *currentScannerNode = currentScanner->getSceneNode();

    // set scanner properties from form values
    currentScanner->setRange(mSpinBoxPropertiesRange->value());
    currentScanner->setSpeed(mSpinBoxPropertiesSpeed->value());
    currentScanner->setAngleStart(mSpinBoxPropertiesAngleStart->value());
    currentScanner->setAngleStop(mSpinBoxPropertiesAngleStop->value());
    currentScanner->setAngleStep(mSpinBoxPropertiesAngleStep->value());

    // set scanner position from form values
    currentScannerNode->setPosition(
            mSpinBoxScannerPositionX->value(),
            mSpinBoxScannerPositionY->value(),
            mSpinBoxScannerPositionZ->value());

    // set scanner orientation from form values
    Ogre::Quaternion pitch(Ogre::Degree(mSpinBoxScannerRotationPitch->value()), Ogre::Vector3::UNIT_X);
    Ogre::Quaternion roll(Ogre::Degree(mSpinBoxScannerRotationRoll->value()), Ogre::Vector3::UNIT_Z);
    Ogre::Quaternion yaw(Ogre::Degree(mSpinBoxScannerRotationYaw->value()), Ogre::Vector3::UNIT_Y);
    currentScannerNode->setOrientation(pitch * roll * yaw);
}

void SimulationControlWidget::slotScannerCreate(void)
{
    LaserScanner* newLaserScanner = new LaserScanner(
            mSimulator,
            mOgreWidget,
            mSettings.value("range").toReal(),
            mSettings.value("speed").toInt(),
            mSettings.value("angleStart").toInt(),
            mSettings.value("angleStop").toInt(),
            mSettings.value("angleStep").toReal());

    mSimulator->getLaserScannerList()->append(newLaserScanner);

    mSpinBoxScannerIndex->setMaximum(mSpinBoxScannerIndex->maximum() + 1);
    mSpinBoxScannerIndex->setValue(mSpinBoxScannerIndex->maximum());
    // TODO: does setValue emit valueChanged? If not, call slotScannerIndexChanged here.
}

void SimulationControlWidget::slotScannerDelete(void)
{
    mSimulator->getLaserScannerList()->at(mSpinBoxScannerIndex->value())->deleteLater();
    // I hope that QSpinBox would change its value after value() > maximum...
    mSpinBoxScannerIndex->setMaximum(mSpinBoxScannerIndex->maximum() - 1);

    // adjust form to a valid scanner
    slotScannerIndexChanged();
}

void SimulationControlWidget::slotScannerSaveConfiguration(void)
{
    QList<LaserScanner*> *scanners = mSimulator->getLaserScannerList();

    // TODO: delete old scanners-section, as that might still
    // contain more scanners than what we're about to write

    mSettings.beginWriteArray("scanners");

    for(int i = 0; i < scanners->size(); ++i)
    {
        // write settings for this scanner into config file
        Ogre::SceneNode *currentScannerNode = scanners->at(i)->getSceneNode();
        mSettings.setArrayIndex(i);

        mSettings.setValue("range", mSpinBoxPropertiesRange->value());
        mSettings.setValue("speed", mSpinBoxPropertiesSpeed->value());
        mSettings.setValue("angleStart", mSpinBoxPropertiesAngleStart->value());
        mSettings.setValue("angleStop", mSpinBoxPropertiesAngleStop->value());
        mSettings.setValue("angleStep", mSpinBoxPropertiesAngleStep->value());

        mSettings.setValue("posX", currentScannerNode->getPosition().x);
        mSettings.setValue("posY", currentScannerNode->getPosition().y);
        mSettings.setValue("posZ", currentScannerNode->getPosition().z);

        mSettings.setValue("rotPitch", currentScannerNode->getOrientation().getPitch().valueDegrees());
        mSettings.setValue("rotRoll", currentScannerNode->getOrientation().getRoll().valueDegrees());
        mSettings.setValue("rotYaw", currentScannerNode->getOrientation().getYaw().valueDegrees());
        mSettings.endGroup();
    }

    mSettings.endArray();

    mSettings.sync();
}

SimulationControlWidget::~SimulationControlWidget()
{
//    delete ui;
}
