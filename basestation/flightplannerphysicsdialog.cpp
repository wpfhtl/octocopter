#include "flightplannerphysicsdialog.h"
#include "ui_flightplannerphysicsdialog.h"

FlightPlannerPhysicsDialog::FlightPlannerPhysicsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::FlightPlannerPhysicsDialog)
{
    ui->setupUi(this);

    connect(ui->mBtnCreateGeometry, SIGNAL(clicked()), SIGNAL(createSampleGeometry()));
    connect(ui->mBtnDeleteGeometry, SIGNAL(clicked()), SIGNAL(deleteSampleGeometry()));
    connect(ui->mBtnProcessPhysics, SIGNAL(clicked(bool)), SIGNAL(processPhysics(bool)));
    connect(ui->mBtnSubmitWaypoints, SIGNAL(clicked()), SIGNAL(submitWayPoints()));

    connect(ui->mSpinBoxGravityX, SIGNAL(valueChanged(double)), SLOT(slotGravityChanged()));
    connect(ui->mSpinBoxGravityY, SIGNAL(valueChanged(double)), SLOT(slotGravityChanged()));
    connect(ui->mSpinBoxGravityZ, SIGNAL(valueChanged(double)), SLOT(slotGravityChanged()));

    connect(ui->mSpinBoxFrictionGround, SIGNAL(valueChanged(double)), SLOT(slotFrictionChanged()));
    connect(ui->mSpinBoxFrictionSampleGeometry, SIGNAL(valueChanged(double)), SLOT(slotFrictionChanged()));
}

FlightPlannerPhysicsDialog::~FlightPlannerPhysicsDialog()
{
    delete ui;
}

float FlightPlannerPhysicsDialog::getSampleSphereRadius() const
{
    return ui->mSpinBoxSphereRadius->value();
}

void FlightPlannerPhysicsDialog::slotSetProgress(const int& value, const int& min, const int& max)
{
    ui->mProgressBarPhysics->setRange(min, max);
    ui->mProgressBarPhysics->setValue(value);
}

void FlightPlannerPhysicsDialog::slotAppendMessage(const QString& message)
{
    ui->mTextBrowser->append(message/* + "\n"*/);
}

void FlightPlannerPhysicsDialog::slotGravityChanged()
{
    emit gravityChanged(
                QVector3D(
                    ui->mSpinBoxGravityX->value(),
                    ui->mSpinBoxGravityY->value(),
                    ui->mSpinBoxGravityZ->value()
                    )
                );
}

void FlightPlannerPhysicsDialog::slotFrictionChanged()
{
    emit frictionChanged(ui->mSpinBoxFrictionGround->value(),ui->mSpinBoxFrictionSampleGeometry->value());
}

float FlightPlannerPhysicsDialog::getFrictionGround() const
{
    return ui->mSpinBoxFrictionGround->value();
}

float FlightPlannerPhysicsDialog::getFrictionSampleGeometry() const
{
    return ui->mSpinBoxFrictionSampleGeometry->value();
}

bool FlightPlannerPhysicsDialog::visualizationActive() const
{
    return ui->mChkBoxVisualize->isChecked();
}

FlightPlannerPhysicsDialog::GenerationType FlightPlannerPhysicsDialog::getGenerationType() const
{
    if(ui->mRadioBtnFillSky->isChecked())
        return GenerateRain;
    else
        return GenerateShootFromVehicle;
}

quint16 FlightPlannerPhysicsDialog::getEmitCount() const
{
    return ui->mSpinBoxEmitCount->value();
}

QVector3D FlightPlannerPhysicsDialog::getEmitVelocity() const
{
    return QVector3D(ui->mSpinBoxEmitVelocityFront->value(), ui->mSpinBoxEmitVelocityRight->value(), ui->mSpinBoxEmitVelocityUp->value());
}
