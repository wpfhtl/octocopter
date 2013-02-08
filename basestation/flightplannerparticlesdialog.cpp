#include "flightplannerparticlesdialog.h"

FlightPlannerParticlesDialog::FlightPlannerParticlesDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::FlightPlannerParticlesDialog)
{
    ui->setupUi(this);

    connect(ui->mBtnResetParticles, SIGNAL(clicked()), SIGNAL(resetParticles()));
    connect(ui->mBtnResetWaypointPressure, SIGNAL(clicked()), SIGNAL(resetWaypointPressure()));
    connect(ui->mBtnGenerateWaypoints, SIGNAL(clicked()), SIGNAL(generateWayPoints()));
    connect(ui->mBtnDeleteWaypoints, SIGNAL(clicked()), SIGNAL(deleteWayPoints()));

    connect(ui->mChkBoxProcessPhysics, SIGNAL(clicked(bool)), SIGNAL(processPhysicsChanged(bool)));
    connect(ui->mChkBoxFollowVehicle, SIGNAL(clicked(bool)), SIGNAL(followVehicleChanged(bool)));
    connect(ui->mChkBoxShowParticles, SIGNAL(clicked(bool)), SIGNAL(showParticlesChanged(bool)));
    connect(ui->mChkBoxShowWaypointPressure, SIGNAL(clicked(bool)), SIGNAL(showWaypointPressureChanged(bool)));

    connect(ui->mSpinBoxAttraction, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxCollisionBoundary, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxCollisionParticle, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxDamping, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxParticleRadius, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxShear, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxSpring, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxGravityX, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxGravityY, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxGravityZ, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
}

FlightPlannerParticlesDialog::~FlightPlannerParticlesDialog()
{
    delete ui;
}

void FlightPlannerParticlesDialog::slotSimulationParametersChanged()
{
    mSimulationParameters.attraction = ui->mSpinBoxAttraction->value();
    mSimulationParameters.velocityFactorCollisionBoundary = ui->mSpinBoxCollisionBoundary->value();
    mSimulationParameters.velocityFactorCollisionParticle = ui->mSpinBoxCollisionParticle->value();
    mSimulationParameters.dampingMotion = ui->mSpinBoxDamping->value();
    mSimulationParameters.particleRadius = ui->mSpinBoxParticleRadius->value();
    mSimulationParameters.shear = ui->mSpinBoxShear->value();
    mSimulationParameters.spring = ui->mSpinBoxSpring->value();
    mSimulationParameters.gravity.x = ui->mSpinBoxGravityX->value();
    mSimulationParameters.gravity.y = ui->mSpinBoxGravityY->value();
    mSimulationParameters.gravity.z = ui->mSpinBoxGravityZ->value();

    emit simulationParameters(&mSimulationParameters);
}
