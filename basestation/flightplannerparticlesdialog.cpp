#include "flightplannerparticlesdialog.h"
#include <QDebug>
FlightPlannerParticlesDialog::FlightPlannerParticlesDialog(const ParametersParticleSystem* const sp, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::FlightPlannerParticlesDialog)
{
    ui->setupUi(this);

    connect(ui->mBtnResetParticles, SIGNAL(clicked()), SIGNAL(resetParticles()));
    connect(ui->mBtnResetInformationGain, SIGNAL(clicked()), SIGNAL(resetInformationGain()));
    connect(ui->mBtnGenerateWaypoints, SIGNAL(clicked()), SIGNAL(generateWayPoints()));

    connect(ui->mBtnReduceColliderCloud, SIGNAL(clicked()), SIGNAL(reduceColliderCloud()));

    connect(ui->mChkBoxProcessPhysics, SIGNAL(clicked(bool)), SIGNAL(processPhysics(bool)));
    connect(ui->mChkBoxFollowVehicle, SIGNAL(clicked(bool)), SIGNAL(followVehicle(bool)));

    connect(ui->mChkBoxCreateWayPoints, SIGNAL(clicked(bool)), SIGNAL(createWayPoints(bool)));
    connect(ui->mChkBoxCheckWayPointSafety, SIGNAL(clicked(bool)), SIGNAL(checkWayPointSafety(bool)));

    connect(ui->mChkBoxRenderParticles, SIGNAL(clicked(bool)), SIGNAL(renderParticles(bool)));
    connect(ui->mChkBoxRenderInformationGain, SIGNAL(clicked(bool)), SIGNAL(renderInformationGain(bool)));
    connect(ui->mChkBoxRenderGridOccupancy, SIGNAL(clicked(bool)), SIGNAL(renderOccupancyGrid(bool)));
    connect(ui->mChkBoxRenderGridPathPlanner, SIGNAL(clicked(bool)), SIGNAL(renderPathPlannerGrid(bool)));

    connect(ui->mSpinBoxTimeStepInner, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxTimeStepOuter, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxAttraction, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxColliderRadius, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxCollisionBoundary, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxCollisionParticle, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxCollisionCollider, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxDamping, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxParticleRadius, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxShear, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxSpring, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxGravityX, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxGravityY, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));
    connect(ui->mSpinBoxGravityZ, SIGNAL(valueChanged(double)), SLOT(slotSimulationParametersChanged()));

    slotSetInitialValues(sp);
}

void FlightPlannerParticlesDialog::slotSetInitialValues(const ParametersParticleSystem* const sp)
{
    ui->mSpinBoxTimeStepInner->setValue(sp->timeStepInner);
    ui->mSpinBoxTimeStepOuter->setValue(sp->timeStepOuter);
    ui->mSpinBoxAttraction->setValue(sp->attraction);
    ui->mSpinBoxColliderRadius->setValue(sp->colliderRadius);
    ui->mSpinBoxCollisionBoundary->setValue(sp->velocityFactorCollisionBoundary);
    ui->mSpinBoxCollisionParticle->setValue(sp->velocityFactorCollisionParticle);
    ui->mSpinBoxCollisionCollider->setValue(sp->velocityFactorCollisionCollider);
    ui->mSpinBoxDamping->setValue(sp->dampingMotion);
    ui->mSpinBoxParticleRadius->setValue(sp->particleRadius);
    ui->mSpinBoxShear->setValue(sp->shear);
    ui->mSpinBoxSpring->setValue(sp->spring);
    ui->mSpinBoxGravityX->setValue(sp->gravity.x);
    ui->mSpinBoxGravityY->setValue(sp->gravity.y);
    ui->mSpinBoxGravityZ->setValue(sp->gravity.z);
}

FlightPlannerParticlesDialog::~FlightPlannerParticlesDialog()
{
    delete ui;
}

void FlightPlannerParticlesDialog::slotSimulationParametersChanged()
{
    mSimulationParameters.timeStepInner = ui->mSpinBoxTimeStepInner->value();
    mSimulationParameters.timeStepOuter = ui->mSpinBoxTimeStepOuter->value();
    mSimulationParameters.attraction = ui->mSpinBoxAttraction->value();
    mSimulationParameters.colliderRadius = ui->mSpinBoxColliderRadius->value();
    mSimulationParameters.velocityFactorCollisionBoundary = ui->mSpinBoxCollisionBoundary->value();
    mSimulationParameters.velocityFactorCollisionParticle = ui->mSpinBoxCollisionParticle->value();
    mSimulationParameters.velocityFactorCollisionCollider = ui->mSpinBoxCollisionCollider->value();
    mSimulationParameters.dampingMotion = ui->mSpinBoxDamping->value();
    mSimulationParameters.particleRadius = ui->mSpinBoxParticleRadius->value();
    mSimulationParameters.shear = ui->mSpinBoxShear->value();
    mSimulationParameters.spring = ui->mSpinBoxSpring->value();
    mSimulationParameters.gravity.x = ui->mSpinBoxGravityX->value();
    mSimulationParameters.gravity.y = ui->mSpinBoxGravityY->value();
    mSimulationParameters.gravity.z = ui->mSpinBoxGravityZ->value();

    emit simulationParameters(&mSimulationParameters);
}

void FlightPlannerParticlesDialog::slotSetPointCloudSizeDense(const quint32 size)
{
    static QLocale locale(QLocale::English, QLocale::UnitedStates);
    ui->mLabelPointCloudSizeDense->setText(locale.toString(size));
}

void FlightPlannerParticlesDialog::slotSetPointCloudSizeSparse(const quint32 size)
{
    static QLocale locale(QLocale::English, QLocale::UnitedStates);
    ui->mLabelPointCloudSizeSparse->setText(locale.toString(size));
}
