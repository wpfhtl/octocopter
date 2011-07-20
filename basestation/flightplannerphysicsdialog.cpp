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
