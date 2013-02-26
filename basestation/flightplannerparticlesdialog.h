#ifndef FLIGHTPLANNERPARTICLESDIALOG_H
#define FLIGHTPLANNERPARTICLESDIALOG_H

#include <QDialog>
#include "particleskernel.cuh"
#include "ui_flightplannerparticlesdialog.h"

namespace Ui {
    class FlightPlannerParticlesDialog;
}

class FlightPlannerParticlesDialog : public QDialog
{
    Q_OBJECT

private:
    Ui::FlightPlannerParticlesDialog *ui;
    SimulationParameters mSimulationParameters;

public:
    explicit FlightPlannerParticlesDialog(const SimulationParameters* const sp, QWidget *parent = 0);
    ~FlightPlannerParticlesDialog();

    SimulationParameters getSimulationParameters() const {return mSimulationParameters;}

    bool processPhysics() const {return ui->mChkBoxProcessPhysics->isChecked();}
    void setProcessPhysics(bool state) {ui->mChkBoxProcessPhysics->setChecked(state);}

    bool followVehicle() const {return ui->mChkBoxFollowVehicle->isChecked();}
    bool showParticles() const {return ui->mChkBoxShowParticles->isChecked();}
    bool showWaypointPressure() const {return ui->mChkBoxShowWaypointPressure->isChecked();}

private slots:
    void slotSimulationParametersChanged();

public slots:
    void slotSetInitialValues(const SimulationParameters* const sp);

signals:
    void simulationParameters(const SimulationParameters*);

    void processPhysicsChanged(bool);
    void followVehicleChanged(bool);
    void showParticlesChanged(bool);
    void showWaypointPressureChanged(bool);

    void resetParticles();
    void resetWaypointPressure();
    void generateWayPoints();
    void deleteWayPoints();
};

#endif // FLIGHTPLANNERPARTICLESDIALOG_H