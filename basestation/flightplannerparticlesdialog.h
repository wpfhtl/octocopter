#ifndef FLIGHTPLANNERPARTICLESDIALOG_H
#define FLIGHTPLANNERPARTICLESDIALOG_H

#include <QDialog>
#include "parametersparticlesystem.cuh"
#include "ui_flightplannerparticlesdialog.h"

namespace Ui {
    class FlightPlannerParticlesDialog;
}

class FlightPlannerParticlesDialog : public QDialog
{
    Q_OBJECT

private:
    Ui::FlightPlannerParticlesDialog *ui;
    ParametersParticleSystem mSimulationParameters;

public:
    explicit FlightPlannerParticlesDialog(const ParametersParticleSystem* const sp, QWidget *parent = 0);
    ~FlightPlannerParticlesDialog();

    ParametersParticleSystem getSimulationParameters() const {return mSimulationParameters;}

    bool processPhysics() const {return ui->mChkBoxProcessPhysics->isChecked();}
    void setProcessPhysics(bool state) {ui->mChkBoxProcessPhysics->setChecked(state);}

    bool followVehicle() const {return ui->mChkBoxFollowVehicle->isChecked();}
    bool showParticles() const {return ui->mChkBoxShowParticles->isChecked();}
    bool showInformationGain() const {return ui->mChkBoxShowInformationGain->isChecked();}

private slots:
    void slotSimulationParametersChanged();

public slots:
    void slotSetInitialValues(const ParametersParticleSystem* const sp);

signals:
    void simulationParameters(const ParametersParticleSystem*);

    void processPhysicsChanged(bool);
    void followVehicleChanged(bool);
    void showParticlesChanged(bool);
    void showInformationGainChanged(bool);
    void showOccupancyGridChanged(bool);
    void showPathFinderGridChanged(bool);

    void resetParticles();
    void resetInformationGain();
    void generateWayPoints();
    void deleteWayPoints();
};

#endif // FLIGHTPLANNERPARTICLESDIALOG_H
