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

    bool isProcessPhysicsActive() const {return ui->mChkBoxProcessPhysics->isChecked();}
    void setProcessPhysicsActive(bool state) {ui->mChkBoxProcessPhysics->setChecked(state); emit processPhysics(state);}

    bool followVehicle() const {return ui->mChkBoxFollowVehicle->isChecked();}

private slots:
    void slotSimulationParametersChanged();

public slots:
    void slotSetInitialValues(const ParametersParticleSystem* const sp);
    void slotSetPointCloudSizeDense(const quint32 size);
    void slotSetPointCloudSizeSparse(const quint32 size);

signals:
    void simulationParameters(const ParametersParticleSystem*);

    void processPhysics(bool);
    void followVehicle(bool);
    void renderParticles(bool);
    void renderInformationGain(bool);
    void renderOccupancyGrid(bool);
    void renderPathPlannerGrid(bool);

    void reduceColliderCloud();
    void resetParticles();
    void resetInformationGain();
    void generateWayPoints();
};

#endif // FLIGHTPLANNERPARTICLESDIALOG_H
