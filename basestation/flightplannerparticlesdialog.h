#ifndef FLIGHTPLANNERPARTICLESDIALOG_H
#define FLIGHTPLANNERPARTICLESDIALOG_H

#include <QDialog>
#include "pointcloudcuda.h"
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

    bool mReactToChangesFromUi;

public:
    explicit FlightPlannerParticlesDialog(QWidget *parent = 0);
    ~FlightPlannerParticlesDialog();

    ParametersParticleSystem getSimulationParameters() const {return mSimulationParameters;}

    bool isProcessPhysicsActive() const {return ui->mChkBoxProcessPhysics->isChecked();}
    void setProcessPhysicsActive(bool state) {ui->mChkBoxProcessPhysics->setChecked(state); emit processPhysics(state);}

    bool followVehicle() const {return ui->mChkBoxFollowVehicle->isChecked();}
    bool createWayPoints() const {return ui->mChkBoxCreateWayPoints->isChecked();}
    bool checkWayPointSafety() const  {return ui->mChkBoxCheckWayPointSafety->isChecked();}

private slots:
    void slotSimulationParametersChangedThroughUi();

public slots:
    void slotSetValuesFromStruct(const ParametersParticleSystem* const sp);
    void slotSetPointCloudParametersDense(const ParametersPointCloud* const p);
    void slotSetPointCloudParametersSparse(const ParametersPointCloud* const p);
    void slotSetRenderParticles(const bool value) {ui->mChkBoxRenderParticles->setChecked(value);}
    void slotSetRenderInformationGain(const bool value) {ui->mChkBoxRenderInformationGain->setChecked(value);}
    void slotSetRenderOccupancyGrid(const bool value) {ui->mChkBoxRenderGridOccupancy->setChecked(value);}

signals:
    void simulationParameters(const ParametersParticleSystem* const);
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
