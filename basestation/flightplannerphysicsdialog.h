#ifndef FLIGHTPLANNERPHYSICSDIALOG_H
#define FLIGHTPLANNERPHYSICSDIALOG_H

#include <QDialog>
#include <QVector3D>

namespace Ui {
    class FlightPlannerPhysicsDialog;
}

class FlightPlannerPhysicsDialog : public QDialog
{
    Q_OBJECT


public:
    explicit FlightPlannerPhysicsDialog(QWidget *parent = 0);
    ~FlightPlannerPhysicsDialog();

    enum GenerationType {GenerateRain, GenerateShootFromVehicle};

    float getSampleSphereRadius() const;
    float getFrictionGround() const;
    float getFrictionSampleGeometry() const;
    bool visualizationActive() const;
    GenerationType getGenerationType() const;
    quint16 getEmitCount() const;
    QVector3D getEmitVelocity() const;

private:
    Ui::FlightPlannerPhysicsDialog *ui;

private slots:
    void slotGravityChanged();
    void slotFrictionChanged();

public slots:
    void slotSetProgress(const int& value, const int& min, const int& max);
    void slotAppendMessage(const QString& message);

signals:
    void createSampleGeometry();
    void deleteSampleGeometry();
    void processPhysics(bool);
    void gravityChanged(QVector3D);
    void frictionChanged(float,float);
    void submitWayPoints();
};

#endif // FLIGHTPLANNERPHYSICSDIALOG_H
