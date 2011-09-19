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
    float getRestitutionGround() const;
    float getRestitutionSampleGeometry() const;
    bool visualizationActive() const;
    GenerationType getGenerationType() const;
    quint16 getEmitCount() const;
    QVector3D getEmitVelocity() const;
    bool useRelativeVelocity() const;

private:
    Ui::FlightPlannerPhysicsDialog *ui;

private slots:
    void slotGravityChanged();
    void slotFrictionChanged();
    void slotRestitutionChanged();

public slots:
    void slotSetProgress(const int& value, const int& min, const int& max);
    void slotAppendMessage(const QString& message);

signals:
    void createSampleGeometry();
    void deleteSampleGeometry();
    void createSafePath();
    void processPhysics(bool);
    void gravityChanged(QVector3D);
    void frictionChanged(float,float);
    void restitutionChanged(float,float);
    void submitWayPoints();
    void deleteWayPoints();
};

#endif // FLIGHTPLANNERPHYSICSDIALOG_H
