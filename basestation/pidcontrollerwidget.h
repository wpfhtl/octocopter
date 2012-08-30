#ifndef PIDCONTROLLERWIDGET_H
#define PIDCONTROLLERWIDGET_H

#include <QDockWidget>
#include <QMessageBox>
#include <QString>
#include <QMap>

#include <flightcontrollervalues.h>

namespace Ui {
class PidControllerWidget;
}

class PidControllerWidget : public QDockWidget
{
    Q_OBJECT
    
public:
    PidControllerWidget(QWidget *parent = 0);
    ~PidControllerWidget();

    QWidget* mParent;
    QMap<QString, const PidController*> mControllers;

    void setControllers(const FlightControllerValues *const fcv);

    bool isPopulated() const {return mPopulated;}


signals:
    // signals that a weight for a single controller was changed. Transmits all the weights.
    void controllerWeight(QString, QMap<QString, float>);
    
private:
    Ui::PidControllerWidget *ui;
    bool mPopulated;

private slots:
    void slotWeightChanged(int row, int column);

public slots:
    void setEnabled(const bool enabled);
    void slotRebuild();
//    void slotClear() {mPopulated = false;}
};

#endif
